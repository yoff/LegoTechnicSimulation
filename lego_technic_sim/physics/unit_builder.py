"""Build rigid units and detect joints from an LDraw build.

Algorithm overview
------------------
1.  For every pair of parts whose axis-aligned bounding boxes (AABBs) are
    within *snap_threshold* LDU of each other, compute a list of *contact
    points* by finding vertices of one part that lie within *snap_threshold*
    of any vertex of the other.

2.  Use union-find to merge all mutually-touching parts into *units*.

3.  Connections that span two *different* units become *joints*.

    * Few contact points (< ``FIXED_CONTACT_MIN``) → REVOLUTE
      (single-point contact, characteristic of a Technic pin or axle –
      the "frictionless snap").
    * Many contact points (≥ ``FIXED_CONTACT_MIN``) → FIXED
      (surface contact, characteristic of stud-on-anti-stud or beam face).

4.  The joint axis is estimated via PCA of the contact-point cloud: the
    direction with the *smallest* variance (the normal to the contact plane)
    is taken as the rotation axis.

This is an approximate heuristic.  Callers may override joint types after the
fact.
"""

from __future__ import annotations

from typing import Dict, List, Set, Tuple

import numpy as np

from ..ldraw.model import LDrawBuild, LDrawPart
from .mesh_properties import (
    ABS_DENSITY_KG_PER_M3,
    LDU_TO_METERS,
    mesh_volume_and_com,
)
from .model import Joint, JointType, PhysicsScene, Unit

# Distance threshold in LDU for two bricks to be considered "snapped".
DEFAULT_SNAP_THRESHOLD_LDU: float = 4.0

# Minimum number of contact points for a joint to be classified as FIXED.
FIXED_CONTACT_MIN: int = 3


# ---------------------------------------------------------------------------
# Helper data structures
# ---------------------------------------------------------------------------


class _UnionFind:
    """Simple union-find (disjoint-set) with path compression and rank."""

    def __init__(self, n: int) -> None:
        self.parent = list(range(n))
        self.rank = [0] * n

    def find(self, x: int) -> int:
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]  # path halving
            x = self.parent[x]
        return x

    def union(self, x: int, y: int) -> None:
        rx, ry = self.find(x), self.find(y)
        if rx == ry:
            return
        if self.rank[rx] < self.rank[ry]:
            rx, ry = ry, rx
        self.parent[ry] = rx
        if self.rank[rx] == self.rank[ry]:
            self.rank[rx] += 1


# ---------------------------------------------------------------------------
# Geometry utilities
# ---------------------------------------------------------------------------


def _brick_aabb(part: LDrawPart) -> Tuple[np.ndarray, np.ndarray]:
    """Return the axis-aligned bounding box (min, max) of a part's mesh."""
    if not part.triangles:
        p = part.position
        return p.copy(), p.copy()
    verts = np.vstack([[t.v0, t.v1, t.v2] for t in part.triangles])
    return verts.min(axis=0), verts.max(axis=0)


def _aabbs_close(
    min_a: np.ndarray,
    max_a: np.ndarray,
    min_b: np.ndarray,
    max_b: np.ndarray,
    threshold: float,
) -> bool:
    """Return True if two AABBs overlap or are within *threshold* of each other."""
    return bool(
        np.all(min_a - threshold <= max_b) and np.all(min_b - threshold <= max_a)
    )


def _contact_points(
    part_a: LDrawPart,
    part_b: LDrawPart,
    threshold: float,
) -> List[np.ndarray]:
    """Find contact points between two parts.

    A contact point is the midpoint between a vertex of *part_a* and the
    nearest vertex of *part_b* that lies within *threshold* LDU of it.

    Returns an empty list if the parts have no vertices or no contacts.
    """
    if not part_a.triangles or not part_b.triangles:
        return []

    verts_a = np.unique(
        np.vstack([[t.v0, t.v1, t.v2] for t in part_a.triangles]), axis=0
    )
    verts_b = np.unique(
        np.vstack([[t.v0, t.v1, t.v2] for t in part_b.triangles]), axis=0
    )

    contacts: List[np.ndarray] = []
    for va in verts_a:
        dists = np.linalg.norm(verts_b - va, axis=1)
        close = verts_b[dists <= threshold]
        if len(close) > 0:
            contacts.append((va + close.mean(axis=0)) / 2.0)
    return contacts


def _estimate_joint_axis(contacts: List[np.ndarray]) -> np.ndarray:
    """Estimate the dominant rotation axis of a contact-point cloud.

    Uses PCA: the direction with the *smallest* variance (normal to the
    contact plane) is the most likely rotation axis for a revolute joint.
    Falls back to the world Y-axis for degenerate inputs.
    """
    if len(contacts) < 2:
        return np.array([0.0, 1.0, 0.0])
    pts = np.array(contacts, dtype=float)
    centered = pts - pts.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    axis: np.ndarray = vt[-1]  # last row = direction of least variance
    norm = float(np.linalg.norm(axis))
    return axis / norm if norm > 1e-12 else np.array([0.0, 1.0, 0.0])


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def build_units_and_joints(
    build: LDrawBuild,
    snap_threshold: float = DEFAULT_SNAP_THRESHOLD_LDU,
    density: float = ABS_DENSITY_KG_PER_M3,
    ldu_to_meters: float = LDU_TO_METERS,
) -> PhysicsScene:
    """Analyse *build* and return a :class:`~lego_technic_sim.physics.model.PhysicsScene`.

    Parameters
    ----------
    build:
        A parsed LDraw build (see :class:`~lego_technic_sim.ldraw.model.LDrawBuild`).
    snap_threshold:
        Distance in LDU within which two bricks are considered connected.
        Default is 4 LDU (≈ 1.6 mm), slightly larger than typical mesh gaps.
    density:
        Material density in kg/m³ used to compute unit masses.
    ldu_to_meters:
        Scale factor from LDU to metres.

    Returns
    -------
    PhysicsScene
        Contains :class:`~lego_technic_sim.physics.model.Unit` objects (one per
        connected component of the brick graph) and
        :class:`~lego_technic_sim.physics.model.Joint` objects (one per
        inter-unit connection).
    """
    parts = build.parts
    n = len(parts)
    if n == 0:
        return PhysicsScene()

    aabbs = [_brick_aabb(p) for p in parts]
    uf = _UnionFind(n)

    # (i, j, contacts) for every pair that is physically touching
    raw_connections: List[Tuple[int, int, List[np.ndarray]]] = []

    for i in range(n):
        for j in range(i + 1, n):
            if not _aabbs_close(*aabbs[i], *aabbs[j], snap_threshold):
                continue
            contacts = _contact_points(parts[i], parts[j], snap_threshold)
            if contacts:
                raw_connections.append((i, j, contacts))
                uf.union(i, j)

    # Group bricks into units via union-find
    unit_map: Dict[int, List[int]] = {}
    for i in range(n):
        root = uf.find(i)
        unit_map.setdefault(root, []).append(i)

    units: List[Unit] = []
    brick_to_unit: Dict[int, int] = {}

    for brick_indices in unit_map.values():
        unit_bricks = [parts[i] for i in brick_indices]

        # Combined mass and centre of mass (mass-weighted average)
        total_mass = 0.0
        weighted_com = np.zeros(3, dtype=float)
        for i in brick_indices:
            vol, com = mesh_volume_and_com(parts[i].triangles, ldu_to_meters)
            mass = density * vol
            total_mass += mass
            weighted_com += mass * com

        if total_mass > 0.0:
            com = weighted_com / total_mass
        else:
            # No geometry – fall back to mean part position
            com = np.mean(
                [parts[i].position * ldu_to_meters for i in brick_indices],
                axis=0,
            )

        unit_idx = len(units)
        units.append(Unit(bricks=unit_bricks, mass=total_mass, center_of_mass=com))
        for i in brick_indices:
            brick_to_unit[i] = unit_idx

    # Build joints between different units
    joints: List[Joint] = []
    seen_pairs: Set[Tuple[int, int]] = set()

    for i, j, contacts in raw_connections:
        ui = brick_to_unit[i]
        uj = brick_to_unit[j]
        if ui == uj:
            continue  # internal – same rigid body
        pair = (min(ui, uj), max(ui, uj))
        if pair in seen_pairs:
            continue
        seen_pairs.add(pair)

        position = np.mean(contacts, axis=0) * ldu_to_meters
        axis = _estimate_joint_axis(contacts)

        # Few contact points → single-point (pin/axle) → revolute
        joint_type = (
            JointType.FIXED
            if len(contacts) >= FIXED_CONTACT_MIN
            else JointType.REVOLUTE
        )
        joints.append(
            Joint(
                unit_a_index=ui,
                unit_b_index=uj,
                joint_type=joint_type,
                position=position,
                axis=axis,
            )
        )

    return PhysicsScene(units=units, joints=joints)
