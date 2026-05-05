"""Build rigid units and detect joints from an LDraw build.

Algorithm overview
------------------
Primary method: **Connector-based detection**

1.  Classify parts into *connectors* (pins, axles) and *structural* (beams,
    bricks, motors, gears, etc.).

2.  For each connector, determine which structural parts it joins by checking
    which structural part meshes overlap the connector's bounding box.

3.  Friction pins create *rigid* bonds (parts they connect form one unit).
    Frictionless pins and axles create *revolute joints* between units.

4.  Union-find groups structural parts connected by rigid bonds into *units*.

5.  Revolute connectors that span two different units become *joints*.
    The joint axis is derived from the connector's longest mesh extent
    (the insertion axis).

Fallback method: **Distance-based detection** (legacy)

    For builds with no recognised connectors, falls back to the original
    vertex-proximity heuristic.
"""

from __future__ import annotations

from typing import Dict, List, Set, Tuple

import numpy as np

from ..ldraw.model import LDrawBuild, LDrawPart
from .connectors import (
    creates_revolute_connection,
    creates_rigid_connection,
    is_connector,
)
from .mesh_properties import (
    ABS_DENSITY_KG_PER_M3,
    LDU_TO_METERS,
    mesh_volume_and_com,
)
from .model import Joint, JointType, PhysicsScene, Unit
from .motor_detection import detect_motors

# Distance threshold in LDU for two bricks to be considered "snapped".
DEFAULT_SNAP_THRESHOLD_LDU: float = 4.0

# Minimum number of contact points for a joint to be classified as FIXED.
FIXED_CONTACT_MIN: int = 3

# Margin in LDU added to connector bounding boxes when finding overlapping parts.
CONNECTOR_OVERLAP_MARGIN: float = 4.0


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


def _connector_axis(part: LDrawPart) -> np.ndarray:
    """Estimate the insertion axis of a connector from its mesh extent.

    The longest dimension of the connector's bounding box corresponds to
    the pin/axle shaft direction.  Falls back to world Y if no geometry.
    """
    if not part.triangles:
        return np.array([0.0, 1.0, 0.0])
    verts = np.vstack([[t.v0, t.v1, t.v2] for t in part.triangles])
    extent = verts.max(axis=0) - verts.min(axis=0)
    axis_idx = int(np.argmax(extent))
    axis = np.zeros(3)
    axis[axis_idx] = 1.0
    return axis


def _find_structural_overlaps(
    connector: LDrawPart,
    structural_parts: List[Tuple[int, LDrawPart]],
    margin: float = CONNECTOR_OVERLAP_MARGIN,
) -> List[int]:
    """Find structural part indices whose mesh overlaps the connector's bbox.

    Returns the global part indices of structural parts that have at least
    one vertex inside the connector's bounding box (expanded by *margin*).
    """
    if not connector.triangles:
        return []
    cverts = np.vstack([[t.v0, t.v1, t.v2] for t in connector.triangles])
    cmn = cverts.min(axis=0) - margin
    cmx = cverts.max(axis=0) + margin

    overlapping: List[int] = []
    for idx, sp in structural_parts:
        if not sp.triangles:
            continue
        sverts = np.vstack([[t.v0, t.v1, t.v2] for t in sp.triangles])
        inside = np.all((sverts >= cmn) & (sverts <= cmx), axis=1)
        if np.any(inside):
            overlapping.append(idx)
    return overlapping


def build_units_and_joints(
    build: LDrawBuild,
    snap_threshold: float = DEFAULT_SNAP_THRESHOLD_LDU,
    density: float = ABS_DENSITY_KG_PER_M3,
    ldu_to_meters: float = LDU_TO_METERS,
) -> PhysicsScene:
    """Analyse *build* and return a :class:`~lego_technic_sim.physics.model.PhysicsScene`.

    Uses connector-based detection (pins, axles) when recognised connector
    parts are present.  Falls back to distance-based vertex proximity for
    builds without connectors.

    Parameters
    ----------
    build:
        A parsed LDraw build (see :class:`~lego_technic_sim.ldraw.model.LDrawBuild`).
    snap_threshold:
        Distance in LDU within which two bricks are considered connected
        (used by the fallback distance-based method and for connector
        overlap margin).
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

    # Separate connectors from structural parts
    connector_indices: List[int] = []
    structural_indices: List[int] = []
    for i, p in enumerate(parts):
        if is_connector(p.part_id):
            connector_indices.append(i)
        else:
            structural_indices.append(i)

    # If we have connectors, use the connector-based approach
    if connector_indices:
        scene = _build_via_connectors(
            parts, connector_indices, structural_indices,
            density, ldu_to_meters, snap_threshold,
        )
    else:
        # Fallback: distance-based detection (original algorithm)
        scene = _build_via_distance(
            parts, snap_threshold, density, ldu_to_meters,
        )

    scene.motors = detect_motors(scene)
    return scene


def _build_via_connectors(
    parts: List[LDrawPart],
    connector_indices: List[int],
    structural_indices: List[int],
    density: float,
    ldu_to_meters: float,
    margin: float,
) -> PhysicsScene:
    """Build units and joints using connector part classification."""
    n_structural = len(structural_indices)
    if n_structural == 0:
        return PhysicsScene()

    # Map from global part index → local structural index
    global_to_local: Dict[int, int] = {
        gi: li for li, gi in enumerate(structural_indices)
    }

    uf = _UnionFind(n_structural)
    structural_tuples = [(i, parts[i]) for i in structural_indices]

    # Revolute connections: (local_i, local_j, connector_part)
    revolute_connections: List[Tuple[int, int, LDrawPart]] = []

    for ci in connector_indices:
        conn_part = parts[ci]
        overlapping = _find_structural_overlaps(
            conn_part, structural_tuples, margin=margin,
        )
        if len(overlapping) < 2:
            continue

        # This connector joins the overlapping structural parts
        if creates_rigid_connection(conn_part.part_id):
            # Friction pin → merge all overlapping into one unit
            first_local = global_to_local[overlapping[0]]
            for gi in overlapping[1:]:
                uf.union(first_local, global_to_local[gi])
        elif creates_revolute_connection(conn_part.part_id):
            # Frictionless pin / axle → potential revolute joint
            # Store all pairs for later (after units are formed)
            for k in range(len(overlapping)):
                for m in range(k + 1, len(overlapping)):
                    revolute_connections.append((
                        global_to_local[overlapping[k]],
                        global_to_local[overlapping[m]],
                        conn_part,
                    ))

    # Build units from union-find groups
    unit_map: Dict[int, List[int]] = {}
    for li in range(n_structural):
        root = uf.find(li)
        unit_map.setdefault(root, []).append(li)

    units: List[Unit] = []
    local_to_unit: Dict[int, int] = {}

    for local_indices in unit_map.values():
        global_indices = [structural_indices[li] for li in local_indices]
        unit_bricks = [parts[gi] for gi in global_indices]

        total_mass = 0.0
        weighted_com = np.zeros(3, dtype=float)
        for gi in global_indices:
            vol, com = mesh_volume_and_com(parts[gi].triangles, ldu_to_meters)
            mass = density * vol
            total_mass += mass
            weighted_com += mass * com

        if total_mass > 0.0:
            com = weighted_com / total_mass
        else:
            com = np.mean(
                [parts[gi].position * ldu_to_meters for gi in global_indices],
                axis=0,
            )

        unit_idx = len(units)
        units.append(Unit(bricks=unit_bricks, mass=total_mass, center_of_mass=com))
        for li in local_indices:
            local_to_unit[li] = unit_idx

    # Build revolute joints from frictionless/axle connections
    joints: List[Joint] = []
    seen_pairs: Set[Tuple[int, int]] = set()

    for li_a, li_b, conn_part in revolute_connections:
        ui = local_to_unit[li_a]
        uj = local_to_unit[li_b]
        if ui == uj:
            continue  # same rigid unit — no joint
        pair = (min(ui, uj), max(ui, uj))
        if pair in seen_pairs:
            continue
        seen_pairs.add(pair)

        position = conn_part.position * ldu_to_meters
        axis = _connector_axis(conn_part)

        joints.append(
            Joint(
                unit_a_index=ui,
                unit_b_index=uj,
                joint_type=JointType.REVOLUTE,
                position=position,
                axis=axis,
            )
        )

    return PhysicsScene(units=units, joints=joints)


def _build_via_distance(
    parts: List[LDrawPart],
    snap_threshold: float,
    density: float,
    ldu_to_meters: float,
) -> PhysicsScene:
    """Fallback: build units and joints using vertex-proximity heuristic."""
    n = len(parts)
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
