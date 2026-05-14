"""Build rigid units and detect joints from an LDraw build.

Algorithm overview
------------------
Primary method: **Port-based connection detection**

1.  Classify parts into *connectors* (pins, axles) and *structural* (beams,
    bricks, motors, gears, etc.).

2.  For each connector, compute its shaft line (world-space endpoints + axis).

3.  For each structural part, extract typed connection ports (round holes,
    axle holes, studs) from LDraw primitives with exact positions.

4.  Match: a structural part connects to a connector when one of its ports
    is on the connector's shaft line AND the port orientation aligns with
    the shaft direction.

5.  Determine connection type from port-type × connector-type:
      - Friction pin + any hole → RIGID
      - Frictionless pin + any hole → REVOLUTE
      - Axle + axle_hole (cross) → RIGID
      - Axle + round_hole → REVOLUTE
      - Axle-pin + any hole → REVOLUTE

6.  Union-find groups structural parts connected by rigid bonds into *units*.
    Revolute connections that span two different units become *joints*.

7.  Stud connections (STUD at same position as another part's port) are rigid.

Fallback method: **Distance-based detection** (legacy)

    For builds with no recognised connectors, falls back to the original
    vertex-proximity heuristic.
"""

from __future__ import annotations

from collections import Counter
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from ..ldraw.model import LDrawBuild, LDrawPart
from .connection_ports import ConnectionPort, PortType
from .connectors import (
    ConnectorType,
    classify_connector,
    creates_revolute_connection,
    creates_rigid_connection,
    is_connector,
)
from .mesh_properties import (
    ABS_DENSITY_KG_PER_M3,
    LDU_TO_METERS,
    mesh_volume_and_com,
)
from .model import GearConstraint, Joint, JointType, PhysicsScene, Unit
from .motor_detection import detect_motors
from .gears import detect_gear_meshes

# Distance threshold in LDU for two bricks to be considered "snapped".
DEFAULT_SNAP_THRESHOLD_LDU: float = 4.0

# Minimum number of contact points for a joint to be classified as FIXED.
FIXED_CONTACT_MIN: int = 3

# Tolerance for port position matching (LDU).
PORT_POSITION_TOLERANCE: float = 3.0

# Tolerance for port orientation alignment (cosine of max angle).
PORT_ORIENTATION_TOLERANCE: float = 0.7  # ~45° to handle off-axis pegholes


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
# Port-based connection detection
# ---------------------------------------------------------------------------


def _connector_shaft(connector: LDrawPart) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute the shaft line segment of a connector in world space.

    Returns (endpoint_a, endpoint_b, axis_unit_vector).
    Falls back to a zero-length segment at the part origin if no geometry.

    The shaft direction is determined from the world-space mesh bounding box:
    the axis with the largest extent is the insertion direction.  Triangles
    are already stored in world coordinates, so no additional rotation is
    needed.
    """
    if not connector.triangles:
        pos = connector.position
        return pos, pos, np.array([0.0, 1.0, 0.0])

    verts = np.vstack([[t.v0, t.v1, t.v2] for t in connector.triangles])
    extent = verts.max(axis=0) - verts.min(axis=0)
    shaft_axis = int(np.argmax(extent))
    half_length = extent[shaft_axis] / 2.0

    # Vertices are already in world space — use the axis directly.
    world_shaft = np.zeros(3)
    world_shaft[shaft_axis] = 1.0

    center = (verts.min(axis=0) + verts.max(axis=0)) / 2.0
    ep_a = center - world_shaft * half_length
    ep_b = center + world_shaft * half_length
    return ep_a, ep_b, world_shaft


def _port_on_shaft(
    port: ConnectionPort,
    ep_a: np.ndarray,
    ep_b: np.ndarray,
    shaft_dir: np.ndarray,
    pos_tol: float = PORT_POSITION_TOLERANCE,
    ori_tol: float = PORT_ORIENTATION_TOLERANCE,
) -> bool:
    """Check if a port's position lies on the shaft line and its orientation aligns.

    The port must be within *pos_tol* of the infinite shaft line
    (lateral distance) **and** within the shaft segment extended by a
    small axial margin to account for insertion depth (pins/axles insert
    into beam holes beyond the connector mesh boundary).
    """
    shaft_vec = ep_b - ep_a
    shaft_len_sq = float(np.dot(shaft_vec, shaft_vec))

    v = port.position - ep_a
    if shaft_len_sq < 1e-12:
        dist = float(np.linalg.norm(v))
    else:
        t = float(np.dot(v, shaft_vec)) / shaft_len_sq
        # Allow extending up to _AXIAL_MARGIN beyond each endpoint
        # to account for pin insertion depth into beam holes (the
        # connector mesh often doesn't extend to the actual engagement
        # point, typically ~1-2 LDU short).
        _AXIAL_MARGIN_LDU = 4.0
        shaft_len = shaft_len_sq ** 0.5
        margin_t = _AXIAL_MARGIN_LDU / shaft_len if shaft_len > 1e-6 else 0.0
        t_clamped = max(-margin_t, min(1.0 + margin_t, t))
        closest = ep_a + t_clamped * shaft_vec
        dist = float(np.linalg.norm(port.position - closest))

    if dist > pos_tol:
        return False

    # Check orientation alignment (port axis should be parallel to shaft)
    cos_angle = abs(float(np.dot(port.orientation, shaft_dir)))
    return cos_angle >= ori_tol


def _is_motor_output_axis(motor: LDrawPart, port: ConnectionPort) -> bool:
    """True when *port* is aligned with the motor's output shaft axis.

    The output axis is defined by the orientation of the motor's AXLE_HOLE
    ports.  ROUND_HOLEs along the same axis are internal bearings that allow
    free rotation, as opposed to perpendicular mounting holes.
    """
    for p in motor.ports:
        if p.port_type == PortType.AXLE_HOLE:
            cos = abs(float(np.dot(port.orientation, p.orientation)))
            return cos > 0.9
    return False


def _determine_connection_type(
    connector_type: ConnectorType,
    port_type: PortType,
    structural_part: Optional[LDrawPart] = None,
    port: Optional[ConnectionPort] = None,
) -> str:
    """Determine if a connector-port pair creates a 'rigid' or 'revolute' bond.

    Returns 'rigid', 'revolute', or 'none'.
    """
    # Motor output shafts: axle holes and output-axis round holes on motors
    # create revolute (driven) joints.  Round holes along the output axis
    # are internal bearings — connectors through them can spin freely.
    if structural_part is not None:
        from .motor_detection import is_motor_part
        if is_motor_part(structural_part.part_id):
            if port_type == PortType.AXLE_HOLE:
                return "revolute"
            if (port_type == PortType.ROUND_HOLE
                    and port is not None
                    and _is_motor_output_axis(structural_part, port)):
                return "revolute"

    if connector_type == ConnectorType.FRICTION_PIN:
        # Friction pins lock into any hole type
        return "rigid"
    elif connector_type == ConnectorType.FRICTIONLESS_PIN:
        # Frictionless pins allow rotation in any hole type
        return "revolute"
    elif connector_type == ConnectorType.AXLE:
        # Axle behaviour depends on hole type
        if port_type == PortType.AXLE_HOLE:
            return "rigid"  # Cross hole grips axle
        elif port_type == PortType.ROUND_HOLE:
            return "revolute"  # Round hole lets axle spin
        else:
            return "rigid"  # Default: treat as rigid
    elif connector_type == ConnectorType.AXLE_PIN:
        # Axle pin: axle end grips cross holes rigidly, pin end has friction
        # ridges and is rigid in round holes.  Exception: axle holes on motor
        # parts are always revolute (driven output shafts).
        if port_type == PortType.AXLE_HOLE:
            if structural_part is not None:
                from .motor_detection import is_motor_part
                if is_motor_part(structural_part.part_id):
                    return "revolute"
            return "rigid"  # Cross hole grips the axle end
        elif port_type == PortType.ROUND_HOLE:
            return "rigid"  # Pin end has friction ridges
        else:
            return "rigid"  # Default: treat as rigid
    return "none"


def _find_port_connections(
    connector: LDrawPart,
    structural_parts: List[Tuple[int, LDrawPart]],
) -> List[Tuple[int, str]]:
    """Find structural parts whose ports align with a connector's shaft.

    Returns list of (global_part_index, connection_type) where
    connection_type is 'rigid' or 'revolute'.
    """
    ep_a, ep_b, shaft_dir = _connector_shaft(connector)
    ctype = classify_connector(connector.part_id)
    if ctype is None:
        return []

    shaft_vec = ep_b - ep_a
    shaft_len_sq = float(np.dot(shaft_vec, shaft_vec))

    connections: List[Tuple[int, str, float]] = []  # (idx, type, t)
    for idx, sp in structural_parts:
        if not sp.ports:
            continue
        for port in sp.ports:
            if port.port_type == PortType.STUD:
                continue  # Studs handled separately
            if _port_on_shaft(port, ep_a, ep_b, shaft_dir):
                conn_type = _determine_connection_type(ctype, port.port_type, sp, port)
                if conn_type != "none":
                    v = port.position - ep_a
                    t = float(np.dot(v, shaft_vec)) / max(shaft_len_sq, 1e-12)
                    connections.append((idx, conn_type, t))
                    break  # One port match per part is sufficient

    # Capacity limit: a connector can physically engage at most
    # shaft_length / 20 parts (standard Technic hole spacing is 20 LDU).
    # When more parts match, drop endpoint matches — they represent
    # coincidental proximity rather than true engagement (e.g. a 3L pin
    # with bushings cannot reach a 4th part past its tip).
    shaft_len = float(np.sqrt(shaft_len_sq))
    max_conns = max(2, round(shaft_len / 20.0))
    if len(connections) > max_conns:
        connections.sort(key=lambda c: abs(c[2] - 0.5))
        connections = connections[:max_conns]

    return [(idx, ct) for idx, ct, _t in connections]


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
    scene.gears = detect_gear_meshes(scene)

    # ── Consistency checks ────────────────────────────────────────────────
    # 1. Every structural brick must have geometry (triangles).
    for i in structural_indices:
        p = parts[i]
        if not p.triangles:
            import warnings
            warnings.warn(
                f"Part {p.part_id} at index {i} has no geometry (0 triangles). "
                f"It may be missing from the LDraw library.",
                stacklevel=2,
            )

    # 2. Every structural brick must belong to exactly one unit.
    assigned_parts = set()
    for unit in scene.units:
        for brick in unit.bricks:
            assigned_parts.add(id(brick))
    for i in structural_indices:
        p = parts[i]
        if id(p) not in assigned_parts:
            import warnings
            warnings.warn(
                f"Part {p.part_id} at index {i} is structural but not "
                f"assigned to any unit.",
                stacklevel=2,
            )

    return scene


def _units_from_uf(
    uf: _UnionFind,
    n_structural: int,
    structural_indices: List[int],
    parts: List[LDrawPart],
    density: float,
    ldu_to_meters: float,
) -> Tuple[List[Unit], Dict[int, int]]:
    """Build unit list and local-to-unit mapping from current UnionFind state."""
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

    return units, local_to_unit


def _build_via_connectors(
    parts: List[LDrawPart],
    connector_indices: List[int],
    structural_indices: List[int],
    density: float,
    ldu_to_meters: float,
    margin: float,
) -> PhysicsScene:
    """Build units and joints using port-based connection detection."""
    n_structural = len(structural_indices)
    if n_structural == 0:
        return PhysicsScene()

    # Map from global part index → local structural index
    global_to_local: Dict[int, int] = {
        gi: li for li, gi in enumerate(structural_indices)
    }

    uf = _UnionFind(n_structural)
    structural_tuples = [(i, parts[i]) for i in structural_indices]

    # Track which structural parts are touched by at least one connector
    touched_by_connector: set = set()

    # Revolute connections: (local_i, local_j, connector_part)
    revolute_connections: List[Tuple[int, int, LDrawPart]] = []

    for ci in connector_indices:
        conn_part = parts[ci]
        connections = _find_port_connections(conn_part, structural_tuples)

        if not connections:
            continue

        # Mark all connected parts as touched
        for gi, _ in connections:
            touched_by_connector.add(global_to_local[gi])

        # Group connections by type
        rigid_parts = [gi for gi, ct in connections if ct == "rigid"]
        revolute_parts = [gi for gi, ct in connections if ct == "revolute"]

        # For mixed connections (axle through both cross and round holes):
        # - All rigid parts merge together
        # - Each revolute part gets a revolute joint to the rigid group
        if rigid_parts:
            first_local = global_to_local[rigid_parts[0]]
            for gi in rigid_parts[1:]:
                uf.union(first_local, global_to_local[gi])
            # Revolute connections go between each revolute part and the
            # rigid group
            for gi in revolute_parts:
                revolute_connections.append((
                    global_to_local[gi],
                    first_local,
                    conn_part,
                ))
        elif len(revolute_parts) >= 2:
            # All revolute — store pairwise connections
            for k in range(len(revolute_parts)):
                for m in range(k + 1, len(revolute_parts)):
                    revolute_connections.append((
                        global_to_local[revolute_parts[k]],
                        global_to_local[revolute_parts[m]],
                        conn_part,
                    ))

    # Parallel-pin rigidity: if two or more frictionless pins connect the
    # same pair of structural parts, the connection is over-constrained and
    # acts as a rigid link (parallel pins prevent rotation).
    pair_pin_counts: Counter = Counter()
    for li_a, li_b, _ in revolute_connections:
        pair = (min(li_a, li_b), max(li_a, li_b))
        pair_pin_counts[pair] += 1

    for (li_a, li_b), count in pair_pin_counts.items():
        if count >= 2:
            uf.union(li_a, li_b)

    # Stud merging: parts with STUD ports co-located with another part's
    # port position are rigidly connected (brick-on-brick stacking).
    _STUD_MATCH_TOL = 4.0  # LDU tolerance for stud-to-port alignment
    for li in range(n_structural):
        sp = parts[structural_indices[li]]
        stud_ports = [p for p in sp.ports if p.port_type == PortType.STUD]
        if not stud_ports:
            continue
        for other_li in range(n_structural):
            if other_li == li:
                continue
            if uf.find(li) == uf.find(other_li):
                continue  # already merged
            other_sp = parts[structural_indices[other_li]]
            other_holes = [p for p in other_sp.ports
                           if p.port_type in (PortType.ROUND_HOLE, PortType.AXLE_HOLE)]
            if not other_holes:
                continue
            # Check if any stud aligns with any hole on the other part
            matched = False
            for stud in stud_ports:
                for hole in other_holes:
                    d = float(np.linalg.norm(stud.position - hole.position))
                    if d < _STUD_MATCH_TOL:
                        matched = True
                        break
                if matched:
                    break
            if matched:
                uf.union(li, other_li)
                touched_by_connector.add(li)
                touched_by_connector.add(other_li)

    # Stud/anti-stud merging: when a STUD from one part is close to an
    # ANTI_STUD on another part (brick stacking), merge them.  Anti-stud
    # tubes sit *between* studs, so we decompose the distance into an
    # axial component (along the stud axis) and a lateral component
    # (perpendicular) and use separate tolerances.
    _STUD_AXIAL_TOL = 10.0   # LDU along stud axis
    _STUD_LATERAL_TOL = 14.0  # LDU perpendicular to stud axis
    for li in range(n_structural):
        sp = parts[structural_indices[li]]
        studs_a = [p for p in sp.ports if p.port_type == PortType.STUD]
        anti_a = [p for p in sp.ports if p.port_type == PortType.ANTI_STUD]
        if not studs_a and not anti_a:
            continue
        for other_li in range(li + 1, n_structural):
            if uf.find(li) == uf.find(other_li):
                continue
            other_sp = parts[structural_indices[other_li]]
            studs_b = [p for p in other_sp.ports if p.port_type == PortType.STUD]
            anti_b = [p for p in other_sp.ports
                       if p.port_type == PortType.ANTI_STUD]
            # Check both directions: A.stud↔B.anti and A.anti↔B.stud
            pairs = []
            if studs_a and anti_b:
                pairs.extend((s, a) for s in studs_a for a in anti_b)
            if anti_a and studs_b:
                pairs.extend((s, a) for s in studs_b for a in anti_a)
            matched = False
            for stud, astud in pairs:
                if np.dot(stud.orientation, astud.orientation) > -0.5:
                    continue
                diff = astud.position - stud.position
                axial = abs(float(np.dot(diff, stud.orientation)))
                lateral = float(np.linalg.norm(
                    diff - np.dot(diff, stud.orientation) * stud.orientation))
                if axial < _STUD_AXIAL_TOL and lateral < _STUD_LATERAL_TOL:
                    matched = True
                    break
            if matched:
                uf.union(li, other_li)
                touched_by_connector.add(li)
                touched_by_connector.add(other_li)

    # Fallback for parts with no ports and no connector touching them:
    # merge with nearest structural part via vertex proximity.
    _SNAP_DIST_LDU = 8.0
    for li in range(n_structural):
        if li in touched_by_connector:
            continue
        sp = parts[structural_indices[li]]
        if sp.ports:
            continue  # Has ports — just not matched by any connector
        if not sp.triangles:
            continue
        sverts = np.vstack([[t.v0, t.v1, t.v2] for t in sp.triangles])
        best_dist = _SNAP_DIST_LDU
        best_li = -1
        for other_li in range(n_structural):
            if other_li == li:
                continue
            other_sp = parts[structural_indices[other_li]]
            if not other_sp.triangles:
                continue
            c2c = float(np.linalg.norm(sp.position - other_sp.position))
            if c2c > 60.0:
                continue
            overts = np.vstack([[t.v0, t.v1, t.v2] for t in other_sp.triangles])
            s_sample = sverts[::3] if len(sverts) > 30 else sverts
            o_sample = overts[::3] if len(overts) > 30 else overts
            diffs = s_sample[:, np.newaxis, :] - o_sample[np.newaxis, :, :]
            dists_sq = np.sum(diffs ** 2, axis=2)
            min_d = float(np.sqrt(dists_sq.min()))
            if min_d < best_dist:
                best_dist = min_d
                best_li = other_li
        if best_li >= 0:
            uf.union(li, best_li)

    # Build units from union-find groups
    units, local_to_unit = _units_from_uf(
        uf, n_structural, structural_indices, parts, density, ldu_to_meters
    )

    # Non-collinear revolute axis merging: if two units share revolute
    # connections whose axes are not collinear (same line), the pair is
    # over-constrained and the units must be merged into one rigid body.
    # Two axes are collinear when they are parallel AND their pivot points
    # lie on the same line.  Non-parallel axes or parallel axes at offset
    # positions both prevent free rotation → rigid.
    _PARALLEL_TOL = 0.95
    _COLLINEAR_DIST_TOL = 2.0  # LDU tolerance for point-to-line distance
    merged_any = True
    while merged_any:
        merged_any = False
        # (shaft_dir, position, local_a, local_b) per unit pair
        pair_entries: Dict[
            Tuple[int, int],
            List[Tuple[np.ndarray, np.ndarray, int, int]],
        ] = {}
        for li_a, li_b, conn_part in revolute_connections:
            ui = local_to_unit[li_a]
            uj = local_to_unit[li_b]
            if ui == uj:
                continue
            pair = (min(ui, uj), max(ui, uj))
            _, _, shaft = _connector_shaft(conn_part)
            pos = conn_part.position.copy()
            pair_entries.setdefault(pair, []).append(
                (shaft, pos, li_a, li_b)
            )

        for _pair, entries in pair_entries.items():
            if len(entries) < 2:
                continue
            ref_axis = entries[0][0]
            ref_pos = entries[0][1]
            for shaft, pos, _la, _lb in entries[1:]:
                is_parallel = abs(float(np.dot(ref_axis, shaft))) >= _PARALLEL_TOL
                if is_parallel:
                    # Check collinearity: distance from pos to the ref axis line
                    v = pos - ref_pos
                    cross = np.cross(v, ref_axis)
                    dist = float(np.linalg.norm(cross))
                    if dist < _COLLINEAR_DIST_TOL:
                        continue  # truly collinear — single revolute DOF
                # Non-collinear (non-parallel or offset-parallel) → merge
                uf.union(entries[0][2], entries[0][3])
                merged_any = True
                break

        if merged_any:
            units, local_to_unit = _units_from_uf(
                uf, n_structural, structural_indices, parts, density,
                ldu_to_meters,
            )

    # Build revolute joints
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

        _, _, shaft_dir = _connector_shaft(conn_part)
        position = conn_part.position * ldu_to_meters

        joints.append(
            Joint(
                unit_a_index=ui,
                unit_b_index=uj,
                joint_type=JointType.REVOLUTE,
                position=position,
                axis=shaft_dir,
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
