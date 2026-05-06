"""Gear mesh detection and constraint generation.

Detects when two gears are meshing (centers at correct distance, axes
compatible) and creates GearConstraint objects that encode the gear ratio
and rotation axes for the Blender exporter.

Gear types:
    - Spur / Double Bevel: axes must be parallel, mesh in perpendicular plane.
    - Bevel: axes at ~90°, mesh at the intersection point.

Pitch radius formula: pitch_radius = teeth × 1.25 LDU (standard Technic module).
Two gears mesh when their center distance ≈ sum of pitch radii (within tolerance).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from ..ldraw.model import LDrawPart
from .model import GearConstraint, PhysicsScene


# ---------------------------------------------------------------------------
# Gear catalog: part_id → (tooth_count, gear_type)
# ---------------------------------------------------------------------------

class GearType:
    """Classification of gear geometry."""
    SPUR = "spur"           # Standard straight-tooth (meshes with parallel axes)
    DOUBLE_BEVEL = "double_bevel"  # Can mesh parallel OR at 90°
    BEVEL = "bevel"         # Meshes at 90° angle


# Part ID → (tooth count, gear type, local rotation axis index)
# The local axis is which axis of the gear's local frame is the rotation axis.
# For all standard Technic gears the rotation axis is local Z (teeth in XY plane).
GEAR_CATALOG: Dict[str, Tuple[int, str, int]] = {
    # Spur gears
    "3647.dat": (8, GearType.SPUR, 2),        # Technic Gear 8 Tooth
    "3648.dat": (24, GearType.SPUR, 2),        # Technic Gear 24 Tooth
    "3648a.dat": (24, GearType.SPUR, 2),       # variant
    "3648b.dat": (24, GearType.SPUR, 2),       # Technic Gear 24 Tooth (Single Axle Hole)
    "3649.dat": (40, GearType.SPUR, 2),        # Technic Gear 40 Tooth
    "94925.dat": (16, GearType.SPUR, 2),       # Technic Gear 16 Tooth
    "32498.dat": (36, GearType.SPUR, 2),       # Technic Gear 36 Tooth Double Bevel
    # Double bevel gears (can mesh parallel or at 90°)
    "32269.dat": (20, GearType.DOUBLE_BEVEL, 2),  # Technic Gear 20 Tooth Double Bevel
    "32270.dat": (12, GearType.DOUBLE_BEVEL, 2),  # Technic Gear 12 Tooth Double Bevel
    "6589.dat": (12, GearType.DOUBLE_BEVEL, 2),   # older 12T double bevel
    "18946.dat": (28, GearType.DOUBLE_BEVEL, 2),  # Technic Gear 28 Tooth Double Bevel
    # Bevel gears (mesh at 90°)
    "4143.dat": (14, GearType.BEVEL, 2),       # Technic Gear 14 Tooth Bevel
    "6588.dat": (12, GearType.BEVEL, 2),       # Technic Gear 12 Tooth Bevel (older)
}

# Standard Technic gear module: pitch_radius = teeth × MODULE_LDU
MODULE_LDU: float = 1.25

# Tolerance for mesh distance check (LDU)
MESH_DISTANCE_TOLERANCE: float = 3.0

# Tolerance for axis alignment checks (cosine)
PARALLEL_AXIS_TOLERANCE: float = 0.9   # cos(~25°) for parallel gears
PERPENDICULAR_AXIS_TOLERANCE: float = 0.3  # cos should be near 0 for 90°


# ---------------------------------------------------------------------------
# Gear info extraction
# ---------------------------------------------------------------------------


def _gear_info(part: LDrawPart) -> Optional[Tuple[int, str, np.ndarray, np.ndarray]]:
    """Extract gear info from a part.

    Returns (tooth_count, gear_type, center_world, axis_world) or None.
    The rotation axis is the part's local Y axis transformed to world space
    (standard for all Technic gears — teeth lie in the XZ plane).
    The gear center is the part's origin position.
    """
    catalog_entry = GEAR_CATALOG.get(part.part_id.lower())
    if catalog_entry is None:
        return None

    teeth, gear_type, local_axis_idx = catalog_entry

    # Gear center is the part origin
    center_world = part.position.copy()

    # Rotation axis from the part's local frame
    rot = part.transform[:3, :3]
    local_dir = np.zeros(3)
    local_dir[local_axis_idx] = 1.0
    axis_world = rot @ local_dir
    norm = np.linalg.norm(axis_world)
    if norm > 1e-12:
        axis_world /= norm

    return teeth, gear_type, center_world, axis_world


# ---------------------------------------------------------------------------
# Mesh detection
# ---------------------------------------------------------------------------


def _can_mesh(type_a: str, type_b: str, cos_angle: float) -> bool:
    """Check if two gear types can mesh given the angle between their axes."""
    abs_cos = abs(cos_angle)

    # Both spur → must be parallel
    if type_a == GearType.SPUR and type_b == GearType.SPUR:
        return abs_cos >= PARALLEL_AXIS_TOLERANCE

    # Both double bevel → can be parallel OR perpendicular
    if type_a == GearType.DOUBLE_BEVEL and type_b == GearType.DOUBLE_BEVEL:
        return abs_cos >= PARALLEL_AXIS_TOLERANCE or abs_cos <= PERPENDICULAR_AXIS_TOLERANCE

    # One bevel + one double bevel (or both bevel) → perpendicular
    if type_a == GearType.BEVEL or type_b == GearType.BEVEL:
        if type_a == GearType.DOUBLE_BEVEL or type_b == GearType.DOUBLE_BEVEL:
            return abs_cos <= PERPENDICULAR_AXIS_TOLERANCE
        # Both bevel
        return abs_cos <= PERPENDICULAR_AXIS_TOLERANCE

    # Spur + double bevel → parallel only
    if (type_a == GearType.SPUR and type_b == GearType.DOUBLE_BEVEL) or \
       (type_a == GearType.DOUBLE_BEVEL and type_b == GearType.SPUR):
        return abs_cos >= PARALLEL_AXIS_TOLERANCE

    return False


def detect_gear_meshes(
    scene: PhysicsScene,
    distance_tolerance: float = MESH_DISTANCE_TOLERANCE,
) -> List[GearConstraint]:
    """Detect meshing gear pairs and return gear constraints.

    For each pair of units containing gears, checks:
    - Parallel axes: center distance ≈ sum of pitch radii.
    - Perpendicular axes (bevel): axis lines nearly intersect and
      on-axis distances to intersection ≈ pitch radii.

    Returns a list of GearConstraint objects.
    """
    # Collect gear info per unit
    gear_units: List[Tuple[int, int, str, np.ndarray, np.ndarray]] = []

    for unit_idx, unit in enumerate(scene.units):
        for brick in unit.bricks:
            info = _gear_info(brick)
            if info is not None:
                teeth, gear_type, center, axis = info
                gear_units.append((unit_idx, teeth, gear_type, center, axis))

    # Check all pairs for meshing
    constraints: List[GearConstraint] = []
    used_pairs = set()

    for i in range(len(gear_units)):
        for j in range(i + 1, len(gear_units)):
            ui, teeth_a, type_a, center_a, axis_a = gear_units[i]
            uj, teeth_b, type_b, center_b, axis_b = gear_units[j]

            # Skip if same unit
            if ui == uj:
                continue

            # Skip duplicate pairs
            pair_key = (min(ui, uj), max(ui, uj))
            if pair_key in used_pairs:
                continue

            # Check axis compatibility
            cos_angle = float(np.dot(axis_a, axis_b))
            if not _can_mesh(type_a, type_b, cos_angle):
                continue

            r_a = teeth_a * MODULE_LDU
            r_b = teeth_b * MODULE_LDU
            delta = center_b - center_a
            dist = float(np.linalg.norm(delta))

            if abs(cos_angle) >= PARALLEL_AXIS_TOLERANCE:
                # Parallel axes: simple center distance check
                expected_dist = r_a + r_b
                if abs(dist - expected_dist) > distance_tolerance:
                    continue
                # Mesh position
                if dist > 1e-6:
                    mesh_pos = center_a + delta * (r_a / dist)
                else:
                    mesh_pos = (center_a + center_b) / 2.0
            else:
                # Perpendicular axes (bevel): check axis line intersection
                mesh_pos = _bevel_mesh_check(
                    center_a, axis_a, r_a,
                    center_b, axis_b, r_b,
                    distance_tolerance,
                )
                if mesh_pos is None:
                    continue

            ratio = teeth_a / teeth_b

            constraints.append(GearConstraint(
                unit_a_index=ui,
                unit_b_index=uj,
                ratio=ratio,
                axis_a=axis_a,
                axis_b=axis_b,
                position=mesh_pos,
            ))
            used_pairs.add(pair_key)

    return constraints


def _bevel_mesh_check(
    center_a: np.ndarray, axis_a: np.ndarray, r_a: float,
    center_b: np.ndarray, axis_b: np.ndarray, r_b: float,
    tolerance: float,
) -> Optional[np.ndarray]:
    """Check if two bevel gears mesh by finding axis line intersection.

    For bevel gears at ~90°, the axes should nearly intersect.
    The distance from each center to the intersection should be
    approximately equal to that gear's pitch radius.

    Returns the mesh position (intersection point) or None if not meshing.
    """
    # Find closest point between the two axis lines
    # Line A: center_a + t * axis_a
    # Line B: center_b + s * axis_b
    w = center_a - center_b
    a_dot_a = 1.0  # axis vectors are unit
    b_dot_b = 1.0
    a_dot_b = float(np.dot(axis_a, axis_b))
    a_dot_w = float(np.dot(axis_a, w))
    b_dot_w = float(np.dot(axis_b, w))

    denom = a_dot_a * b_dot_b - a_dot_b * a_dot_b
    if abs(denom) < 1e-12:
        return None  # Lines are parallel

    t = (a_dot_b * b_dot_w - b_dot_b * a_dot_w) / denom
    s = (a_dot_a * b_dot_w - a_dot_b * a_dot_w) / denom

    closest_a = center_a + t * axis_a
    closest_b = center_b + s * axis_b
    perp_dist = float(np.linalg.norm(closest_a - closest_b))

    # Lines should nearly intersect (perpendicular distance < tolerance)
    if perp_dist > tolerance:
        return None

    # Check on-axis distances match pitch radii (with generous tolerance)
    dist_a = abs(t)
    dist_b = abs(s)
    bevel_tol = tolerance * 2  # More generous for bevel gears

    if abs(dist_a - r_a) > bevel_tol and abs(dist_a - r_a) > r_a * 0.4:
        return None
    if abs(dist_b - r_b) > bevel_tol and abs(dist_b - r_b) > r_b * 0.4:
        return None

    # Intersection point (midpoint of closest approach)
    return (closest_a + closest_b) / 2.0
