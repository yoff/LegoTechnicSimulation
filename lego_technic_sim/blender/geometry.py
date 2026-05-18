"""Shared geometry helpers for Blender script generation.

Provides coordinate-system conversion and triangle collection used by both
the simulation exporter and the assembly-animation exporter.
"""

from __future__ import annotations

from typing import Callable, List, Sequence, Tuple

import numpy as np

from ..physics.mesh_properties import LDU_TO_METERS


def ldraw_to_blender(v: np.ndarray) -> np.ndarray:
    """Convert a 3-D point from LDraw space to Blender space.

    LDraw: X right, Y down, Z toward viewer.
    Blender: X right, Y toward viewer, Z up.
    """
    return np.array([v[0], -v[2], -v[1]], dtype=float)


def collect_geometry(
    bricks: Sequence,
) -> Tuple[List[List[float]], List[List[int]]]:
    """Return (vertices, faces) for a set of bricks, in Blender coordinates.

    Each element of *bricks* must have a ``triangles`` attribute containing
    triangle objects with ``v0``, ``v1``, ``v2`` numpy-array attributes
    (in LDraw world space).

    Returns lists suitable for Blender's
    ``mesh.from_pydata(vertices, [], faces)``.
    """
    vertices: List[List[float]] = []
    faces: List[List[int]] = []
    vi = 0
    for brick in bricks:
        for tri in brick.triangles:
            v0 = ldraw_to_blender(tri.v0) * LDU_TO_METERS
            v1 = ldraw_to_blender(tri.v1) * LDU_TO_METERS
            v2 = ldraw_to_blender(tri.v2) * LDU_TO_METERS
            vertices.append(
                [round(float(v0[0]), 7), round(float(v0[1]), 7), round(float(v0[2]), 7)]
            )
            vertices.append(
                [round(float(v1[0]), 7), round(float(v1[1]), 7), round(float(v1[2]), 7)]
            )
            vertices.append(
                [round(float(v2[0]), 7), round(float(v2[1]), 7), round(float(v2[2]), 7)]
            )
            faces.append([vi, vi + 1, vi + 2])
            vi += 3
    return vertices, faces


def emit_kinematic_rotation(
    emit: Callable[[str], None],
    obj_expr: str,
    pivot: np.ndarray,
    axis: np.ndarray,
    angle_per_frame: float,
) -> None:
    """Emit Blender Python that sets up driver-based rotation for an object.

    Relocates the object origin to *pivot*, sets rotation_mode to AXIS_ANGLE,
    and adds a driver expression that rotates at *angle_per_frame* rad/frame
    around *axis*.

    Args:
        emit: Line-emitter callback (appends to script).
        obj_expr: Python expression referencing the Blender object.
        pivot: Hinge pivot position in Blender space.
        axis: Normalised rotation axis in Blender space.
        angle_per_frame: Rotation increment per frame (rad).
    """
    emit(f"_kin_obj = {obj_expr}")
    emit(f"_kin_pivot = mathutils.Vector(({pivot[0]:.6f}, {pivot[1]:.6f}, {pivot[2]:.6f}))")
    emit("_kin_offset = _kin_obj.location - _kin_pivot")
    emit("if _kin_obj.data:")
    emit("    for v in _kin_obj.data.vertices:")
    emit("        v.co += _kin_offset")
    emit("_kin_obj.location = _kin_pivot")
    emit("_kin_obj.rotation_mode = 'AXIS_ANGLE'")
    emit(f"_kin_obj.rotation_axis_angle = (0.0, {axis[0]:.6f}, {axis[1]:.6f}, {axis[2]:.6f})")
    emit("_kin_drv = _kin_obj.driver_add('rotation_axis_angle', 0)")
    emit("_kin_drv.driver.type = 'SCRIPTED'")
    emit(f"_kin_drv.driver.expression = 'frame * {angle_per_frame:.8f}'")
