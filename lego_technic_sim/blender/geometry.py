"""Shared geometry helpers for Blender script generation.

Provides coordinate-system conversion and triangle collection used by both
the simulation exporter and the assembly-animation exporter.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..physics.mesh_properties import LDU_TO_METERS


def parse_ldconfig(ldraw_library: Optional[Path] = None) -> Dict[int, Tuple[float, float, float]]:
    """Parse LDConfig.ldr and return a mapping of color code → (R, G, B) floats [0-1]."""
    candidates = []
    if ldraw_library:
        candidates.append(Path(ldraw_library) / "LDConfig.ldr")
    candidates += [
        Path("/opt/ldraw/ldraw/LDConfig.ldr"),
        Path("/opt/ldraw/LDConfig.ldr"),
    ]

    config_path = None
    for p in candidates:
        if p.exists():
            config_path = p
            break

    colors: Dict[int, Tuple[float, float, float]] = {}
    if config_path is None:
        return colors

    pattern = re.compile(r"!COLOUR\s+\S+\s+CODE\s+(\d+)\s+VALUE\s+#([0-9A-Fa-f]{6})")
    with open(config_path) as f:
        for line in f:
            m = pattern.search(line)
            if m:
                code = int(m.group(1))
                hex_rgb = m.group(2)
                r = int(hex_rgb[0:2], 16) / 255.0
                g = int(hex_rgb[2:4], 16) / 255.0
                b = int(hex_rgb[4:6], 16) / 255.0
                colors[code] = (r, g, b)
    return colors


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


def collect_geometry_colored(
    bricks: Sequence,
) -> Tuple[List[List[float]], List[List[int]], List[int]]:
    """Return (vertices, faces, face_colors) for a set of bricks.

    Like collect_geometry but also returns a per-face list of LDraw color codes
    (one entry per face, corresponding to the brick's .color attribute).
    """
    vertices: List[List[float]] = []
    faces: List[List[int]] = []
    face_colors: List[int] = []
    vi = 0
    for brick in bricks:
        color = getattr(brick, "color", 7)  # default Light_Grey
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
            face_colors.append(color)
            vi += 3
    return vertices, faces, face_colors


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


def emit_lighting_check(emit: Callable[[str], None], check_frame: int = -1) -> None:
    """Emit a lighting auto-adjustment pass into the Blender script.

    Renders a single frame at 32×32 with 1 sample, reads pixel values,
    computes average luminance of non-background pixels, and adjusts light
    energy if too bright or too dark.  Iterates up to 4 times.

    Takes ~1-2 seconds per iteration (no GPU needed, CPU Cycles at 1 sample).

    Args:
        emit: Line-emitter callback.
        check_frame: Frame to check; -1 means the last frame (all objects visible).
    """
    emit("")
    emit("# ── Lighting Check ─────────────────────────────────────────────")
    emit("def _check_and_adjust_lighting(target_min=0.25, target_max=0.55, max_iters=4):")
    emit("    \"\"\"Auto-adjust lighting by rendering a tiny test frame.\"\"\"")
    emit("    # Save render settings")
    emit("    _orig_x = scene.render.resolution_x")
    emit("    _orig_y = scene.render.resolution_y")
    emit("    _orig_samples = scene.cycles.samples")
    emit("    _orig_filepath = scene.render.filepath")
    emit("    _orig_format = scene.render.image_settings.file_format")
    if check_frame == -1:
        emit("    _check_frame = scene.frame_end")
    else:
        emit(f"    _check_frame = {check_frame}")
    emit("    # Temporarily set tiny render")
    emit("    scene.render.resolution_x = 32")
    emit("    scene.render.resolution_y = 32")
    emit("    scene.cycles.samples = 1")
    emit("    scene.render.filepath = '/tmp/_lighting_check'")
    emit("    scene.render.image_settings.file_format = 'PNG'")
    emit("    scene.frame_set(_check_frame)")
    emit("    # Find all lights")
    emit("    _lights = [o for o in bpy.data.objects if o.type == 'LIGHT']")
    emit("    for _iteration in range(max_iters):")
    emit("        bpy.ops.render.render(write_still=True)")
    emit("        # Load the saved PNG to read pixels")
    emit("        import os")
    emit("        _check_path = '/tmp/_lighting_check.png'")
    emit("        if not os.path.exists(_check_path):")
    emit("            print('  Lighting check: render file not found, skipping.')")
    emit("            break")
    emit("        _img = bpy.data.images.load(_check_path)")
    emit("        _pixels = list(_img.pixels)")
    emit("        _n_pixels = len(_pixels) // 4  # RGBA")
    emit("        bpy.data.images.remove(_img)")
    emit("        if _n_pixels == 0:")
    emit("            print('  Lighting check: empty render, skipping.')")
    emit("            break")
    emit("        # Detect if we have a transparent background")
    emit("        _has_transparent = any(_pixels[i*4+3] < 0.5 for i in range(0, _n_pixels, max(1, _n_pixels//16)))")
    emit("        _lum_sum = 0.0")
    emit("        _count = 0")
    emit("        for _pi in range(_n_pixels):")
    emit("            _r = _pixels[_pi * 4]")
    emit("            _g = _pixels[_pi * 4 + 1]")
    emit("            _b = _pixels[_pi * 4 + 2]")
    emit("            _a = _pixels[_pi * 4 + 3]")
    emit("            _lum = 0.2126 * _r + 0.7152 * _g + 0.0722 * _b")
    emit("            if _has_transparent:")
    emit("                if _a < 0.01:")
    emit("                    continue  # skip transparent background")
    emit("            else:")
    emit("                if _lum < 0.02:")
    emit("                    continue  # skip near-black background pixels")
    emit("            _lum_sum += _lum")
    emit("            _count += 1")
    emit("        if _count == 0:")
    emit("            print('  Lighting check: no object pixels found, skipping.')")
    emit("            break")
    emit("        _avg_lum = _lum_sum / _count")
    emit("        print(f'  Lighting check iter {_iteration}: avg luminance = {_avg_lum:.3f} (target: {target_min:.2f}-{target_max:.2f})')")
    emit("        if target_min <= _avg_lum <= target_max:")
    emit("            print('  Lighting OK.')")
    emit("            break")
    emit("        # Adjust: scale all lights proportionally")
    emit("        _target_mid = (target_min + target_max) / 2")
    emit("        _scale = _target_mid / max(_avg_lum, 0.001)")
    emit("        _scale = max(0.3, min(_scale, 3.0))  # clamp adjustment")
    emit("        for _light in _lights:")
    emit("            _light.data.energy *= _scale")
    emit("        print(f'    Scaled lights by {_scale:.2f}')")
    emit("    # Restore render settings")
    emit("    scene.render.resolution_x = _orig_x")
    emit("    scene.render.resolution_y = _orig_y")
    emit("    scene.cycles.samples = _orig_samples")
    emit("    scene.render.filepath = _orig_filepath")
    emit("    scene.render.image_settings.file_format = _orig_format")
    emit("    scene.frame_set(scene.frame_start)")
    emit("")
    emit("_check_and_adjust_lighting()")
    emit("print()")
