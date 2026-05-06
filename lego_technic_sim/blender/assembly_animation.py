"""Generate a Blender Python script that renders an assembly animation.

The generated script shows rigid units appearing one by one, giving a
step-by-step assembly visualisation.  It is self-contained and can be run via::

    blender --background --python assembly_animation.py

The rendered frames are saved to an output directory (default: ``//assembly_``
relative to the blend file, or ``/tmp/assembly_`` in background mode).
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import numpy as np

from ..physics.mesh_properties import LDU_TO_METERS
from ..physics.model import PhysicsScene, Unit


def _ldraw_to_blender(v: np.ndarray) -> np.ndarray:
    """Convert a 3-D point from LDraw space to Blender space."""
    return np.array([v[0], -v[2], -v[1]], dtype=float)


def generate_assembly_animation(
    scene: PhysicsScene,
    output_path: Optional[Path] = None,
    frames_per_unit: int = 10,
    hold_frames: int = 30,
    render_output: str = "/tmp/assembly_",
    resolution_x: int = 1280,
    resolution_y: int = 720,
    render_format: str = "FFMPEG",
    cycles_samples: int = 32,
) -> str:
    """Generate a Blender script that renders units appearing in sequence.

    Args:
        scene:           The physics scene containing units to animate.
        output_path:     If provided, write the script to this file.
        frames_per_unit: Frames between each unit appearing.
        hold_frames:     Extra frames to hold after all units are visible.
        render_output:   Output path for the rendered animation.
        resolution_x:    Render width in pixels.
        resolution_y:    Render height in pixels.
        render_format:   Blender output format ('FFMPEG', 'PNG', 'JPEG').
        cycles_samples:  Number of Cycles render samples (lower = faster).

    Returns:
        The generated Python script as a string.
    """
    lines: List[str] = []

    def emit(text: str = "") -> None:
        lines.append(text)

    total_frames = len(scene.units) * frames_per_unit + hold_frames

    # Compute scene bounds for camera placement
    all_positions = []
    for unit in scene.units:
        bl_pos = _ldraw_to_blender(unit.center_of_mass)
        all_positions.append(bl_pos)

    if all_positions:
        positions_arr = np.array(all_positions)
        scene_center = positions_arr.mean(axis=0)
        scene_extent = positions_arr.max(axis=0) - positions_arr.min(axis=0)
        cam_distance = float(np.linalg.norm(scene_extent)) * 1.2 + 0.1
    else:
        scene_center = np.zeros(3)
        cam_distance = 5.0

    # ------------------------------------------------------------------
    # Script header
    # ------------------------------------------------------------------
    emit("# Auto-generated Blender assembly animation script.")
    emit("# Created by lego_technic_sim – do not edit by hand.")
    emit()
    emit("import bpy")
    emit("import mathutils")
    emit("import math")
    emit()

    # ------------------------------------------------------------------
    # Scene setup
    # ------------------------------------------------------------------
    emit("# ── Scene setup ──────────────────────────────────────────────")
    emit("bpy.ops.object.select_all(action='SELECT')")
    emit("bpy.ops.object.delete(use_global=False)")
    emit()
    emit("# Remove default collections' remaining objects")
    emit("for obj in bpy.data.objects:")
    emit("    bpy.data.objects.remove(obj, do_unlink=True)")
    emit()
    emit("scene = bpy.context.scene")
    emit(f"scene.frame_start = 1")
    emit(f"scene.frame_end = {total_frames}")
    emit("scene.render.fps = 24")
    emit(f"scene.render.resolution_x = {resolution_x}")
    emit(f"scene.render.resolution_y = {resolution_y}")
    emit(f"scene.render.filepath = {render_output!r}")
    if render_format == "FFMPEG":
        emit(f"scene.render.image_settings.file_format = 'FFMPEG'")
        emit("scene.render.ffmpeg.format = 'MPEG4'")
        emit("scene.render.ffmpeg.codec = 'H264'")
    else:
        emit(f"scene.render.image_settings.file_format = {render_format!r}")
    emit()
    emit("# Use Cycles with CPU for headless compatibility")
    emit("scene.render.engine = 'CYCLES'")
    emit("scene.cycles.device = 'CPU'")
    emit(f"scene.cycles.samples = {cycles_samples}")
    emit()

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------
    emit("# ── Camera ─────────────────────────────────────────────────")
    cx, cy, cz = scene_center
    emit(f"bpy.ops.object.camera_add(location=("
         f"{cx + cam_distance * 0.6:.6f}, "
         f"{cy - cam_distance * 0.8:.6f}, "
         f"{cz + cam_distance * 0.5:.6f}))")
    emit("_cam = bpy.context.active_object")
    emit("scene.camera = _cam")
    emit(f"_target = mathutils.Vector(({cx:.6f}, {cy:.6f}, {cz:.6f}))")
    emit("_direction = _target - _cam.location")
    emit("_cam.rotation_euler = _direction.to_track_quat('-Z', 'Y').to_euler()")
    emit()

    # ------------------------------------------------------------------
    # Lighting
    # ------------------------------------------------------------------
    emit("# ── Lighting ───────────────────────────────────────────────")
    emit(f"bpy.ops.object.light_add(type='SUN', location=(0, 0, {cam_distance:.2f}))")
    emit("_sun = bpy.context.active_object")
    emit("_sun.data.energy = 3.0")
    emit()
    emit("# Set a simple world background")
    emit("world = bpy.data.worlds.get('World') or bpy.data.worlds.new('World')")
    emit("scene.world = world")
    emit("world.use_nodes = True")
    emit("bg = world.node_tree.nodes.get('Background')")
    emit("if bg:")
    emit("    bg.inputs[0].default_value = (0.05, 0.05, 0.08, 1.0)")
    emit()

    # ------------------------------------------------------------------
    # Create unit objects with visibility keyframes
    # ------------------------------------------------------------------
    emit("# ── Units (appearing in sequence) ───────────────────────────")
    emit("_units = []")
    emit()

    # Assign colours to units for visual distinction
    emit("import colorsys")
    emit(f"_n_units = {len(scene.units)}")
    emit()

    for idx, unit in enumerate(scene.units):
        com_bl = _ldraw_to_blender(unit.center_of_mass)
        safe_name = unit.name.replace('"', "")
        appear_frame = idx * frames_per_unit + 1

        # Collect all triangles from all bricks in this unit,
        # converting from LDraw LDU to Blender metres.
        vertices: List[List[float]] = []
        faces: List[List[int]] = []
        vi = 0
        for brick in unit.bricks:
            for tri in brick.triangles:
                v0 = _ldraw_to_blender(tri.v0) * LDU_TO_METERS
                v1 = _ldraw_to_blender(tri.v1) * LDU_TO_METERS
                v2 = _ldraw_to_blender(tri.v2) * LDU_TO_METERS
                vertices.append([round(float(v0[0]), 7), round(float(v0[1]), 7), round(float(v0[2]), 7)])
                vertices.append([round(float(v1[0]), 7), round(float(v1[1]), 7), round(float(v1[2]), 7)])
                vertices.append([round(float(v2[0]), 7), round(float(v2[1]), 7), round(float(v2[2]), 7)])
                faces.append([vi, vi + 1, vi + 2])
                vi += 3

        emit(f"# Unit {idx}: {safe_name} (appears at frame {appear_frame})")

        if vertices:
            emit(f"_verts = {vertices!r}")
            emit(f"_faces = {faces!r}")
            emit(f"_mesh = bpy.data.meshes.new({safe_name + '_mesh'!r})")
            emit("_mesh.from_pydata(_verts, [], _faces)")
            emit("_mesh.update()")
            emit(f"_obj = bpy.data.objects.new({safe_name!r}, _mesh)")
            emit("bpy.context.collection.objects.link(_obj)")
        else:
            # Fallback: empty cube if no geometry
            emit(
                f"bpy.ops.mesh.primitive_cube_add("
                f"size=0.005, "
                f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))"
            )
            emit("_obj = bpy.context.active_object")
            emit(f"_obj.name = {safe_name!r}")

        emit()

        # Material with unique colour
        emit(f"_mat = bpy.data.materials.new(name='mat_{idx}')")
        emit("_mat.use_nodes = True")
        emit("_bsdf = _mat.node_tree.nodes.get('Principled BSDF')")
        emit(f"_hue = {idx} / max(_n_units, 1)")
        emit("_r, _g, _b = colorsys.hsv_to_rgb(_hue, 0.7, 0.9)")
        emit("if _bsdf:")
        emit("    _bsdf.inputs['Base Color'].default_value = (_r, _g, _b, 1.0)")
        emit("_obj.data.materials.append(_mat)")
        emit()

        # Visibility keyframes: hidden before appear_frame, visible after
        emit(f"_obj.hide_viewport = True")
        emit(f"_obj.hide_render = True")
        emit(f"_obj.keyframe_insert(data_path='hide_viewport', frame=1)")
        emit(f"_obj.keyframe_insert(data_path='hide_render', frame=1)")
        if appear_frame > 1:
            emit(f"_obj.keyframe_insert(data_path='hide_viewport', frame={appear_frame - 1})")
            emit(f"_obj.keyframe_insert(data_path='hide_render', frame={appear_frame - 1})")
        emit(f"_obj.hide_viewport = False")
        emit(f"_obj.hide_render = False")
        emit(f"_obj.keyframe_insert(data_path='hide_viewport', frame={appear_frame})")
        emit(f"_obj.keyframe_insert(data_path='hide_render', frame={appear_frame})")
        emit("_units.append(_obj)")
        emit()

    # ------------------------------------------------------------------
    # Set keyframe interpolation to constant (instant appear/disappear)
    # ------------------------------------------------------------------
    emit("# ── Set constant interpolation (instant visibility switch) ──")
    emit("for obj in _units:")
    emit("    if obj.animation_data and obj.animation_data.action:")
    emit("        for fcurve in obj.animation_data.action.fcurves:")
    emit("            for kp in fcurve.keyframe_points:")
    emit("                kp.interpolation = 'CONSTANT'")
    emit()

    # ------------------------------------------------------------------
    # Text overlay showing unit number on each frame
    # ------------------------------------------------------------------
    emit("# ── Unit number overlay ──────────────────────────────────────")
    emit("scene.use_nodes = True")
    emit("tree = scene.node_tree")
    emit("for node in tree.nodes:")
    emit("    tree.nodes.remove(node)")
    emit("rl = tree.nodes.new('CompositorNodeRLayers')")
    emit("rl.location = (0, 0)")
    emit("comp = tree.nodes.new('CompositorNodeComposite')")
    emit("comp.location = (600, 0)")
    emit("txt = tree.nodes.new('CompositorNodeOutputFile')")
    emit("txt.location = (600, -200)")
    emit("")
    emit("# Create text strip via handler that updates each frame")
    emit("def _unit_label_handler(scene_ref):")
    emit(f"    fpu = {frames_per_unit}")
    emit("    frame = scene_ref.frame_current")
    emit("    unit_idx = min((frame - 1) // fpu, len(_units) - 1)")
    emit("    # Update metadata text (shown in stamp)")
    emit("    scene_ref.render.stamp_note_text = f'Unit {unit_idx}'")
    emit("")
    emit("bpy.app.handlers.frame_change_pre.append(_unit_label_handler)")
    emit("scene.render.use_stamp = True")
    emit("scene.render.use_stamp_note = True")
    emit("scene.render.stamp_font_size = 24")
    emit("scene.render.use_stamp_date = False")
    emit("scene.render.use_stamp_time = False")
    emit("scene.render.use_stamp_render_time = False")
    emit("scene.render.use_stamp_frame = True")
    emit("scene.render.use_stamp_camera = False")
    emit("scene.render.use_stamp_scene = False")
    emit("scene.render.use_stamp_filename = False")
    emit("scene.render.use_stamp_memory = False")
    emit("scene.render.use_stamp_hostname = False")
    emit("scene.render.stamp_foreground = (1, 1, 1, 1)")
    emit("scene.render.stamp_background = (0, 0, 0, 0.6)")
    emit("")
    emit("# Connect compositor: render layers → composite output")
    emit("tree.links.new(rl.outputs['Image'], comp.inputs['Image'])")
    emit()

    # ------------------------------------------------------------------
    # Render
    # ------------------------------------------------------------------
    emit("# ── Render animation ────────────────────────────────────────")
    emit("print(f'Rendering {scene.frame_end} frames to {scene.render.filepath}...')")
    emit("bpy.ops.render.render(animation=True)")
    emit("print('Assembly animation render complete.')")

    script = "\n".join(lines)

    if output_path is not None:
        Path(output_path).write_text(script, encoding="utf-8")

    return script
