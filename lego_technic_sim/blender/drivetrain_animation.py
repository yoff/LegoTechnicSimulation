"""Generate a Blender Python script for drive train animation.

Shows the gear chain from motor/crank root outward: each driven unit appears
in sequence and spins for a few frames at its correct gear ratio speed,
visualising how power flows through the mechanism.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import numpy as np

from ..physics.drive_train import DriveNode, DriveTree
from ..physics.mesh_properties import LDU_TO_METERS
from ..physics.model import Unit


def _ldraw_to_blender(v: np.ndarray) -> np.ndarray:
    """Convert LDraw coords to Blender coords."""
    return np.array([v[0], -v[2], -v[1]], dtype=float)


def _ldraw_axis_to_blender(a: np.ndarray) -> np.ndarray:
    """Convert LDraw axis direction to Blender space."""
    return np.array([a[0], -a[2], -a[1]], dtype=float)


def generate_drivetrain_animation(
    tree: DriveTree,
    output_path: Optional[Path] = None,
    render_output: str = "/tmp/drivetrain_",
    resolution_x: int = 1280,
    resolution_y: int = 720,
    cycles_samples: int = 32,
    spin_frames: int = 48,
    appear_frames: int = 6,
) -> str:
    """Generate Blender script showing the drive train in action.

    Each node in the tree appears in BFS order, then spins at its
    accumulated gear ratio for `spin_frames` frames before the next
    node appears.

    Args:
        tree:           DriveTree from build_drive_train().
        output_path:    Write script to this file.
        render_output:  Blender render output path prefix.
        resolution_x:   Render width.
        resolution_y:   Render height.
        cycles_samples: Cycles samples.
        spin_frames:    Frames each node spins before next appears.
        appear_frames:  Frames to pause when a new node appears (before spin).
    """
    scene = tree.scene
    nodes = tree.all_nodes
    n_nodes = len(nodes)

    # Total frames: each node gets appear_frames + spin_frames
    total_frames = n_nodes * (appear_frames + spin_frames) + spin_frames

    # Compute scene bounds for camera
    all_positions = []
    for node in nodes:
        unit = scene.units[node.unit_index]
        pos = _ldraw_to_blender(unit.center_of_mass)
        all_positions.append(pos)

    if all_positions:
        positions_arr = np.array(all_positions)
        scene_center = positions_arr.mean(axis=0)
        scene_extent = positions_arr.max(axis=0) - positions_arr.min(axis=0)
        cam_distance = float(np.linalg.norm(scene_extent)) * 1.2 + 0.1
    else:
        scene_center = np.zeros(3)
        cam_distance = 5.0

    lines: List[str] = []

    def emit(text: str = "") -> None:
        lines.append(text)

    # Header
    emit("# Auto-generated drive train animation script.")
    emit("import bpy, mathutils, math, colorsys")
    emit()

    # Scene setup
    emit("bpy.ops.object.select_all(action='SELECT')")
    emit("bpy.ops.object.delete(use_global=False)")
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
    emit("scene.render.image_settings.file_format = 'FFMPEG'")
    emit("scene.render.ffmpeg.format = 'MPEG4'")
    emit("scene.render.ffmpeg.codec = 'H264'")
    emit("scene.render.engine = 'CYCLES'")
    emit("scene.cycles.device = 'CPU'")
    emit(f"scene.cycles.samples = {cycles_samples}")
    emit()

    # Camera
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

    # Lighting
    emit(f"bpy.ops.object.light_add(type='SUN', location=(0, 0, {cam_distance:.2f}))")
    emit("_sun = bpy.context.active_object")
    emit("_sun.data.energy = 3.0")
    emit("world = bpy.data.worlds.get('World') or bpy.data.worlds.new('World')")
    emit("scene.world = world")
    emit("world.use_nodes = True")
    emit("bg = world.node_tree.nodes.get('Background')")
    emit("if bg:")
    emit("    bg.inputs[0].default_value = (0.05, 0.05, 0.08, 1.0)")
    emit()

    # Create mesh objects for each drive train node
    emit("_dt_objects = []")
    emit()

    for node_idx, node in enumerate(nodes):
        unit = scene.units[node.unit_index]
        safe_name = f"dt_{node_idx}_{unit.name}".replace('"', "")

        # Collect mesh
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

        appear_frame = node_idx * (appear_frames + spin_frames) + 1

        emit(f"# DriveNode {node_idx}: unit {node.unit_index} depth={node.depth} "
             f"ratio={node.accumulated_ratio:.3f}")

        if vertices:
            emit(f"_verts = {vertices!r}")
            emit(f"_faces = {faces!r}")
            emit(f"_mesh = bpy.data.meshes.new({safe_name + '_mesh'!r})")
            emit("_mesh.from_pydata(_verts, [], _faces)")
            emit("_mesh.update()")
            emit(f"_obj = bpy.data.objects.new({safe_name!r}, _mesh)")
            emit("bpy.context.collection.objects.link(_obj)")
        else:
            com_bl = _ldraw_to_blender(unit.center_of_mass)
            emit(f"bpy.ops.mesh.primitive_cube_add(size=0.005, "
                 f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))")
            emit("_obj = bpy.context.active_object")
            emit(f"_obj.name = {safe_name!r}")

        # Material - colour by depth
        emit(f"_mat = bpy.data.materials.new(name='mat_dt_{node_idx}')")
        emit("_mat.use_nodes = True")
        emit("_bsdf = _mat.node_tree.nodes.get('Principled BSDF')")
        emit(f"_hue = {node.depth} / max({max(n.depth for n in nodes) + 1}, 1)")
        emit("_r, _g, _b = colorsys.hsv_to_rgb(_hue, 0.8, 0.95)")
        emit("if _bsdf:")
        emit("    _bsdf.inputs['Base Color'].default_value = (_r, _g, _b, 1.0)")
        emit("_obj.data.materials.append(_mat)")
        emit()

        # Visibility keyframes
        emit("_obj.hide_viewport = True")
        emit("_obj.hide_render = True")
        emit(f"_obj.keyframe_insert(data_path='hide_viewport', frame=1)")
        emit(f"_obj.keyframe_insert(data_path='hide_render', frame=1)")
        if appear_frame > 1:
            emit(f"_obj.keyframe_insert(data_path='hide_viewport', frame={appear_frame - 1})")
            emit(f"_obj.keyframe_insert(data_path='hide_render', frame={appear_frame - 1})")
        emit("_obj.hide_viewport = False")
        emit("_obj.hide_render = False")
        emit(f"_obj.keyframe_insert(data_path='hide_viewport', frame={appear_frame})")
        emit(f"_obj.keyframe_insert(data_path='hide_render', frame={appear_frame})")
        emit()

        # Rotation keyframes: spin around hinge joint axis at rate proportional
        # to ratio.  The pivot and axis come from the revolute joint that
        # connects this gear unit to its parent/frame — NOT from the gear mesh
        # axis, which can differ for bevel gears.
        from ..physics.model import JointType

        hinge_joint = None
        for j in scene.joints:
            if j.joint_type != JointType.REVOLUTE:
                continue
            if node.unit_index in (j.unit_a_index, j.unit_b_index):
                hinge_joint = j
                break

        if hinge_joint is not None:
            pivot_bl = _ldraw_to_blender(hinge_joint.position)
            axis_bl = _ldraw_axis_to_blender(hinge_joint.axis)
        else:
            # Fallback: use COM and gear mesh axis
            pivot_bl = _ldraw_to_blender(unit.center_of_mass)
            axis_bl = _ldraw_axis_to_blender(node.axis)
        norm = float(np.linalg.norm(axis_bl))
        if norm > 1e-12:
            axis_bl = axis_bl / norm

        # Set origin to center of mass for rotation
        emit(f"_obj.location = (0, 0, 0)")
        emit(f"# Rotation pivot at hinge joint")
        emit(f"_pivot = mathutils.Vector(({pivot_bl[0]:.6f}, {pivot_bl[1]:.6f}, {pivot_bl[2]:.6f}))")
        emit(f"_axis = mathutils.Vector(({axis_bl[0]:.6f}, {axis_bl[1]:.6f}, {axis_bl[2]:.6f}))")
        emit()

        # Keyframe rotation: from appear_frame to end of its spin window
        spin_start = appear_frame + appear_frames
        # All nodes keep spinning once they appear (until end of animation)
        # Angular displacement per frame: base_speed * ratio * (2π / spin_frames)
        # So one full revolution per spin_frames at ratio=1
        base_rps = 1.0  # revolutions per spin_frames
        revolutions = node.accumulated_ratio * base_rps
        angle_per_frame = revolutions * 2.0 * np.pi / spin_frames

        emit(f"# Spin: ratio={node.accumulated_ratio:.3f}, "
             f"angle/frame={angle_per_frame:.4f} rad")
        emit(f"_angle_per_frame = {angle_per_frame:.8f}")
        emit(f"_spin_start = {spin_start}")
        emit(f"_appear = {appear_frame}")
        emit()

        # We'll keyframe at spin_start and at the end of the animation
        # Using a frame_change handler for smooth continuous rotation
        emit(f"_dt_objects.append((_obj, _pivot, _axis, _angle_per_frame, _spin_start))")
        emit()

    # Frame handler for rotation
    emit("# ── Rotation handler ──────────────────────────────────────────")
    emit("def _drivetrain_spin(scene_ref):")
    emit("    frame = scene_ref.frame_current")
    emit("    for obj, pivot, axis, angle_per_frame, spin_start in _dt_objects:")
    emit("        if frame < spin_start:")
    emit("            continue")
    emit("        elapsed = frame - spin_start")
    emit("        angle = angle_per_frame * elapsed")
    emit("        rot = mathutils.Matrix.Rotation(angle, 4, axis)")
    emit("        # Apply rotation around pivot")
    emit("        obj.matrix_world = (")
    emit("            mathutils.Matrix.Translation(pivot)")
    emit("            @ rot")
    emit("            @ mathutils.Matrix.Translation(-pivot)")
    emit("        )")
    emit()
    emit("bpy.app.handlers.frame_change_pre.append(_drivetrain_spin)")
    emit()

    # Set constant interpolation for visibility
    emit("for obj, *_ in _dt_objects:")
    emit("    if obj.animation_data and obj.animation_data.action:")
    emit("        for fcurve in obj.animation_data.action.fcurves:")
    emit("            for kp in fcurve.keyframe_points:")
    emit("                kp.interpolation = 'CONSTANT'")
    emit()

    # Stamp overlay
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
    emit(f"scene.render.stamp_note_text = 'Drive Train'")
    emit()

    # Compositing
    emit("scene.use_nodes = True")
    emit("tree = scene.node_tree")
    emit("for node in tree.nodes:")
    emit("    tree.nodes.remove(node)")
    emit("rl = tree.nodes.new('CompositorNodeRLayers')")
    emit("comp = tree.nodes.new('CompositorNodeComposite')")
    emit("tree.links.new(rl.outputs['Image'], comp.inputs['Image'])")
    emit()

    # Render
    emit("print(f'Rendering {scene.frame_end} drive train frames...')")
    emit("bpy.ops.render.render(animation=True)")
    emit("print('Drive train animation complete.')")

    script = "\n".join(lines)
    if output_path is not None:
        Path(output_path).write_text(script, encoding="utf-8")
    return script
