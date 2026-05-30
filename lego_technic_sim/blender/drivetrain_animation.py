"""Generate a Blender Python script for drive train animation.

Shows the gear chain from motor/crank root outward: each driven unit appears
in sequence and spins for a few frames at its correct gear ratio speed,
visualising how power flows through the mechanism.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..physics.drive_train import DriveNode, DriveTree
from ..physics.model import Unit
from .geometry import (
    ldraw_to_blender as _ldraw_to_blender,
    collect_geometry,
    collect_geometry_colored,
    emit_framing_check,
    emit_kinematic_rotation,
    emit_lighting_check,
    parse_ldconfig,
)


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
    presentation: bool = False,
    ldraw_library: Optional[Path] = None,
    build_parts: Optional[List] = None,
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
        presentation:   If True, use realistic LDraw colors, group units
                        by depth (same depth appears simultaneously), and
                        disable text annotations.
        ldraw_library:  Path to LDraw library (for color lookup in presentation mode).
        build_parts:    Full list of LDrawParts from the build (includes connectors).
                        When provided in presentation mode, connectors (axles/pins)
                        between drivetrain units are rendered as static geometry.
    """
    scene = tree.scene
    nodes = tree.all_nodes
    n_nodes = len(nodes)

    # Identify the motor body unit (appears with the first gear but doesn't spin)
    motor_unit_idx: Optional[int] = None
    if scene.motors:
        motor_joint = scene.joints[scene.motors[0].joint_index]
        motor_unit_idx = motor_joint.unit_a_index

    # In presentation mode, group nodes by depth so same-depth units appear together
    if presentation:
        depth_groups: Dict[int, List[int]] = {}
        for idx, node in enumerate(nodes):
            depth_groups.setdefault(node.depth, []).append(idx)
        sorted_depths = sorted(depth_groups.keys())
        n_groups = len(sorted_depths)
        # Compute tail frames: one full revolution of the slowest (leaf) gear
        min_ratio = min(abs(node.accumulated_ratio) for node in nodes)
        tail_frames = int(round(spin_frames / max(min_ratio, 1e-6)))
        tail_frames = max(tail_frames, spin_frames)  # at least one spin period
        total_frames = n_groups * (appear_frames + spin_frames) + tail_frames
    else:
        tail_frames = spin_frames
        total_frames = n_nodes * (appear_frames + spin_frames) + tail_frames

    # Parse LDraw colors for presentation mode
    ldraw_colors: Dict[int, Tuple[float, float, float]] = {}
    if presentation:
        ldraw_colors = parse_ldconfig(ldraw_library)

    # Compute scene bounds for camera (include motor unit if present)
    all_positions = []
    for node in nodes:
        unit = scene.units[node.unit_index]
        pos = _ldraw_to_blender(unit.center_of_mass)
        all_positions.append(pos)
    if motor_unit_idx is not None:
        motor_unit = scene.units[motor_unit_idx]
        all_positions.append(_ldraw_to_blender(motor_unit.center_of_mass))

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
    if presentation:
        emit(f"bpy.ops.object.light_add(type='SUN', location=(0, 0, {cam_distance:.2f}))")
        emit("_sun = bpy.context.active_object")
        emit("_sun.data.energy = 1.5")
        emit("_sun.data.angle = 0.2")
        emit(f"bpy.ops.object.light_add(type='AREA', location=("
             f"{cx:.4f}, {cy - cam_distance * 0.7:.4f}, {cz:.4f}))")
        emit("_fill = bpy.context.active_object")
        emit("_fill.data.energy = 8.0")
        emit("_fill.data.size = 0.5")
        emit(f"_fill.rotation_euler = (1.3, 0, 0)")
        emit("world = bpy.data.worlds.get('World') or bpy.data.worlds.new('World')")
        emit("scene.world = world")
        emit("world.use_nodes = True")
        emit("bg = world.node_tree.nodes.get('Background')")
        emit("if bg:")
        emit("    bg.inputs[0].default_value = (0.0, 0.0, 0.0, 1.0)")
    else:
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

    # LDraw color materials (presentation mode)
    if presentation and ldraw_colors:
        emit("# ── LDraw color materials ──────────────────────────────────")
        emit("_ldraw_colors = {}")
        for code, (r, g, b) in sorted(ldraw_colors.items()):
            emit(f"_ldraw_colors[{code}] = ({r:.4f}, {g:.4f}, {b:.4f})")
        emit("")
        emit("def _get_ldraw_mat(code):")
        emit("    name = f'LDraw_{code}'")
        emit("    mat = bpy.data.materials.get(name)")
        emit("    if mat:")
        emit("        return mat")
        emit("    mat = bpy.data.materials.new(name=name)")
        emit("    mat.use_nodes = True")
        emit("    bsdf = mat.node_tree.nodes.get('Principled BSDF')")
        emit("    if bsdf:")
        emit("        r, g, b = _ldraw_colors.get(code, (0.5, 0.5, 0.5))")
        emit("        bsdf.inputs['Base Color'].default_value = (r, g, b, 1.0)")
        emit("        bsdf.inputs['Roughness'].default_value = 0.3")
        emit("        bsdf.inputs['Specular IOR Level'].default_value = 0.5")
        emit("    return mat")
        emit()

    # Create mesh objects for each drive train node
    emit("_dt_objects = []")
    emit()

    # Motor body (appears with the first gear, does not spin)
    # Show only the motor brick(s), not the whole unit
    if motor_unit_idx is not None:
        from ..physics.motor_detection import is_motor_part
        motor_unit = scene.units[motor_unit_idx]
        motor_bricks = [b for b in motor_unit.bricks if is_motor_part(b.part_id)]
        motor_name = "motor"

        emit(f"# Motor brick (from unit {motor_unit_idx}, static)")
        if presentation:
            vertices, faces, face_colors = collect_geometry_colored(motor_bricks)
        else:
            vertices, faces = collect_geometry(motor_bricks)

        if vertices:
            emit(f"_verts = {vertices!r}")
            emit(f"_faces = {faces!r}")
            emit(f"_mesh = bpy.data.meshes.new({motor_name + '_mesh'!r})")
            emit("_mesh.from_pydata(_verts, [], _faces)")
            if presentation:
                # Assign per-face LDraw materials
                unique_colors = sorted(set(face_colors))
                emit(f"_face_colors = {face_colors!r}")
                emit("_color_set = sorted(set(_face_colors))")
                emit("for _cc in _color_set:")
                emit("    _mesh.materials.append(_get_ldraw_mat(_cc))")
                emit("_mat_idx_map = {_cc: i for i, _cc in enumerate(_color_set)}")
                emit("for _fi, _fc in enumerate(_face_colors):")
                emit("    _mesh.polygons[_fi].material_index = _mat_idx_map[_fc]")
            emit("_mesh.update()")
            emit(f"_motor_obj = bpy.data.objects.new({motor_name!r}, _mesh)")
            emit("bpy.context.collection.objects.link(_motor_obj)")
        else:
            com_bl = _ldraw_to_blender(motor_unit.center_of_mass)
            emit(f"bpy.ops.mesh.primitive_cube_add(size=0.005, "
                 f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))")
            emit("_motor_obj = bpy.context.active_object")
            emit(f"_motor_obj.name = {motor_name!r}")

        if not presentation:
            # Dark grey material for motor
            emit("_mmat = bpy.data.materials.new(name='mat_motor')")
            emit("_mmat.use_nodes = True")
            emit("_mbsdf = _mmat.node_tree.nodes.get('Principled BSDF')")
            emit("if _mbsdf:")
            emit("    _mbsdf.inputs['Base Color'].default_value = (0.25, 0.25, 0.25, 1.0)")
            emit("_motor_obj.data.materials.append(_mmat)")
        emit()

        # Visible from frame 1 (same as first gear)
        emit("_motor_obj.hide_viewport = False")
        emit("_motor_obj.hide_render = False")
        emit()

    # Compute appear_frame per node
    # In presentation mode: group by depth, same depth = same frame
    # In debug mode: sequential, one per node
    node_appear_frames: List[int] = []
    if presentation:
        depth_to_frame = {}
        for group_idx, depth in enumerate(sorted_depths):
            depth_to_frame[depth] = group_idx * (appear_frames + spin_frames) + 1
        for node in nodes:
            node_appear_frames.append(depth_to_frame[node.depth])
    else:
        for node_idx in range(n_nodes):
            node_appear_frames.append(node_idx * (appear_frames + spin_frames) + 1)

    for node_idx, node in enumerate(nodes):
        unit = scene.units[node.unit_index]
        safe_name = f"dt_{node_idx}_{unit.name}".replace('"', "")

        appear_frame = node_appear_frames[node_idx]

        emit(f"# DriveNode {node_idx}: unit {node.unit_index} depth={node.depth} "
             f"ratio={node.accumulated_ratio:.3f}")

        if presentation:
            vertices, faces, face_colors = collect_geometry_colored(unit.bricks)
        else:
            vertices, faces = collect_geometry(unit.bricks)

        if vertices:
            emit(f"_verts = {vertices!r}")
            emit(f"_faces = {faces!r}")
            emit(f"_mesh = bpy.data.meshes.new({safe_name + '_mesh'!r})")
            emit("_mesh.from_pydata(_verts, [], _faces)")
            if presentation:
                # Per-face LDraw materials
                emit(f"_face_colors = {face_colors!r}")
                emit("_color_set = sorted(set(_face_colors))")
                emit("for _cc in _color_set:")
                emit("    _mesh.materials.append(_get_ldraw_mat(_cc))")
                emit("_mat_idx_map = {_cc: i for i, _cc in enumerate(_color_set)}")
                emit("for _fi, _fc in enumerate(_face_colors):")
                emit("    _mesh.polygons[_fi].material_index = _mat_idx_map[_fc]")
            emit("_mesh.update()")
            emit(f"_obj = bpy.data.objects.new({safe_name!r}, _mesh)")
            emit("bpy.context.collection.objects.link(_obj)")
        else:
            com_bl = _ldraw_to_blender(unit.center_of_mass)
            emit(f"bpy.ops.mesh.primitive_cube_add(size=0.005, "
                 f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))")
            emit("_obj = bpy.context.active_object")
            emit(f"_obj.name = {safe_name!r}")

        if not presentation:
            # Material - colour by depth (debug mode)
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

        # Rotation: spin around hinge joint axis at rate proportional to ratio.
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
            pivot_bl = _ldraw_to_blender(unit.center_of_mass)
            axis_bl = _ldraw_axis_to_blender(node.axis)
        norm = float(np.linalg.norm(axis_bl))
        if norm > 1e-12:
            axis_bl = axis_bl / norm

        # One full revolution per spin_frames at ratio=1
        base_rps = 1.0
        revolutions = node.accumulated_ratio * base_rps
        angle_per_frame = revolutions * 2.0 * np.pi / spin_frames

        emit(f"# Spin: ratio={node.accumulated_ratio:.3f}, "
             f"angle/frame={angle_per_frame:.4f} rad")
        emit_kinematic_rotation(emit, "_obj", pivot_bl, axis_bl, angle_per_frame)
        emit("_dt_objects.append(_kin_obj)")
        emit()

    # Render connector parts (axles/pins) in presentation mode
    if presentation and build_parts:
        from ..ldraw.model import LDrawBuild
        from ..physics.connectors import is_connector
        from ..physics.unit_builder import _find_port_connections

        # Only include connectors whose ALL connections are within the DT
        dt_unit_indices = {node.unit_index for node in nodes}
        if motor_unit_idx is not None:
            dt_unit_indices.add(motor_unit_idx)

        # Build part identity → unit index lookup
        brick_to_unit: Dict[int, int] = {}
        for uid, unit in enumerate(scene.units):
            for brick in unit.bricks:
                brick_to_unit[id(brick)] = uid

        connector_indices = [i for i, p in enumerate(build_parts)
                            if is_connector(p.part_id)]
        structural_tuples = [(i, build_parts[i]) for i in range(len(build_parts))
                            if not is_connector(build_parts[i].part_id)]

        unit_connectors: Dict[int, List] = {}
        for ci in connector_indices:
            conn_part = build_parts[ci]
            connections = _find_port_connections(conn_part, structural_tuples)
            if not connections:
                continue
            # Get the units this connector touches
            connected_units = set()
            for gi, ct in connections:
                part = build_parts[gi]
                uid = brick_to_unit.get(id(part))
                if uid is not None:
                    connected_units.add(uid)
            # Only include connectors that bridge two different DT units
            if len(connected_units) > 1 and connected_units.issubset(dt_unit_indices):
                # Assign to the latest-appearing unit it connects
                primary_uid = max(connected_units,
                                  key=lambda u: next(
                                      (node_appear_frames[ni] for ni, n in enumerate(nodes)
                                       if n.unit_index == u), 1))
                unit_connectors.setdefault(primary_uid, []).append(conn_part)

        if unit_connectors:
            emit("# ── Connectors (axles/pins) ────────────────────────────────")
            for uid, conn_parts in sorted(unit_connectors.items()):
                vertices, faces, face_colors = collect_geometry_colored(conn_parts)
                if not vertices:
                    continue
                # Find the appear frame for this unit
                # Motor unit appears at frame 1
                if uid == motor_unit_idx:
                    conn_appear = 1
                else:
                    # Find which node has this unit_index
                    conn_appear = 1
                    for ni, node in enumerate(nodes):
                        if node.unit_index == uid:
                            conn_appear = node_appear_frames[ni]
                            break

                cname = f"conn_u{uid}"
                emit(f"_conn_verts = {vertices!r}")
                emit(f"_conn_faces = {faces!r}")
                emit(f"_conn_mesh = bpy.data.meshes.new('{cname}_mesh')")
                emit("_conn_mesh.from_pydata(_conn_verts, [], _conn_faces)")
                emit(f"_conn_face_colors = {face_colors!r}")
                emit("_conn_color_set = sorted(set(_conn_face_colors))")
                emit("for _cc in _conn_color_set:")
                emit("    _conn_mesh.materials.append(_get_ldraw_mat(_cc))")
                emit("_conn_mat_map = {_cc: i for i, _cc in enumerate(_conn_color_set)}")
                emit("for _fi, _fc in enumerate(_conn_face_colors):")
                emit("    _conn_mesh.polygons[_fi].material_index = _conn_mat_map[_fc]")
                emit("_conn_mesh.update()")
                emit(f"_conn_obj = bpy.data.objects.new('{cname}', _conn_mesh)")
                emit("bpy.context.collection.objects.link(_conn_obj)")
                # Visibility: appear with the unit they belong to
                emit("_conn_obj.hide_viewport = True")
                emit("_conn_obj.hide_render = True")
                emit("_conn_obj.keyframe_insert(data_path='hide_viewport', frame=1)")
                emit("_conn_obj.keyframe_insert(data_path='hide_render', frame=1)")
                if conn_appear > 1:
                    emit(f"_conn_obj.keyframe_insert(data_path='hide_viewport', frame={conn_appear - 1})")
                    emit(f"_conn_obj.keyframe_insert(data_path='hide_render', frame={conn_appear - 1})")
                emit("_conn_obj.hide_viewport = False")
                emit("_conn_obj.hide_render = False")
                emit(f"_conn_obj.keyframe_insert(data_path='hide_viewport', frame={conn_appear})")
                emit(f"_conn_obj.keyframe_insert(data_path='hide_render', frame={conn_appear})")
                emit("_dt_objects.append(_conn_obj)")
                emit()

    # Set constant interpolation for visibility
    emit("for obj in _dt_objects:")
    emit("    if obj.animation_data and obj.animation_data.action:")
    emit("        for fcurve in obj.animation_data.action.fcurves:")
    emit("            for kp in fcurve.keyframe_points:")
    emit("                kp.interpolation = 'CONSTANT'")
    emit()

    # Stamp overlay with per-frame unit labels (debug mode only)
    if not presentation:
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
        emit()

        # Build frame→label map
        frame_labels: dict[int, str] = {}
        for node_idx, node in enumerate(nodes):
            af = node_appear_frames[node_idx]
            frame_labels[af] = (
                f"Unit {node.unit_index} (depth={node.depth}, "
                f"ratio={node.accumulated_ratio:.2f})"
            )
        emit(f"_frame_labels = {frame_labels!r}")
        emit("def _dt_label_handler(scene_ref):")
        emit("    frame = scene_ref.frame_current")
        emit("    label = 'Drive Train'")
        emit("    for f in sorted(_frame_labels.keys()):")
        emit("        if f <= frame:")
        emit("            label = _frame_labels[f]")
        emit("    scene_ref.render.stamp_note_text = label")
        emit()
        emit("bpy.app.handlers.frame_change_pre.append(_dt_label_handler)")
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

    # Framing auto-adjustment (zoom to fit across all frames)
    if presentation:
        emit_framing_check(emit, objects_var="_dt_objects", margin=0.05)

    # Lighting auto-adjustment (presentation mode)
    if presentation:
        emit_lighting_check(emit, check_frame=total_frames)

    # Render
    emit("print(f'Rendering {scene.frame_end} drive train frames...')")
    emit("bpy.ops.render.render(animation=True)")
    emit("print('Drive train animation complete.')")

    script = "\n".join(lines)
    if output_path is not None:
        Path(output_path).write_text(script, encoding="utf-8")
    return script
