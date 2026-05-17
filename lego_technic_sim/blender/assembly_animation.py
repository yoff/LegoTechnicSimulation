"""Generate a Blender Python script that renders an assembly animation.

The generated script shows rigid units appearing one by one, giving a
step-by-step assembly visualisation.  It is self-contained and can be run via::

    blender --background --python assembly_animation.py

The rendered frames are saved to an output directory (default: ``//assembly_``
relative to the blend file, or ``/tmp/assembly_`` in background mode).
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Set

import numpy as np

from ..ldraw.model import LDrawBuild, LDrawPart
from ..physics.connectors import is_connector
from ..physics.mesh_properties import LDU_TO_METERS
from ..physics.model import PhysicsScene, Unit
from ..physics.unit_builder import _connector_shaft, _find_port_connections
from .geometry import ldraw_to_blender as _ldraw_to_blender, collect_geometry


def _map_connectors_to_units(
    build: LDrawBuild,
    scene: PhysicsScene,
) -> Dict[int, int]:
    """Map each connector part index to the unit it should be displayed with.

    A connector is assigned to the first rigid-connected unit it touches.
    If all connections are revolute, it is assigned to the first connected
    unit.  Returns a dict mapping connector part index → unit index.
    """
    # Build part identity → unit index lookup
    brick_to_unit: Dict[int, int] = {}
    for uid, unit in enumerate(scene.units):
        for brick in unit.bricks:
            brick_to_unit[id(brick)] = uid

    connector_indices = [i for i, p in enumerate(build.parts) if is_connector(p.part_id)]
    structural_tuples = [
        (i, build.parts[i]) for i in range(len(build.parts))
        if not is_connector(build.parts[i].part_id)
    ]

    result: Dict[int, int] = {}
    for ci in connector_indices:
        conn_part = build.parts[ci]
        connections = _find_port_connections(conn_part, structural_tuples)
        if not connections:
            continue

        # Prefer a rigid connection's unit
        rigid_gi = [gi for gi, ct in connections if ct == "rigid"]
        revolute_gi = [gi for gi, ct in connections if ct == "revolute"]

        target_gi = rigid_gi[0] if rigid_gi else (revolute_gi[0] if revolute_gi else None)
        if target_gi is not None:
            part = build.parts[target_gi]
            uid = brick_to_unit.get(id(part))
            if uid is not None:
                result[ci] = uid

    return result


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
    build: Optional[LDrawBuild] = None,
    model_path: Optional[Path] = None,
    ldraw_library: Optional[Path] = None,
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
        build:           The original LDraw build.  When provided, connector
                         geometry (pins, axles) is included in the
                         visualisation and each unit (except 0) gets a solo
                         frame before appearing in the full assembly.
        model_path:      Absolute path to the ``.ldr`` model file.  When
                         provided (together with *ldraw_library*), the
                         generated script parses the model at render time
                         instead of embedding geometry inline.
        ldraw_library:   Absolute path to the LDraw parts library root.

    Returns:
        The generated Python script as a string.
    """
    _runtime_geometry = model_path is not None and ldraw_library is not None

    lines: List[str] = []

    def emit(text: str = "") -> None:
        lines.append(text)

    # Map connectors to their display unit (if build is provided)
    connector_to_unit: Dict[int, int] = {}
    unit_connectors: Dict[int, List[LDrawPart]] = {}
    show_solo = False
    if build is not None:
        connector_to_unit = _map_connectors_to_units(build, scene)
        for ci, uid in connector_to_unit.items():
            unit_connectors.setdefault(uid, []).append(build.parts[ci])
        show_solo = True

    # With solo frames: unit 0 gets 1 frame (assembly only), every other
    # unit gets 2 frames (solo then assembly).
    if show_solo:
        total_unit_frames = (1 + (len(scene.units) - 1) * 2) * frames_per_unit
    else:
        total_unit_frames = len(scene.units) * frames_per_unit
    total_frames = total_unit_frames + hold_frames

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
    # Runtime geometry preamble
    # ------------------------------------------------------------------
    if _runtime_geometry:
        import lego_technic_sim as _pkg
        pkg_root = str(Path(_pkg.__file__).resolve().parent.parent)
        emit("# ── Parse model for geometry at render time ─────────────────")
        emit("import sys")
        emit(f"sys.path.insert(0, {pkg_root!r})")
        emit("from lego_technic_sim.ldraw.parser import LDrawParser")
        emit("from lego_technic_sim.physics.unit_builder import build_units_and_joints")
        emit("from lego_technic_sim.blender.geometry import collect_geometry")
        emit()
        emit(f"print('Parsing LDraw model...')")
        emit(f"_build = LDrawParser({str(ldraw_library)!r}).parse_build({str(model_path)!r})")
        emit("_scene = build_units_and_joints(_build)")
        emit(f"print(f'Loaded {{len(_scene.units)}} units')")
        # Connector-to-unit mapping (computed at emit time, small dict)
        if connector_to_unit:
            emit(f"_connector_to_unit = {connector_to_unit!r}")
            emit("_unit_connectors = {}")
            emit("for _ci, _uid in _connector_to_unit.items():")
            emit("    _unit_connectors.setdefault(_uid, []).append(_build.parts[_ci])")
        else:
            emit("_unit_connectors = {}")
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
    emit("_solo_objs = []  # solo-frame duplicates (visible for 1 frame only)")
    emit()

    # Assign colours to units for visual distinction
    emit("import colorsys")
    emit(f"_n_units = {len(scene.units)}")
    emit()

    # Compute appear frame for each unit.  Unit 0 has no solo frame;
    # every other unit gets a solo frame then an assembly frame.
    appear_frames: List[int] = []  # frame where unit appears in assembly
    solo_frames: List[Optional[int]] = []  # frame for solo view (None for unit 0)
    frame_cursor = 1
    for idx in range(len(scene.units)):
        if idx == 0 or not show_solo:
            solo_frames.append(None)
            appear_frames.append(frame_cursor)
            frame_cursor += frames_per_unit
        else:
            solo_frames.append(frame_cursor)
            frame_cursor += frames_per_unit
            appear_frames.append(frame_cursor)
            frame_cursor += frames_per_unit

    for idx, unit in enumerate(scene.units):
        com_bl = _ldraw_to_blender(unit.center_of_mass)
        safe_name = unit.name.replace('"', "")
        appear_frame = appear_frames[idx]
        solo_frame = solo_frames[idx]

        emit(f"# Unit {idx}: {safe_name} (appears at frame {appear_frame})")

        if _runtime_geometry:
            # Geometry loaded at render time from the parsed model
            emit(f"_all_bricks = list(_scene.units[{idx}].bricks) + _unit_connectors.get({idx}, [])")
            emit("_verts, _faces = collect_geometry(_all_bricks)")
            emit("if _verts:")
            emit(f"    _mesh = bpy.data.meshes.new({safe_name + '_mesh'!r})")
            emit("    _mesh.from_pydata(_verts, [], _faces)")
            emit("    _mesh.update()")
            emit(f"    _obj = bpy.data.objects.new({safe_name!r}, _mesh)")
            emit("    bpy.context.collection.objects.link(_obj)")
            emit("else:")
            emit(
                f"    bpy.ops.mesh.primitive_cube_add("
                f"size=0.005, "
                f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))"
            )
            emit("    _obj = bpy.context.active_object")
            emit(f"    _obj.name = {safe_name!r}")
            # Track whether we have geometry for the solo frame
            emit("_has_geo = bool(_verts)")
        else:
            # Inline geometry (legacy path)
            all_bricks = list(unit.bricks) + unit_connectors.get(idx, [])
            vertices, faces = collect_geometry(all_bricks)
            if vertices:
                emit(f"_verts = {vertices!r}")
                emit(f"_faces = {faces!r}")
                emit(f"_mesh = bpy.data.meshes.new({safe_name + '_mesh'!r})")
                emit("_mesh.from_pydata(_verts, [], _faces)")
                emit("_mesh.update()")
                emit(f"_obj = bpy.data.objects.new({safe_name!r}, _mesh)")
                emit("bpy.context.collection.objects.link(_obj)")
            else:
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

        # Solo frame: show this unit alone for one frame before it joins
        if solo_frame is not None:
            if _runtime_geometry:
                # Guard solo-frame creation with runtime geometry check
                emit(f"if _has_geo:  # solo preview of unit {idx}")
                _indent = "    "
            elif vertices:
                emit(f"# Solo preview of unit {idx} at frame {solo_frame}")
                _indent = ""
            else:
                _indent = None  # skip — no geometry in inline path

            if _indent is not None:
                solo_name = f"solo_{idx}"
                emit(f"{_indent}_solo_mesh = bpy.data.meshes.new({solo_name + '_mesh'!r})")
                emit(f"{_indent}_solo_mesh.from_pydata(_verts, [], _faces)")
                emit(f"{_indent}_solo_mesh.update()")
                emit(f"{_indent}_solo_obj = bpy.data.objects.new({solo_name!r}, _solo_mesh)")
                emit(f"{_indent}bpy.context.collection.objects.link(_solo_obj)")
                # Use a brighter version of the same colour
                emit(f"{_indent}_smat = bpy.data.materials.new(name='solo_mat_{idx}')")
                emit(f"{_indent}_smat.use_nodes = True")
                emit(f"{_indent}_sbsdf = _smat.node_tree.nodes.get('Principled BSDF')")
                emit(f"{_indent}_sr, _sg, _sb = colorsys.hsv_to_rgb(_hue, 0.9, 1.0)")
                emit(f"{_indent}if _sbsdf:")
                emit(f"{_indent}    _sbsdf.inputs['Base Color'].default_value = (_sr, _sg, _sb, 1.0)")
                emit(f"{_indent}_solo_obj.data.materials.append(_smat)")
                # Visible only on the solo frame
                emit(f"{_indent}_solo_obj.hide_viewport = True")
                emit(f"{_indent}_solo_obj.hide_render = True")
                emit(f"{_indent}_solo_obj.keyframe_insert(data_path='hide_viewport', frame=1)")
                emit(f"{_indent}_solo_obj.keyframe_insert(data_path='hide_render', frame=1)")
                if solo_frame > 1:
                    emit(f"{_indent}_solo_obj.keyframe_insert(data_path='hide_viewport', frame={solo_frame - 1})")
                    emit(f"{_indent}_solo_obj.keyframe_insert(data_path='hide_render', frame={solo_frame - 1})")
                emit(f"{_indent}_solo_obj.hide_viewport = False")
                emit(f"{_indent}_solo_obj.hide_render = False")
                emit(f"{_indent}_solo_obj.keyframe_insert(data_path='hide_viewport', frame={solo_frame})")
                emit(f"{_indent}_solo_obj.keyframe_insert(data_path='hide_render', frame={solo_frame})")
                # Hide again after the solo frame
                emit(f"{_indent}_solo_obj.hide_viewport = True")
                emit(f"{_indent}_solo_obj.hide_render = True")
                emit(f"{_indent}_solo_obj.keyframe_insert(data_path='hide_viewport', frame={solo_frame + 1})")
                emit(f"{_indent}_solo_obj.keyframe_insert(data_path='hide_render', frame={solo_frame + 1})")
                emit(f"{_indent}_solo_objs.append(_solo_obj)")
                emit()

    # ------------------------------------------------------------------
    # Hide assembly units during solo frames
    # ------------------------------------------------------------------
    if show_solo:
        emit("# ── Hide assembly objects on solo frames ──────────────────")
        emit("# On each solo frame, all previously-visible assembly units")
        emit("# must be temporarily hidden so only the solo object is seen.")
        for idx in range(len(scene.units)):
            sf = solo_frames[idx]
            af = appear_frames[idx]
            if sf is None:
                continue
            # On the solo frame, hide every assembly unit that appeared before
            for prev in range(idx):
                prev_af = appear_frames[prev]
                if prev_af <= sf:
                    emit(f"_units[{prev}].hide_render = True")
                    emit(f"_units[{prev}].keyframe_insert(data_path='hide_render', frame={sf})")
                    emit(f"_units[{prev}].hide_viewport = True")
                    emit(f"_units[{prev}].keyframe_insert(data_path='hide_viewport', frame={sf})")
                    # Restore on the assembly frame
                    emit(f"_units[{prev}].hide_render = False")
                    emit(f"_units[{prev}].keyframe_insert(data_path='hide_render', frame={af})")
                    emit(f"_units[{prev}].hide_viewport = False")
                    emit(f"_units[{prev}].keyframe_insert(data_path='hide_viewport', frame={af})")
        emit()

    # ------------------------------------------------------------------
    # Set keyframe interpolation to constant (instant appear/disappear)
    # ------------------------------------------------------------------
    emit("# ── Set constant interpolation (instant visibility switch) ──")
    emit("for obj in _units + _solo_objs:")
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
    # Emit frame→label map so the handler knows what to display
    frame_labels: Dict[int, str] = {}
    for idx in range(len(scene.units)):
        sf = solo_frames[idx]
        af = appear_frames[idx]
        if sf is not None:
            frame_labels[sf] = f"Unit {idx} (solo)"
        frame_labels[af] = f"Unit {idx}"
    emit(f"_frame_labels = {frame_labels!r}")
    emit("def _unit_label_handler(scene_ref):")
    emit("    frame = scene_ref.frame_current")
    emit("    # Walk backwards to find the most recent label")
    emit("    label = ''")
    emit("    for f in sorted(_frame_labels.keys()):")
    emit("        if f <= frame:")
    emit("            label = _frame_labels[f]")
    emit("    scene_ref.render.stamp_note_text = label")
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
