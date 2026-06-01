"""Generate a Blender Python script that sets up a rigid-body physics scene.

The generated script is self-contained and can be executed inside Blender's
*Scripting* workspace (or via ``blender --background --python simulation.py``).
It will:

1. Delete any existing scene objects.
2. Enable the Blender Rigid Body world.
3. Create a mesh object for every :class:`~lego_technic_sim.physics.model.Unit`.
4. Add Rigid Body Constraints (Empty objects) for every
   :class:`~lego_technic_sim.physics.model.Joint`.
5. Configure angular-motor parameters for every
   :class:`~lego_technic_sim.physics.model.Motor`.
6. Optionally render the simulation.

Coordinate-system conversion
-----------------------------
LDraw uses a right-handed system where **Y points downward** (positive Y = toward
the floor).  Blender uses a right-handed system where **Z points upward**.  The
mapping applied here is::

    Blender X  =  LDraw X
    Blender Y  = -LDraw Z
    Blender Z  = -LDraw Y

Usage
-----
::

    from lego_technic_sim.blender.exporter import generate_blender_script
    script = generate_blender_script(scene, output_path="simulation.py")
    # Then in Blender: exec(open("simulation.py").read())
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import numpy as np

from ..physics.mesh_properties import LDU_TO_METERS
from ..physics.model import Joint, JointType, Motor, PhysicsScene, Unit
from ..physics.drive_train import build_drive_train
from .geometry import (
    ldraw_to_blender as _ldraw_to_blender,
    collect_geometry,
    collect_geometry_colored,
    parse_ldconfig,
    emit_framing_check,
    emit_kinematic_rotation,
    emit_lighting_check,
)


def generate_blender_script(
    scene: PhysicsScene,
    output_path: Optional[Path] = None,
    fps: int = 60,
    gravity: Optional[np.ndarray] = None,
    render: bool = False,
    render_output: str = "/tmp/simulation_",
    resolution_x: int = 1280,
    resolution_y: int = 720,
    cycles_samples: int = 32,
    sim_frames: int = 120,
    use_mesh: bool = True,
    follow_unit: Optional[int] = None,
    anchor_motor: bool = False,
    collision_mode: str = "convex_hull",
    model_path: Optional[Path] = None,
    ldraw_library: Optional[Path] = None,
    gltf_export: Optional[str] = None,
) -> str:
    """Generate a Blender Python script for the given physics scene.

    Args:
        scene:       The :class:`~lego_technic_sim.physics.model.PhysicsScene`
                     to export.
        output_path: If provided, write the script to this file as well as
                     returning it.
        fps:         Blender render frame rate (also used for the rigid-body
                     world step rate).
        gravity:     Gravity vector in Blender space (m/s²).  Defaults to
                     ``[0, 0, -9.81]`` (downward in Blender's Z-up world).
        render:      If True, add render commands to the script.
        render_output: Output path for rendered animation.
        resolution_x: Render width.
        resolution_y: Render height.
        cycles_samples: Cycles samples for rendering.
        sim_frames:  Number of frames to simulate/render.
        use_mesh:    If True, create actual part meshes; if False, use cubes.
        follow_unit: If set, camera tracks this unit index during simulation.
        model_path:  Absolute path to the ``.ldr`` model file.  When provided
                     (together with *ldraw_library*), the generated script
                     imports the package and parses the model at render time
                     instead of embedding geometry inline.
        ldraw_library: Absolute path to the LDraw parts library root.
        gltf_export: If provided, export the animated scene to a glTF/GLB
                     file at this path after baking the simulation.

    Returns:
        The generated Python script as a string.
    """
    if gravity is None:
        gravity = np.array([0.0, 0.0, -9.81])

    # Determine whether to parse geometry at runtime (thin script) or
    # embed it inline (legacy behaviour for tests / when paths are absent).
    _runtime_geometry = model_path is not None and ldraw_library is not None

    lines: List[str] = []

    def emit(text: str = "") -> None:
        lines.append(text)

    # ------------------------------------------------------------------
    # Header
    # ------------------------------------------------------------------
    emit("# Auto-generated Blender physics simulation script.")
    emit("# Created by lego_technic_sim – do not edit by hand.")
    emit()
    emit("import bpy")
    emit("import mathutils")
    emit("import colorsys")
    emit()

    # ------------------------------------------------------------------
    # Runtime geometry preamble (when model/library paths are available)
    # ------------------------------------------------------------------
    if _runtime_geometry:
        import lego_technic_sim as _pkg
        pkg_root = str(Path(_pkg.__file__).resolve().parent.parent)
        emit("# ── Parse model for geometry at render time ─────────────────")
        emit("import sys")
        emit(f"sys.path.insert(0, {pkg_root!r})")
        emit("from lego_technic_sim.ldraw.parser import LDrawParser")
        emit("from lego_technic_sim.physics.unit_builder import build_units_and_joints")
        emit("from lego_technic_sim.blender.geometry import collect_geometry, collect_geometry_colored")
        emit()
        emit(f"print('Parsing LDraw model...')")
        emit(f"_build = LDrawParser({str(ldraw_library)!r}).parse_build({str(model_path)!r})")
        emit("_scene = build_units_and_joints(_build)")
        emit(f"print(f'Loaded {{len(_scene.units)}} units')")
        emit()

    # ------------------------------------------------------------------
    # LDraw color table (parsed at script-generation time)
    # ------------------------------------------------------------------
    _ldraw_colors = parse_ldconfig(ldraw_library)
    if _ldraw_colors:
        emit("# ── LDraw color table ─────────────────────────────────────────")
        emit(f"_ldraw_colors = {dict(_ldraw_colors)!r}")
        emit()
    else:
        emit("_ldraw_colors = {}")
        emit()

    # ------------------------------------------------------------------
    # Scene setup
    # ------------------------------------------------------------------
    emit("# ── Scene setup ──────────────────────────────────────────────")
    emit("bpy.ops.object.select_all(action='SELECT')")
    emit("bpy.ops.object.delete(use_global=False)")
    emit("for obj in bpy.data.objects:")
    emit("    bpy.data.objects.remove(obj, do_unlink=True)")
    emit()
    emit("scene = bpy.context.scene")
    emit(f"scene.frame_start = 1")
    emit(f"scene.frame_end = {sim_frames}")
    emit(f"scene.render.fps = {fps}")
    emit("if scene.rigidbody_world:")
    emit("    bpy.ops.rigidbody.world_remove()")
    emit("bpy.ops.rigidbody.world_add()")
    emit("scene.rigidbody_world.time_scale = 1.0")
    emit("scene.rigidbody_world.substeps_per_frame = 60")
    emit("scene.rigidbody_world.solver_iterations = 60")
    emit(f"scene.rigidbody_world.point_cache.frame_end = {sim_frames}")
    emit(
        f"scene.gravity = ({gravity[0]:.6f}, {gravity[1]:.6f}, {gravity[2]:.6f})"
    )
    emit()

    if render:
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

    # ------------------------------------------------------------------
    # Camera & Lighting (only when rendering)
    # ------------------------------------------------------------------
    if render:
        # Compute scene bounds from actual mesh geometry (in metres)
        all_pts: list[np.ndarray] = []
        for unit in scene.units:
            for brick in unit.bricks:
                for tri in brick.triangles:
                    for v in (tri.v0, tri.v1, tri.v2):
                        all_pts.append(_ldraw_to_blender(np.asarray(v)) * LDU_TO_METERS)
        if all_pts:
            pts_arr = np.array(all_pts)
            scene_min = pts_arr.min(axis=0)
            scene_max = pts_arr.max(axis=0)
            scene_center = (scene_min + scene_max) / 2
            scene_extent = scene_max - scene_min
            cam_distance = float(np.linalg.norm(scene_extent)) * 3.0 + 0.05
        else:
            scene_center = np.zeros(3)
            cam_distance = 0.5

        cx, cy, cz = scene_center
        emit("# ── Camera ─────────────────────────────────────────────────")
        if follow_unit is not None:
            # Position camera relative to the followed unit's initial position
            follow_pos = _ldraw_to_blender(scene.units[follow_unit].center_of_mass)
            fx, fy, fz = follow_pos
            emit(f"bpy.ops.object.camera_add(location=("
                 f"{fx + cam_distance * 0.4:.6f}, "
                 f"{fy - cam_distance * 0.6:.6f}, "
                 f"{fz + cam_distance * 0.3:.6f}))")
        else:
            emit(f"bpy.ops.object.camera_add(location=("
                 f"{cx + cam_distance * 0.6:.6f}, "
                 f"{cy - cam_distance * 0.8:.6f}, "
                 f"{cz + cam_distance * 0.5:.6f}))")
        emit("_cam = bpy.context.active_object")
        emit("scene.camera = _cam")
        emit("_cam.data.clip_start = 0.001")
        emit("_cam.data.clip_end = 100.0")

        if follow_unit is None:
            emit(f"_target = mathutils.Vector(({cx:.6f}, {cy:.6f}, {cz:.6f}))")
            emit("_direction = _target - _cam.location")
            emit("_cam.rotation_euler = _direction.to_track_quat('-Z', 'Y').to_euler()")
        emit()
        emit("# ── Lighting ───────────────────────────────────────────────")
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
        emit()

    # ------------------------------------------------------------------
    # Build drive train for kinematic gear animation
    # ------------------------------------------------------------------
    drive_tree = build_drive_train(scene)

    # Identify unit roles:
    # - kinematic_units: internal gears (PASSIVE, driver-animated)
    # - output_units: drive-tree leaves that connect to dynamic legs (ACTIVE)
    # - chassis_units: motor body (PASSIVE, static)
    kinematic_units: set[int] = set()
    output_units: set[int] = set()
    chassis_unit: int = 0  # motor body

    # Map unit → drive node for speed/axis lookup
    unit_to_node: dict = {}

    if drive_tree:
        gear_unit_set = {n.unit_index for n in drive_tree.all_nodes}
        for node in drive_tree.all_nodes:
            unit_to_node[node.unit_index] = node

        # Leaf nodes in drive tree (no children) are output units (cranks)
        for node in drive_tree.all_nodes:
            if not node.children:
                output_units.add(node.unit_index)
            else:
                kinematic_units.add(node.unit_index)
    else:
        gear_unit_set = set()

    # Chassis/motor units
    _motor_unit_indices = set()
    if scene.motors:
        motor_joint = scene.joints[scene.motors[0].joint_index]
        chassis_unit = motor_joint.unit_a_index
    if anchor_motor:
        # Pin the chassis (and motor body) as PASSIVE kinematic
        _motor_unit_indices.add(chassis_unit)
        for motor in scene.motors:
            joint = scene.joints[motor.joint_index]
            _motor_unit_indices.add(joint.unit_a_index)

    # Find hinge joints for kinematic gear units (for driver animation)
    # and for output units (for torque-limited MOTOR constraints)
    unit_hinge: dict[int, Joint] = {}
    for j in scene.joints:
        if j.joint_type != JointType.REVOLUTE:
            continue
        for uid in (j.unit_a_index, j.unit_b_index):
            if uid in kinematic_units or uid in output_units:
                if uid not in unit_hinge:
                    unit_hinge[uid] = j

    # ------------------------------------------------------------------
    # Kinematic closure: detect units in closed loops back to the
    # kinematic zone.  These units' positions are fully determined by
    # the crank angles, so they should be kinematic too.
    # ------------------------------------------------------------------
    # Build adjacency graph (revolute joints only — fixed joints are rigid)
    from collections import defaultdict, deque
    adj: dict[int, set[int]] = defaultdict(set)
    for j in scene.joints:
        if j.joint_type == JointType.REVOLUTE:
            adj[j.unit_a_index].add(j.unit_b_index)
            adj[j.unit_b_index].add(j.unit_a_index)

    # Seed: chassis + kinematic gears + output cranks
    # (chassis is always in kinematic_zone for closure detection, even when
    # it is ACTIVE for physics — its position anchors the linkage graph)
    kinematic_zone = _motor_unit_indices | kinematic_units | output_units
    kinematic_zone.add(chassis_unit)

    # Kinematic closure: a non-kinematic unit is part of a closed loop if
    # it lies on a path between two kinematic-zone units (through other
    # non-kinematic units).  Detect by finding connected components of
    # non-kinematic units that touch >=2 kinematic-zone units.
    non_kin = set(range(len(scene.units))) - kinematic_zone
    visited_nk: set[int] = set()
    for start in non_kin:
        if start in visited_nk:
            continue
        # BFS to find this connected component (through non-kin units only)
        component: set[int] = set()
        queue: deque[int] = deque([start])
        kin_neighbors: set[int] = set()
        while queue:
            u = queue.popleft()
            if u in component:
                continue
            component.add(u)
            for nb in adj[u]:
                if nb in kinematic_zone:
                    kin_neighbors.add(nb)
                elif nb not in component:
                    queue.append(nb)
        visited_nk |= component
        # If this component connects to >=2 kinematic-zone units, all its
        # members are in a closed loop and should be kinematic.
        if len(kin_neighbors) >= 2:
            kinematic_zone |= component

    # Units added by closure (not gears/motor/output/chassis)
    linkage_units = kinematic_zone - _motor_unit_indices - kinematic_units - output_units
    linkage_units.discard(chassis_unit)

    # Find hinge pivots for linkage units too
    for j in scene.joints:
        if j.joint_type != JointType.REVOLUTE:
            continue
        for uid in (j.unit_a_index, j.unit_b_index):
            if uid in linkage_units and uid not in unit_hinge:
                unit_hinge[uid] = j

    emit("# ── Units (rigid bodies) ─────────────────────────────────────")
    emit(f"_n_units = {len(scene.units)}")
    emit("_units = []")
    emit()

    for idx, unit in enumerate(scene.units):
        com_bl = _ldraw_to_blender(unit.center_of_mass)
        safe_name = unit.name.replace('"', "")
        emit(f"# Unit {idx}: {safe_name}")

        if use_mesh:
            if _runtime_geometry:
                # Geometry loaded at render time from the parsed model
                emit(f"_verts, _faces, _face_colors = collect_geometry_colored(_scene.units[{idx}].bricks)")
                emit("if _verts:")
                emit(f"    _mesh = bpy.data.meshes.new({safe_name + '_mesh'!r})")
                emit("    _mesh.from_pydata(_verts, [], _faces)")
                # Per-face color materials
                emit("    _color_set = sorted(set(_face_colors))")
                emit("    _color_to_slot = {}")
                emit("    for _ci, _cc in enumerate(_color_set):")
                emit("        _rgb = _ldraw_colors.get(_cc, (0.5, 0.5, 0.5))")
                emit("        _mat_name = f'ldraw_{_cc}'")
                emit("        _mat = bpy.data.materials.get(_mat_name)")
                emit("        if _mat is None:")
                emit("            _mat = bpy.data.materials.new(name=_mat_name)")
                emit("            _mat.use_nodes = True")
                emit("            _bsdf = _mat.node_tree.nodes.get('Principled BSDF')")
                emit("            if _bsdf:")
                emit("                _bsdf.inputs['Base Color'].default_value = (_rgb[0], _rgb[1], _rgb[2], 1.0)")
                emit("                _bsdf.inputs['Roughness'].default_value = 0.3")
                emit("                _bsdf.inputs['Specular IOR Level'].default_value = 0.5")
                emit("        _mesh.materials.append(_mat)")
                emit("        _color_to_slot[_cc] = _ci")
                emit("    for _fi, _fc in enumerate(_face_colors):")
                emit("        _mesh.polygons[_fi].material_index = _color_to_slot[_fc]")
                emit("    _mesh.update()")
                emit(f"    _obj = bpy.data.objects.new({safe_name!r}, _mesh)")
                emit("    bpy.context.collection.objects.link(_obj)")
                emit("    bpy.context.view_layer.objects.active = _obj")
                emit("    _obj.select_set(True)")
                emit("else:")
                emit(
                    f"    bpy.ops.mesh.primitive_cube_add("
                    f"size=0.005, "
                    f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))"
                )
                emit("    _obj = bpy.context.active_object")
                emit(f"    _obj.name = {safe_name!r}")
            else:
                # Inline geometry (legacy path for tests)
                vertices, faces = collect_geometry(unit.bricks)
                if vertices:
                    emit(f"_verts = {vertices!r}")
                    emit(f"_faces = {faces!r}")
                    emit(f"_mesh = bpy.data.meshes.new({safe_name + '_mesh'!r})")
                    emit("_mesh.from_pydata(_verts, [], _faces)")
                    emit("_mesh.update()")
                    emit(f"_obj = bpy.data.objects.new({safe_name!r}, _mesh)")
                    emit("bpy.context.collection.objects.link(_obj)")
                    emit("bpy.context.view_layer.objects.active = _obj")
                    emit("_obj.select_set(True)")
                else:
                    emit(
                        f"bpy.ops.mesh.primitive_cube_add("
                        f"size=0.005, "
                        f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))"
                    )
                    emit("_obj = bpy.context.active_object")
                    emit(f"_obj.name = {safe_name!r}")
        else:
            emit(
                f"bpy.ops.mesh.primitive_cube_add("
                f"size=0.02, "
                f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))"
            )
            emit("_obj = bpy.context.active_object")
            emit(f"_obj.name = {safe_name!r}")

        emit("bpy.ops.rigidbody.object_add()")
        # Ensure minimum mass so physics solver doesn't skip bodies
        mass = max(unit.mass, 0.001)
        emit(f"_obj.rigid_body.mass = {mass:.6f}")
        # Kinematic units: motor body, gears, output cranks, and linkage units
        # Chassis is ACTIVE when not anchored (free to move under physics)
        if idx == chassis_unit and not anchor_motor:
            emit("_obj.rigid_body.type = 'ACTIVE'")
        elif idx in kinematic_zone:
            emit("_obj.rigid_body.type = 'PASSIVE'")
            emit("_obj.rigid_body.kinematic = True")
        else:
            emit("_obj.rigid_body.type = 'ACTIVE'")
        _shape = {"convex_hull": "CONVEX_HULL", "mesh": "MESH", "none": "CONVEX_HULL"}[collision_mode]
        emit(f"_obj.rigid_body.collision_shape = '{_shape}'")
        if collision_mode == "mesh":
            emit("_obj.rigid_body.mesh_source = 'FINAL'")
        emit("_obj.rigid_body.friction = 0.5")
        emit("_obj.rigid_body.restitution = 0.0")
        emit("_obj.rigid_body.linear_damping = 0.04")
        emit("_obj.rigid_body.angular_damping = 0.1")
        if not anchor_motor:
            # Separate collision layers: chassis+ground in collection 0,
            # kinematic units in collection 1 (so legs don't push chassis)
            if idx == chassis_unit:
                emit("_obj.rigid_body.collision_collections[0] = True")
                emit("_obj.rigid_body.collision_collections[1] = False")
            else:
                emit("_obj.rigid_body.collision_collections[0] = False")
                emit("_obj.rigid_body.collision_collections[1] = True")
        elif collision_mode == "none":
            # Put units in collision collection 1 (not 0) so they only
            # collide with objects in collection 1 (i.e. the ground).
            emit("_obj.rigid_body.collision_collections[0] = False")
            emit("_obj.rigid_body.collision_collections[1] = True")

        # Material (fallback for non-runtime or non-mesh paths)
        if render and not (_runtime_geometry and use_mesh):
            emit(f"_mat = bpy.data.materials.new(name='mat_{idx}')")
            emit("_mat.use_nodes = True")
            emit("_bsdf = _mat.node_tree.nodes.get('Principled BSDF')")
            emit(f"_hue = {idx} / max(_n_units, 1)")
            emit("_r, _g, _b = colorsys.hsv_to_rgb(_hue, 0.7, 0.9)")
            emit("if _bsdf:")
            emit("    _bsdf.inputs['Base Color'].default_value = (_r, _g, _b, 1.0)")
            emit("_obj.data.materials.append(_mat)")

        emit("_units.append(_obj)")
        emit()

    # ------------------------------------------------------------------
    # Camera tracking (after units are created)
    # ------------------------------------------------------------------
    if render and follow_unit is not None:
        emit("# ── Camera Track To constraint ─────────────────────────────")
        emit("_track = _cam.constraints.new(type='TRACK_TO')")
        emit(f"_track.target = _units[{follow_unit}]")
        emit("_track.track_axis = 'TRACK_NEGATIVE_Z'")
        emit("_track.up_axis = 'UP_Y'")
        emit()

    # ------------------------------------------------------------------
    # Ground plane
    # ------------------------------------------------------------------
    emit("# ── Ground plane ───────────────────────────────────────────")
    # Place ground just below the lowest unit
    all_z = [_ldraw_to_blender(u.center_of_mass)[2] for u in scene.units]
    ground_z = min(all_z) - 0.05 if all_z else -0.1
    emit(f"bpy.ops.mesh.primitive_plane_add(size=2.0, location=(0, 0, {ground_z:.4f}))")
    emit("_ground = bpy.context.active_object")
    emit("_ground.name = 'Ground'")
    emit("bpy.ops.rigidbody.object_add()")
    emit("_ground.rigid_body.type = 'PASSIVE'")
    emit("_ground.rigid_body.collision_shape = 'BOX'")
    emit("_ground.rigid_body.friction = 0.8")
    if not anchor_motor:
        # Ground in collection 0 only — collides with chassis, not legs
        emit("_ground.rigid_body.collision_collections[0] = True")
        emit("_ground.rigid_body.collision_collections[1] = False")
    elif collision_mode == "none":
        # Ground must be in collection 1 to collide with units
        emit("_ground.rigid_body.collision_collections[1] = True")
    emit("_ground.hide_render = True")
    emit()

    # ------------------------------------------------------------------
    # Joints → rigid body constraints
    # ------------------------------------------------------------------
    # Skip joints between two kinematic/chassis units (no physics needed).
    # Convert joints from chassis to output units into MOTOR constraints.
    emit("# ── Joints (constraints) ─────────────────────────────────────")
    emit("_joints = []")
    emit()

    # All kinematic-zone units (gears, cranks, linkage) stay PASSIVE.
    # Skip joints between them — their motion is handled kinematically.
    passive_set = kinematic_zone

    for idx, joint in enumerate(scene.joints):
        ua, ub = joint.unit_a_index, joint.unit_b_index
        # Skip joints where both sides are kinematic
        if ua in passive_set and ub in passive_set:
            emit(f"# Joint {idx}: skipped (both units kinematic)")
            emit("_joints.append(None)")
            emit()
            continue

        pos_bl = _ldraw_to_blender(joint.position)
        axis_bl = _ldraw_to_blender(joint.axis)
        axis_norm = float(np.linalg.norm(axis_bl))
        if axis_norm > 1e-12:
            axis_bl = axis_bl / axis_norm

        # Standard joint (HINGE, FIXED, SLIDER)
        blender_type = {
            JointType.REVOLUTE: "HINGE",
            JointType.FIXED: "FIXED",
            JointType.SLIDER: "SLIDER",
        }[joint.joint_type]

        emit(
            f"# Joint {idx}: {joint.joint_type.name} "
            f"(unit {ua} ↔ unit {ub})"
        )
        emit(
            f"bpy.ops.object.empty_add("
            f"location=({pos_bl[0]:.6f}, {pos_bl[1]:.6f}, {pos_bl[2]:.6f}))"
        )
        emit("_con = bpy.context.active_object")
        emit(f"_con.name = 'joint_{idx}'")
        emit(f"bpy.ops.rigidbody.constraint_add(type={blender_type!r})")
        emit(f"_con.rigid_body_constraint.object1 = _units[{ua}]")
        emit(f"_con.rigid_body_constraint.object2 = _units[{ub}]")
        emit("_con.rigid_body_constraint.disable_collisions = True")
        emit("_con.rigid_body_constraint.use_breaking = False")

        if joint.joint_type == JointType.REVOLUTE:
            emit(
                f"_axis = mathutils.Vector("
                f"({axis_bl[0]:.6f}, {axis_bl[1]:.6f}, {axis_bl[2]:.6f}))"
            )
            emit("_up = mathutils.Vector((0.0, 0.0, 1.0))")
            emit("_rot = _up.rotation_difference(_axis)")
            emit("_con.rotation_mode = 'QUATERNION'")
            emit("_con.rotation_quaternion = _rot")

        emit("_joints.append(_con)")
        emit()

    # ------------------------------------------------------------------
    # Kinematic gear animation (drivers on PASSIVE gear units)
    # ------------------------------------------------------------------
    if kinematic_units and drive_tree and scene.motors:
        motor_speed = scene.motors[0].speed
        emit("# ── Kinematic gear animation (driver expressions) ──────────")
        emit("# Internal gear units are PASSIVE and driven by scripted")
        emit("# rotation expressions at exact gear ratios.")
        emit()

        for uid in sorted(kinematic_units):
            node = unit_to_node.get(uid)
            hinge = unit_hinge.get(uid)
            if node is None or hinge is None:
                continue

            gear_speed = motor_speed * node.accumulated_ratio
            angle_per_frame = gear_speed / fps

            pivot_bl = _ldraw_to_blender(hinge.position)
            axis_bl = _ldraw_to_blender(hinge.axis)
            ax_norm = float(np.linalg.norm(axis_bl))
            if ax_norm > 1e-12:
                axis_bl = axis_bl / ax_norm

            emit(f"# Gear unit {uid}: speed={gear_speed:.4f} rad/s, "
                 f"angle/frame={angle_per_frame:.6f}")
            emit_kinematic_rotation(emit, f"_units[{uid}]", pivot_bl, axis_bl, angle_per_frame)
            emit("_kin_obj.rigid_body.kinematic = True")
            emit()

    # ------------------------------------------------------------------
    # Armature-based IK linkage solver
    # ------------------------------------------------------------------
    # All linkage + output units form closed kinematic loops.  Blender's
    # physics solver cannot handle these (cranks stall).  Instead:
    # 1. Output cranks get kinematic rotation drivers (like gears).
    # 2. BFS from ground pivots (anchors) through linkage units to crank
    #    joints (driven points) produces IK chains.
    # 3. One armature bone per unit on each chain.  IK targets are Empties
    #    parented to the crank mesh (move with motor).
    # 4. Remaining units form closure chains between solved positions.
    # 5. Meshes parented to bones — no frame_change handler needed.
    if linkage_units and output_units and drive_tree and scene.motors:
        motor_speed = scene.motors[0].speed

        # --- Crank rotation (same as gear drivers) ---
        emit("# ── Kinematic output crank animation ──────────────────────────")
        emit()
        for uid in sorted(output_units):
            node = unit_to_node.get(uid)
            hinge = unit_hinge.get(uid)
            if node is None or hinge is None:
                continue
            crank_speed = motor_speed * node.accumulated_ratio
            angle_per_frame = crank_speed / fps
            pivot_bl = _ldraw_to_blender(hinge.position)
            axis_bl = _ldraw_to_blender(hinge.axis)
            ax_norm = float(np.linalg.norm(axis_bl))
            if ax_norm > 1e-12:
                axis_bl = axis_bl / ax_norm
            emit(f"# Output crank unit {uid}: speed={crank_speed:.4f} rad/s")
            emit_kinematic_rotation(
                emit, f"_units[{uid}]", pivot_bl, axis_bl, angle_per_frame
            )
            emit("_kin_obj.rigid_body.kinematic = True")
            emit()

        # --- Build joint position lookup ---
        from collections import defaultdict, deque

        # (uid_a, uid_b) → Blender-space joint position
        _joint_pos: dict[tuple, np.ndarray] = {}
        for j in scene.joints:
            if j.joint_type != JointType.REVOLUTE:
                continue
            pos = _ldraw_to_blender(j.position)
            _joint_pos[(j.unit_a_index, j.unit_b_index)] = pos
            _joint_pos[(j.unit_b_index, j.unit_a_index)] = pos

        def jpos(ua: int, ub: int) -> np.ndarray:
            return _joint_pos.get((ua, ub), np.zeros(3))

        # --- Identify anchors and driven points ---
        anchors: dict[int, np.ndarray] = {}  # linkage_uid → ground pivot pos
        for j in scene.joints:
            if j.joint_type != JointType.REVOLUTE:
                continue
            if j.unit_a_index == chassis_unit and j.unit_b_index in linkage_units:
                anchors.setdefault(j.unit_b_index, _ldraw_to_blender(j.position))
            elif j.unit_b_index == chassis_unit and j.unit_a_index in linkage_units:
                anchors.setdefault(j.unit_a_index, _ldraw_to_blender(j.position))

        driven: dict[tuple, np.ndarray] = {}  # (link_uid, crank_uid) → pos
        for j in scene.joints:
            if j.joint_type != JointType.REVOLUTE:
                continue
            if j.unit_a_index in output_units and j.unit_b_index in linkage_units:
                driven[(j.unit_b_index, j.unit_a_index)] = _ldraw_to_blender(j.position)
            elif j.unit_b_index in output_units and j.unit_a_index in linkage_units:
                driven[(j.unit_a_index, j.unit_b_index)] = _ldraw_to_blender(j.position)

        # --- BFS: primary chains (anchor → driven) ---
        adj_linkage: dict[int, set] = defaultdict(set)
        for j in scene.joints:
            if j.joint_type != JointType.REVOLUTE:
                continue
            ua, ub = j.unit_a_index, j.unit_b_index
            if ua in linkage_units and ub in linkage_units:
                adj_linkage[ua].add(ub)
                adj_linkage[ub].add(ua)

        # Each chain: (anchor_uid, crank_uid, path=[linkage units from anchor to driven])
        primary_chains: list = []
        primary_units: set = set()
        for anchor_uid in sorted(anchors.keys()):
            parent_map: dict[int, int | None] = {anchor_uid: None}
            queue: deque = deque([anchor_uid])
            while queue:
                u = queue.popleft()
                for nb in adj_linkage[u]:
                    if nb not in parent_map:
                        parent_map[nb] = u
                        queue.append(nb)
            for (link_uid, crank_uid), _ in sorted(driven.items()):
                if link_uid in parent_map:
                    path = []
                    cur: int | None = link_uid
                    while cur is not None:
                        path.append(cur)
                        cur = parent_map[cur]
                    path.reverse()
                    primary_chains.append((anchor_uid, crank_uid, path))
                    primary_units.update(path)

        # --- Closure chains (remaining units between solved positions) ---
        remaining_units = linkage_units - primary_units
        # Iteratively find closure chains, adding solved units each round
        closure_chains: list = []  # (start_uid, end_uid, path)
        solved_so_far = set(primary_units)

        while remaining_units:
            # Build adjacency within current remaining set
            remaining_adj: dict[int, set] = defaultdict(set)
            for j in scene.joints:
                if j.joint_type != JointType.REVOLUTE:
                    continue
                ua, ub = j.unit_a_index, j.unit_b_index
                if ua in remaining_units and ub in remaining_units:
                    remaining_adj[ua].add(ub)
                    remaining_adj[ub].add(ua)

            # Find which remaining units touch a solved unit
            remaining_anchors: dict[int, int] = {}
            for j in scene.joints:
                if j.joint_type != JointType.REVOLUTE:
                    continue
                ua, ub = j.unit_a_index, j.unit_b_index
                if ua in remaining_units and ub in solved_so_far:
                    remaining_anchors.setdefault(ua, ub)
                elif ub in remaining_units and ua in solved_so_far:
                    remaining_anchors.setdefault(ub, ua)

            if not remaining_anchors:
                break  # no more solvable units

            # BFS from each anchor, find shortest path to another anchor
            found_chain = False
            for start_uid in sorted(remaining_anchors.keys()):
                if start_uid not in remaining_units:
                    continue
                parent_map: dict[int, int | None] = {start_uid: None}
                queue = deque([start_uid])
                while queue:
                    u = queue.popleft()
                    for nb in remaining_adj[u]:
                        if nb not in parent_map:
                            parent_map[nb] = u
                            queue.append(nb)
                # Find shortest path to another remaining-anchor
                for end_uid in sorted(parent_map.keys()):
                    if end_uid == start_uid:
                        continue
                    if end_uid in remaining_anchors:
                        path = []
                        cur: int | None = end_uid
                        while cur is not None:
                            path.append(cur)
                            cur = parent_map[cur]
                        path.reverse()
                        closure_chains.append((
                            remaining_anchors[start_uid],
                            remaining_anchors[end_uid],
                            path
                        ))
                        # Mark these as solved
                        solved_so_far.update(path)
                        remaining_units -= set(path)
                        found_chain = True
                        break
                if found_chain:
                    break

            if not found_chain:
                break  # no more chains possible

        # Determine common hinge axis (all linkage joints parallel)
        common_axis = np.array([0.0, -1.0, 0.0])
        for j in scene.joints:
            if j.joint_type != JointType.REVOLUTE:
                continue
            if j.unit_a_index == chassis_unit or j.unit_b_index == chassis_unit:
                ax = _ldraw_to_blender(j.axis)
                ax_n = float(np.linalg.norm(ax))
                if ax_n > 1e-12:
                    common_axis = ax / ax_n
                    break

        # --- Emit IK target empties (parented to cranks) ---
        emit("# ── IK targets (Empties parented to cranks) ──────────────────")
        emit()
        ik_targets: dict[int, str] = {}  # chain_idx → target_name
        for chain_idx, (anchor_uid, crank_uid, path) in enumerate(primary_chains):
            tip_uid = path[-1]
            target_pos = jpos(tip_uid, crank_uid)
            target_name = f"ik_target_{chain_idx}"
            ik_targets[chain_idx] = target_name
            emit(f"# IK target {chain_idx}: crank {crank_uid}, "
                 f"chain [{' → '.join(str(u) for u in path)}]")
            emit(f"bpy.ops.object.empty_add(type='PLAIN_AXES', "
                 f"location=({target_pos[0]:.6f}, "
                 f"{target_pos[1]:.6f}, {target_pos[2]:.6f}))")
            emit(f"_tgt = bpy.context.active_object")
            emit(f"_tgt.name = '{target_name}'")
            emit(f"_tgt.empty_display_size = 0.003")
            emit(f"_tgt.parent = _units[{crank_uid}]")
            emit(f"_tgt.matrix_parent_inverse = "
                 f"_units[{crank_uid}].matrix_world.inverted()")
            emit()

        # Closure chain targets: Empties at the tip position on the solved bone
        closure_targets: dict[int, str] = {}
        for ci, (solved_anchor, solved_target, path) in enumerate(closure_chains):
            tip_uid = path[-1]
            target_pos = jpos(tip_uid, solved_target)
            target_name = f"ik_closure_target_{ci}"
            closure_targets[ci] = target_name
            emit(f"# Closure target {ci}: path [{' → '.join(str(u) for u in path)}]")
            emit(f"bpy.ops.object.empty_add(type='PLAIN_AXES', "
                 f"location=({target_pos[0]:.6f}, "
                 f"{target_pos[1]:.6f}, {target_pos[2]:.6f}))")
            emit(f"_tgt = bpy.context.active_object")
            emit(f"_tgt.name = '{target_name}'")
            emit(f"_tgt.empty_display_size = 0.003")
            # Parent to the solved unit's bone (will be set after armature)
            emit(f"_tgt.parent = _units[{solved_target}]")
            emit(f"_tgt.matrix_parent_inverse = "
                 f"_units[{solved_target}].matrix_world.inverted()")
            emit()

        # --- Create armature ---
        emit("# ── Linkage armature ────────────────────────────────────────────")
        emit("bpy.ops.object.armature_add(enter_editmode=True, location=(0,0,0))")
        emit("_arm_obj = bpy.context.active_object")
        emit("_arm_obj.name = 'LinkageArmature'")
        emit("_arm = _arm_obj.data")
        emit("_arm.name = 'LinkageArmData'")
        emit("_arm.edit_bones.remove(_arm.edit_bones[0])")
        emit()

        # --- Create bones for primary chains ---
        # Each bone spans from one joint to the next along the path.
        # bone[i]: head = joint(path[i-1], path[i]), tail = joint(path[i], path[i+1])
        # First bone: head = anchor_pos, tail = joint(path[0], path[1])
        # Last bone: head = joint(path[-2], path[-1]), tail = driven_pos
        bone_names: dict[tuple, str] = {}  # (chain_idx, unit_idx) → bone_name
        unit_bone: dict[int, str] = {}  # uid → primary bone_name

        for chain_idx, (anchor_uid, crank_uid, path) in enumerate(primary_chains):
            anchor_pos = anchors[anchor_uid]
            driven_pos = jpos(path[-1], crank_uid)

            for i, uid in enumerate(path):
                bone_name = f"chain{chain_idx}_u{uid}"
                bone_names[(chain_idx, uid)] = bone_name
                if uid not in unit_bone:
                    unit_bone[uid] = bone_name

                # Compute head and tail
                if i == 0:
                    head = anchor_pos
                else:
                    head = jpos(path[i - 1], path[i])

                if i == len(path) - 1:
                    tail = driven_pos
                else:
                    tail = jpos(path[i], path[i + 1])

                # Check for zero-length bone
                bone_len = float(np.linalg.norm(tail - head))
                if bone_len < 1e-6:
                    # Offset tail slightly along common axis
                    tail = head + common_axis * 0.001

                emit(f"_b = _arm.edit_bones.new('{bone_name}')")
                emit(f"_b.head = ({head[0]:.6f}, {head[1]:.6f}, {head[2]:.6f})")
                emit(f"_b.tail = ({tail[0]:.6f}, {tail[1]:.6f}, {tail[2]:.6f})")
                if i > 0:
                    parent_bone = bone_names[(chain_idx, path[i - 1])]
                    emit(f"_b.parent = _arm.edit_bones['{parent_bone}']")
                    emit(f"_b.use_connect = True")
                emit()

        # --- Create bones for closure chains ---
        closure_bone_names: dict[tuple, str] = {}
        for ci, (solved_anchor, solved_target, path) in enumerate(closure_chains):
            anchor_pos = jpos(path[0], solved_anchor)
            target_pos = jpos(path[-1], solved_target)

            for i, uid in enumerate(path):
                bone_name = f"closure{ci}_u{uid}"
                closure_bone_names[(ci, uid)] = bone_name
                if uid not in unit_bone:
                    unit_bone[uid] = bone_name

                if i == 0:
                    head = anchor_pos
                else:
                    head = jpos(path[i - 1], path[i])

                if i == len(path) - 1:
                    tail = target_pos
                else:
                    tail = jpos(path[i], path[i + 1])

                bone_len = float(np.linalg.norm(tail - head))
                if bone_len < 1e-6:
                    tail = head + common_axis * 0.001

                emit(f"_b = _arm.edit_bones.new('{bone_name}')")
                emit(f"_b.head = ({head[0]:.6f}, {head[1]:.6f}, {head[2]:.6f})")
                emit(f"_b.tail = ({tail[0]:.6f}, {tail[1]:.6f}, {tail[2]:.6f})")
                if i > 0:
                    parent_bone = closure_bone_names[(ci, path[i - 1])]
                    emit(f"_b.parent = _arm.edit_bones['{parent_bone}']")
                    emit(f"_b.use_connect = True")
                elif solved_anchor in unit_bone:
                    # Parent root bone to the anchor's bone so it follows motion
                    emit(f"_b.parent = _arm.edit_bones['{unit_bone[solved_anchor]}']")
                    emit(f"_b.use_connect = False")
                emit()

        # --- Switch to pose mode and add IK constraints ---
        emit("# Switch to pose mode for constraints")
        emit("bpy.ops.object.mode_set(mode='OBJECT')")
        emit("bpy.context.view_layer.objects.active = _arm_obj")
        emit("bpy.ops.object.mode_set(mode='POSE')")
        emit()

        # Primary chain IK
        emit("# ── IK constraints on primary chains ─────────────────────────")
        for chain_idx, (anchor_uid, crank_uid, path) in enumerate(primary_chains):
            tip_bone = bone_names[(chain_idx, path[-1])]
            target_name = ik_targets[chain_idx]
            chain_count = len(path)

            emit(f"# Chain {chain_idx}: "
                 f"[{' → '.join(str(u) for u in path)}] → crank {crank_uid}")
            emit(f"_pb = _arm_obj.pose.bones['{tip_bone}']")
            emit(f"_ik = _pb.constraints.new('IK')")
            emit(f"_ik.target = bpy.data.objects['{target_name}']")
            emit(f"_ik.chain_count = {chain_count}")
            emit(f"_ik.use_stretch = False")
            emit()

        # Closure chain IK
        emit("# ── IK constraints on closure chains ─────────────────────────")
        for ci, (solved_anchor, solved_target, path) in enumerate(closure_chains):
            tip_bone = closure_bone_names[(ci, path[-1])]
            target_name = closure_targets[ci]
            chain_count = len(path)

            emit(f"# Closure {ci}: "
                 f"[{' → '.join(str(u) for u in path)}]")
            emit(f"_pb = _arm_obj.pose.bones['{tip_bone}']")
            emit(f"_ik = _pb.constraints.new('IK')")
            emit(f"_ik.target = bpy.data.objects['{target_name}']")
            emit(f"_ik.chain_count = {chain_count}")
            emit(f"_ik.use_stretch = False")
            emit()

        # --- Exit pose mode ---
        emit("bpy.ops.object.mode_set(mode='OBJECT')")
        emit()

        # Hide armature from render but keep in viewport
        emit("_arm_obj.hide_render = True")
        emit()

        # --- Parent armature and kinematic units to chassis when free ---
        if not anchor_motor:
            emit("# ── Parent armature & kinematic units to chassis ────────────")
            emit("# When chassis is ACTIVE (free), the armature and gear/crank")
            emit("# objects must follow it so the IK linkage stays attached.")
            emit(f"_arm_obj.parent = _units[{chassis_unit}]")
            emit(f"_arm_obj.matrix_parent_inverse = "
                 f"_units[{chassis_unit}].matrix_world.inverted()")
            emit()
            # Parent gear and crank units to chassis
            for uid in sorted(kinematic_units | output_units):
                emit(f"_units[{uid}].parent = _units[{chassis_unit}]")
                emit(f"_units[{uid}].matrix_parent_inverse = "
                     f"_units[{chassis_unit}].matrix_world.inverted()")
            emit()

        # --- Attach meshes to bones via CHILD_OF constraints ---
        emit("# ── Attach linkage meshes to armature bones ──────────────────")
        emit("# Use CHILD_OF constraints (avoids bone-parenting matrix issues).")
        emit("# The constraint's inverse matrix is set so meshes stay in place at")
        emit("# frame 0, then follow bone movement as IK solves.")
        for uid in sorted(linkage_units):
            if uid not in unit_bone:
                continue
            bone_name = unit_bone[uid]
            emit(f"_con = _units[{uid}].constraints.new('CHILD_OF')")
            emit(f"_con.target = _arm_obj")
            emit(f"_con.subtarget = '{bone_name}'")
            emit(f"_con.inverse_matrix = ("
                 f"_arm_obj.matrix_world @ "
                 f"_arm_obj.pose.bones['{bone_name}'].matrix).inverted() @ "
                 f"_units[{uid}].matrix_world")
            emit()

        emit()

    # ------------------------------------------------------------------
    # Gear mesh collision exclusion
    # ------------------------------------------------------------------
    if scene.gears:
        emit("# ── Gear mesh collision exclusion ───────────────────────────")
        emit("# Disable collision between meshing gears (their meshes are")
        emit("# decorative — power transfer is via kinematic animation +")
        emit("# torque-limited MOTOR at the output).")
        emit()
        for gidx, gc in enumerate(scene.gears):
            midpoint = _ldraw_to_blender(gc.position) * LDU_TO_METERS
            emit(f"# Gear mesh {gidx}: unit {gc.unit_a_index} ↔ unit {gc.unit_b_index}"
                 f" (ratio {gc.ratio:.3f})")
            emit(
                f"bpy.ops.object.empty_add("
                f"location=({midpoint[0]:.6f}, {midpoint[1]:.6f}, {midpoint[2]:.6f}))"
            )
            emit("_gc = bpy.context.active_object")
            emit(f"_gc.name = 'gear_mesh_{gidx}'")
            emit(f"bpy.ops.rigidbody.constraint_add(type='FIXED')")
            emit(f"_gc.rigid_body_constraint.object1 = _units[{gc.unit_a_index}]")
            emit(f"_gc.rigid_body_constraint.object2 = _units[{gc.unit_b_index}]")
            emit("_gc.rigid_body_constraint.disable_collisions = True")
            emit("_gc.rigid_body_constraint.enabled = False")
            emit("_gc.rigid_body_constraint.use_breaking = False")
            emit()

    # ------------------------------------------------------------------
    # Direct motor (fallback when no drive tree / no gears)
    # ------------------------------------------------------------------
    if scene.motors and not drive_tree:
        emit("# ── Direct Motor ─────────────────────────────────────────────")
        emit()
        for midx, motor in enumerate(scene.motors):
            joint = scene.joints[motor.joint_index]
            pos_bl = _ldraw_to_blender(joint.position)
            axis_bl = _ldraw_to_blender(joint.axis)
            axis_norm = float(np.linalg.norm(axis_bl))
            if axis_norm > 1e-12:
                axis_bl = axis_bl / axis_norm
            max_impulse = motor.max_torque / fps
            emit(f"# Motor {midx}: drives joint {motor.joint_index}")
            emit(
                f"bpy.ops.object.empty_add("
                f"location=({pos_bl[0]:.6f}, {pos_bl[1]:.6f}, {pos_bl[2]:.6f}))"
            )
            emit("_mot = bpy.context.active_object")
            emit(f"_mot.name = 'motor_{midx}'")
            emit(
                f"_drive_axis = mathutils.Vector("
                f"({axis_bl[0]:.6f}, {axis_bl[1]:.6f}, {axis_bl[2]:.6f}))"
            )
            emit("_x_axis = mathutils.Vector((1.0, 0.0, 0.0))")
            emit("_mot_rot = _x_axis.rotation_difference(_drive_axis)")
            emit("_mot.rotation_mode = 'QUATERNION'")
            emit("_mot.rotation_quaternion = _mot_rot")
            emit("bpy.ops.rigidbody.constraint_add(type='MOTOR')")
            emit(f"_mot.rigid_body_constraint.object1 = _units[{joint.unit_a_index}]")
            emit(f"_mot.rigid_body_constraint.object2 = _units[{joint.unit_b_index}]")
            emit("_mot.rigid_body_constraint.use_motor_ang = True")
            emit(f"_mot.rigid_body_constraint.motor_ang_target_velocity = {motor.speed:.6f}")
            emit(f"_mot.rigid_body_constraint.motor_ang_max_impulse = {max_impulse:.6f}")
            emit()

    # ------------------------------------------------------------------
    # Bake and render
    # ------------------------------------------------------------------
    emit("# ── Finalise ─────────────────────────────────────────────────")
    emit("bpy.context.view_layer.update()")
    emit("# Bake rigid body simulation")
    emit("override = bpy.context.copy()")
    emit("override['point_cache'] = scene.rigidbody_world.point_cache")
    emit("with bpy.context.temp_override(**override):")
    emit("    bpy.ops.ptcache.bake(bake=True)")
    emit("print('Physics bake complete.')")
    emit()

    # Post-bake diagnostics: log rotation of output/active units per frame
    emit("# ── Diagnostics ────────────────────────────────────────────────")
    emit("import math")
    emit("print('\\n=== POST-BAKE DIAGNOSTICS ===')")
    emit(f"print(f'Frames: 1..{sim_frames}')")
    # Track output units (cranks) and sample linkage units
    diag_units = list(output_units)[:2]
    # Add grounded linkage units as diagnostic targets
    if linkage_units:
        grounded_diag = sorted(uid for uid in linkage_units
                               if any(j.unit_a_index == chassis_unit and j.unit_b_index == uid
                                      or j.unit_b_index == chassis_unit and j.unit_a_index == uid
                                      for j in scene.joints if j.joint_type == JointType.REVOLUTE))
        diag_units.extend(grounded_diag[:2])
    emit(f"_diag_units = {diag_units}")
    emit("for _du in _diag_units:")
    emit(f"    print(f'  Unit {{_du}}: {{_units[_du].name}}')")
    emit("print()")
    emit("print('frame | ' + ' | '.join(f'unit{u}_rot(deg)' for u in _diag_units))")
    emit("print('-' * 80)")
    emit(f"for _f in range(1, {sim_frames} + 1, max(1, {sim_frames} // 12)):")
    emit("    scene.frame_set(_f)")
    emit("    _vals = []")
    emit("    for _du in _diag_units:")
    emit("        _obj = _units[_du]")
    emit("        _e = _obj.matrix_world.to_euler()")
    emit("        _total = math.degrees(math.sqrt(_e.x**2 + _e.y**2 + _e.z**2))")
    emit("        _vals.append(f'{_total:8.2f}')")
    emit("    print(f'{_f:5d} | ' + ' | '.join(_vals))")
    emit("print('=== END DIAGNOSTICS ===\\n')")
    emit()

    # ------------------------------------------------------------------
    # Lighting check – renders a tiny test frame and adjusts light energy.
    # ------------------------------------------------------------------
    if _runtime_geometry and not gltf_export:
        emit_lighting_check(emit)

    # ------------------------------------------------------------------
    # Framing check – projects all unit bounding boxes into camera space
    # across every frame and auto-adjusts camera if content exceeds frame.
    # ------------------------------------------------------------------
    if not gltf_export:
        emit()
        emit_framing_check(emit, objects_var="_units", margin=0.03)
        emit()

    if render:
        emit()
        emit("# ── Render ─────────────────────────────────────────────────")
        emit("print(f'Rendering {scene.frame_end} frames...')")
        emit("bpy.ops.render.render(animation=True)")
        emit("print('Simulation render complete.')")
    elif not gltf_export:
        emit("print('LegoTechnicSimulation: scene ready.')")

    # ------------------------------------------------------------------
    # glTF export – bake all object transforms and export animated .glb
    # ------------------------------------------------------------------
    if gltf_export:
        emit()
        emit("# ── glTF Export ────────────────────────────────────────────")
        emit("print('Baking animation for glTF export...')")
        emit("# Collect all unit mesh objects")
        emit("_export_objects = [obj for obj in _units if obj is not None]")
        emit("# Also collect ground plane if present")
        emit("if '_ground' in dir() and _ground is not None:")
        emit("    _export_objects.append(_ground)")
        emit()
        emit("# Select only export objects")
        emit("bpy.ops.object.select_all(action='DESELECT')")
        emit("for obj in _export_objects:")
        emit("    obj.select_set(True)")
        emit()
        emit("# Bake visual transforms to keyframes for all frames")
        emit("bpy.context.view_layer.objects.active = _export_objects[0]")
        emit("bpy.ops.nla.bake(")
        emit("    frame_start=scene.frame_start,")
        emit("    frame_end=scene.frame_end,")
        emit("    only_selected=True,")
        emit("    visual_keying=True,")
        emit("    clear_constraints=True,")
        emit("    clear_parents=True,")
        emit("    use_current_action=True,")
        emit("    bake_types={'OBJECT'},")
        emit(")")
        emit("print('Keyframe bake complete.')")
        emit()
        emit("# Remove rigid body world (not needed in glTF)")
        emit("if scene.rigidbody_world:")
        emit("    bpy.ops.rigidbody.world_remove()")
        emit()
        emit("# Remove armatures and non-mesh objects from export set")
        emit("bpy.ops.object.select_all(action='DESELECT')")
        emit("for obj in _export_objects:")
        emit("    if obj.type == 'MESH':")
        emit("        obj.select_set(True)")
        emit()
        emit("# Export as glTF")
        gltf_path = str(gltf_export)
        emit(f"_gltf_path = r'{gltf_path}'")
        emit("bpy.ops.export_scene.gltf(")
        emit("    filepath=_gltf_path,")
        emit("    use_selection=True,")
        emit("    export_format='GLB' if _gltf_path.endswith('.glb') else 'GLTF_SEPARATE',")
        emit("    export_animations=True,")
        emit("    export_frame_range=True,")
        emit("    export_nla_strips=False,")
        emit("    export_current_frame=False,")
        emit("    export_apply=True,")
        emit(")")
        emit(f"print(f'glTF exported to: {{_gltf_path}}')")

    script = "\n".join(lines)

    if output_path is not None:
        Path(output_path).write_text(script, encoding="utf-8")

    return script
