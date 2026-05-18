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
    emit_kinematic_rotation,
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
    model_path: Optional[Path] = None,
    ldraw_library: Optional[Path] = None,
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
        emit("from lego_technic_sim.blender.geometry import collect_geometry")
        emit()
        emit(f"print('Parsing LDraw model...')")
        emit(f"_build = LDrawParser({str(ldraw_library)!r}).parse_build({str(model_path)!r})")
        emit("_scene = build_units_and_joints(_build)")
        emit(f"print(f'Loaded {{len(_scene.units)}} units')")
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
        emit("_sun.data.energy = 3.0")
        emit("world = bpy.data.worlds.get('World') or bpy.data.worlds.new('World')")
        emit("scene.world = world")
        emit("world.use_nodes = True")
        emit("bg = world.node_tree.nodes.get('Background')")
        emit("if bg:")
        emit("    bg.inputs[0].default_value = (0.05, 0.05, 0.08, 1.0)")
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

    # Chassis/motor units are always PASSIVE static
    _motor_unit_indices = set()
    if anchor_motor:
        for motor in scene.motors:
            joint = scene.joints[motor.joint_index]
            _motor_unit_indices.add(joint.unit_a_index)
    # Always anchor the motor body in kinematic mode
    if scene.motors:
        motor_joint = scene.joints[scene.motors[0].joint_index]
        chassis_unit = motor_joint.unit_a_index
    _motor_unit_indices.add(chassis_unit)

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
                emit(f"_verts, _faces = collect_geometry(_scene.units[{idx}].bricks)")
                emit("if _verts:")
                emit(f"    _mesh = bpy.data.meshes.new({safe_name + '_mesh'!r})")
                emit("    _mesh.from_pydata(_verts, [], _faces)")
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
        # Kinematic units: motor body (static) and internal gears (animated)
        if idx in _motor_unit_indices or idx in kinematic_units:
            emit("_obj.rigid_body.type = 'PASSIVE'")
            emit("_obj.rigid_body.kinematic = True")
        else:
            emit("_obj.rigid_body.type = 'ACTIVE'")
        emit("_obj.rigid_body.collision_shape = 'CONVEX_HULL'")
        emit("_obj.rigid_body.friction = 0.5")
        emit("_obj.rigid_body.restitution = 0.0")
        emit("_obj.rigid_body.linear_damping = 0.04")
        emit("_obj.rigid_body.angular_damping = 0.1")

        # Material
        if render:
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
    if render:
        emit("_gmat = bpy.data.materials.new(name='ground_mat')")
        emit("_gmat.use_nodes = True")
        emit("_gbsdf = _gmat.node_tree.nodes.get('Principled BSDF')")
        emit("if _gbsdf:")
        emit("    _gbsdf.inputs['Base Color'].default_value = (0.3, 0.3, 0.3, 1.0)")
        emit("_ground.data.materials.append(_gmat)")
    emit()

    # ------------------------------------------------------------------
    # Joints → rigid body constraints
    # ------------------------------------------------------------------
    # Skip joints between two kinematic/chassis units (no physics needed).
    # Convert joints from chassis to output units into MOTOR constraints.
    emit("# ── Joints (constraints) ─────────────────────────────────────")
    emit("_joints = []")
    emit()

    passive_set = _motor_unit_indices | kinematic_units

    for idx, joint in enumerate(scene.joints):
        ua, ub = joint.unit_a_index, joint.unit_b_index
        # Skip joints where both sides are kinematic/chassis
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

        # Check if this is an output joint (chassis → output crank)
        is_output_joint = (
            (ua in passive_set and ub in output_units) or
            (ub in passive_set and ua in output_units)
        )

        if is_output_joint and joint.joint_type == JointType.REVOLUTE:
            # Torque-limited MOTOR constraint at gear→leg interface
            # We need BOTH a HINGE (positional constraint) and a MOTOR (drive).
            output_uid = ub if ub in output_units else ua
            node = unit_to_node.get(output_uid)
            if node and scene.motors:
                motor_spec = scene.motors[0]
                # Output torque = stall_torque / accumulated_ratio
                # (gear reduction amplifies torque)
                output_torque = motor_spec.max_torque / abs(node.accumulated_ratio)
                output_impulse = output_torque / fps
                output_speed = motor_spec.speed * node.accumulated_ratio
            else:
                output_impulse = 0.01
                output_speed = 1.0

            emit(
                f"# Joint {idx}: OUTPUT HINGE+MOTOR "
                f"(unit {ua} ↔ unit {ub}, torque-limited)"
            )
            # First: HINGE to hold position
            emit(
                f"bpy.ops.object.empty_add("
                f"location=({pos_bl[0]:.6f}, {pos_bl[1]:.6f}, {pos_bl[2]:.6f}))"
            )
            emit("_con = bpy.context.active_object")
            emit(f"_con.name = 'joint_{idx}'")
            emit(
                f"_axis = mathutils.Vector("
                f"({axis_bl[0]:.6f}, {axis_bl[1]:.6f}, {axis_bl[2]:.6f}))"
            )
            emit("_up = mathutils.Vector((0.0, 0.0, 1.0))")
            emit("_rot = _up.rotation_difference(_axis)")
            emit("_con.rotation_mode = 'QUATERNION'")
            emit("_con.rotation_quaternion = _rot")
            emit("bpy.ops.rigidbody.constraint_add(type='HINGE')")
            emit(f"_con.rigid_body_constraint.object1 = _units[{ua}]")
            emit(f"_con.rigid_body_constraint.object2 = _units[{ub}]")
            emit("_con.rigid_body_constraint.disable_collisions = True")
            emit("_con.rigid_body_constraint.use_breaking = False")
            emit()
            # Second: MOTOR to drive rotation
            emit(
                f"bpy.ops.object.empty_add("
                f"location=({pos_bl[0]:.6f}, {pos_bl[1]:.6f}, {pos_bl[2]:.6f}))"
            )
            emit("_mot = bpy.context.active_object")
            emit(f"_mot.name = 'output_motor_{idx}'")
            emit("_x_axis = mathutils.Vector((1.0, 0.0, 0.0))")
            emit("_om_rot = _x_axis.rotation_difference(_axis)")
            emit("_mot.rotation_mode = 'QUATERNION'")
            emit("_mot.rotation_quaternion = _om_rot")
            emit("bpy.ops.rigidbody.constraint_add(type='MOTOR')")
            emit(f"_mot.rigid_body_constraint.object1 = _units[{ua}]")
            emit(f"_mot.rigid_body_constraint.object2 = _units[{ub}]")
            emit("_mot.rigid_body_constraint.use_motor_ang = True")
            emit(f"_mot.rigid_body_constraint.motor_ang_target_velocity = {abs(output_speed):.6f}")
            emit(f"_mot.rigid_body_constraint.motor_ang_max_impulse = {output_impulse:.6f}")
            emit("_mot.rigid_body_constraint.disable_collisions = True")
            emit("_mot.rigid_body_constraint.use_breaking = False")
        else:
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

    if render:
        emit()
        emit("# ── Render ─────────────────────────────────────────────────")
        emit("print(f'Rendering {scene.frame_end} frames...')")
        emit("bpy.ops.render.render(animation=True)")
        emit("print('Simulation render complete.')")
    else:
        emit("print('LegoTechnicSimulation: scene ready.')")

    script = "\n".join(lines)

    if output_path is not None:
        Path(output_path).write_text(script, encoding="utf-8")

    return script
