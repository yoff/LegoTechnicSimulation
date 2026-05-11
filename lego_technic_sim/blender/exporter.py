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


def _ldraw_to_blender(v: np.ndarray) -> np.ndarray:
    """Convert a 3-D point from LDraw space to Blender space.

    LDraw: X right, Y down, Z toward viewer.
    Blender: X right, Y toward viewer, Z up.
    """
    return np.array([v[0], -v[2], -v[1]], dtype=float)


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

    Returns:
        The generated Python script as a string.
    """
    if gravity is None:
        gravity = np.array([0.0, 0.0, -9.81])

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
    # Units → rigid bodies
    # ------------------------------------------------------------------
    # Identify motor units (optionally PASSIVE to anchor the mechanism)
    _motor_unit_indices = set()
    if anchor_motor:
        for motor in scene.motors:
            joint = scene.joints[motor.joint_index]
            _motor_unit_indices.add(joint.unit_a_index)

    emit("# ── Units (rigid bodies) ─────────────────────────────────────")
    emit(f"_n_units = {len(scene.units)}")
    emit("_units = []")
    emit()

    for idx, unit in enumerate(scene.units):
        com_bl = _ldraw_to_blender(unit.center_of_mass)
        safe_name = unit.name.replace('"', "")
        emit(f"# Unit {idx}: {safe_name}")

        if use_mesh:
            # Create real mesh from triangles
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
        # Motor units are PASSIVE (anchored) so they drive without flying away
        if idx in _motor_unit_indices:
            emit("_obj.rigid_body.type = 'PASSIVE'")
        else:
            emit("_obj.rigid_body.type = 'ACTIVE'")
        emit("_obj.rigid_body.collision_shape = 'CONVEX_HULL'")
        emit("_obj.rigid_body.friction = 0.5")
        emit("_obj.rigid_body.restitution = 0.2")
        emit("_obj.rigid_body.linear_damping = 0.1")
        emit("_obj.rigid_body.angular_damping = 0.3")

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
    emit("# ── Joints (constraints) ─────────────────────────────────────")
    emit("_joints = []")
    emit()

    for idx, joint in enumerate(scene.joints):
        pos_bl = _ldraw_to_blender(joint.position)
        axis_bl = _ldraw_to_blender(joint.axis)
        axis_norm = float(np.linalg.norm(axis_bl))
        if axis_norm > 1e-12:
            axis_bl = axis_bl / axis_norm

        blender_type = {
            JointType.REVOLUTE: "HINGE",
            JointType.FIXED: "FIXED",
            JointType.SLIDER: "SLIDER",
        }[joint.joint_type]

        emit(
            f"# Joint {idx}: {joint.joint_type.name} "
            f"(unit {joint.unit_a_index} ↔ unit {joint.unit_b_index})"
        )
        emit(
            f"bpy.ops.object.empty_add("
            f"location=({pos_bl[0]:.6f}, {pos_bl[1]:.6f}, {pos_bl[2]:.6f}))"
        )
        emit("_con = bpy.context.active_object")
        emit(f"_con.name = 'joint_{idx}'")
        emit(f"bpy.ops.rigidbody.constraint_add(type={blender_type!r})")
        emit(f"_con.rigid_body_constraint.object1 = _units[{joint.unit_a_index}]")
        emit(f"_con.rigid_body_constraint.object2 = _units[{joint.unit_b_index}]")
        emit("_con.rigid_body_constraint.disable_collisions = True")

        if joint.joint_type == JointType.REVOLUTE:
            # Orient the constraint empty so its local Z aligns with the hinge axis
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
    # Motors — use dedicated MOTOR constraint (drives around local X axis)
    # ------------------------------------------------------------------
    if scene.motors:
        emit("# ── Motors ───────────────────────────────────────────────")
        emit()
        for midx, motor in enumerate(scene.motors):
            joint = scene.joints[motor.joint_index]
            pos_bl = _ldraw_to_blender(joint.position)
            axis_bl = _ldraw_to_blender(joint.axis)
            axis_norm = float(np.linalg.norm(axis_bl))
            if axis_norm > 1e-12:
                axis_bl = axis_bl / axis_norm

            max_impulse = motor.max_torque * 100.0
            emit(f"# Motor {midx}: drives joint {motor.joint_index}")
            emit(f"#   axis = ({axis_bl[0]:.3f}, {axis_bl[1]:.3f}, {axis_bl[2]:.3f})")
            # Create a MOTOR constraint empty oriented so its X axis = hinge axis
            emit(
                f"bpy.ops.object.empty_add("
                f"location=({pos_bl[0]:.6f}, {pos_bl[1]:.6f}, {pos_bl[2]:.6f}))"
            )
            emit("_mot = bpy.context.active_object")
            emit(f"_mot.name = 'motor_{midx}'")
            # Orient: local X must align with the rotation axis
            emit(
                f"_drive_axis = mathutils.Vector("
                f"({axis_bl[0]:.6f}, {axis_bl[1]:.6f}, {axis_bl[2]:.6f}))"
            )
            emit("_x_axis = mathutils.Vector((1.0, 0.0, 0.0))")
            emit("_mot_rot = _x_axis.rotation_difference(_drive_axis)")
            emit("_mot.rotation_mode = 'QUATERNION'")
            emit("_mot.rotation_quaternion = _mot_rot")
            emit(f"bpy.ops.rigidbody.constraint_add(type='MOTOR')")
            emit(f"_mot.rigid_body_constraint.object1 = _units[{joint.unit_a_index}]")
            emit(f"_mot.rigid_body_constraint.object2 = _units[{joint.unit_b_index}]")
            emit("_mot.rigid_body_constraint.use_motor_ang = True")
            emit(f"_mot.rigid_body_constraint.motor_ang_target_velocity = {motor.speed:.6f}")
            emit(f"_mot.rigid_body_constraint.motor_ang_max_impulse = {max_impulse:.6f}")
            emit()

    # ------------------------------------------------------------------
    # Gear meshes — disable collisions and build kinematic coupling
    # ------------------------------------------------------------------
    # Build drive tree to determine which gear drives which
    drive_tree = build_drive_train(scene)
    # Map: driven_unit_index → (driver_unit_index, ratio, driver_axis_bl, driven_axis_bl, hinge_pos_bl)
    gear_couplings: list[tuple[int, int, float, np.ndarray, np.ndarray, np.ndarray]] = []

    if drive_tree and scene.gears:
        # Build parent map from drive tree
        parent_map: dict[int, int] = {}
        for node in drive_tree.all_nodes:
            for child in node.children:
                parent_map[child.unit_index] = node.unit_index

        # Build map: (unit_a, unit_b) → joint for finding hinge positions
        joint_map: dict[tuple[int, int], Joint] = {}
        for j in scene.joints:
            joint_map[(j.unit_a_index, j.unit_b_index)] = j
            joint_map[(j.unit_b_index, j.unit_a_index)] = j

        for gc in scene.gears:
            # Determine which unit is the driver (closer to motor)
            if gc.unit_a_index in parent_map and parent_map[gc.unit_a_index] == gc.unit_b_index:
                # B drives A
                driver_idx, driven_idx = gc.unit_b_index, gc.unit_a_index
                driver_axis = _ldraw_to_blender(gc.axis_b)
                driven_axis = _ldraw_to_blender(gc.axis_a)
                ratio = 1.0 / gc.ratio  # invert: A is driven
            elif gc.unit_b_index in parent_map and parent_map[gc.unit_b_index] == gc.unit_a_index:
                # A drives B
                driver_idx, driven_idx = gc.unit_a_index, gc.unit_b_index
                driver_axis = _ldraw_to_blender(gc.axis_a)
                driven_axis = _ldraw_to_blender(gc.axis_b)
                ratio = gc.ratio
            else:
                continue  # Not in drive tree — skip coupling

            # Find the hinge joint for the driven gear (connects it to the frame)
            hinge_pos = None
            for j in scene.joints:
                if j.joint_type == JointType.REVOLUTE:
                    if driven_idx in (j.unit_a_index, j.unit_b_index):
                        hinge_pos = _ldraw_to_blender(j.position)
                        break
            if hinge_pos is None:
                # Fallback to gear mesh position
                hinge_pos = _ldraw_to_blender(gc.position) * LDU_TO_METERS

            # Normalise axes
            dn = float(np.linalg.norm(driver_axis))
            if dn > 1e-12:
                driver_axis = driver_axis / dn
            dn2 = float(np.linalg.norm(driven_axis))
            if dn2 > 1e-12:
                driven_axis = driven_axis / dn2

            # For meshing spur gears the driven gear spins opposite
            # (teeth push in reverse). Check if axes are parallel vs anti-parallel.
            dot = float(np.dot(driver_axis, driven_axis))
            if dot > 0:
                # Same direction axes → counter-rotate
                sign = -1.0
            else:
                # Anti-parallel axes → same rotation sign
                sign = 1.0

            gear_couplings.append(
                (driver_idx, driven_idx, sign * ratio, driver_axis, driven_axis, hinge_pos)
            )

    if scene.gears:
        emit("# ── Gear meshes (collision exclusion) ─────────────────────")
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
            emit()

    if gear_couplings:
        emit("# ── Kinematic gear coupling ────────────────────────────────")
        emit("# Driven gears are set to kinematic; after the physics bake,")
        emit("# their rotation is overwritten based on the driving gear's")
        emit("# baked rotation scaled by the gear ratio.")
        emit()
        # Mark driven gears as kinematic
        driven_indices = {c[1] for c in gear_couplings}
        for di in driven_indices:
            emit(f"_units[{di}].rigid_body.kinematic = True")
        emit()

        # Build the coupling data structure in the generated script
        emit("_gear_couplings = [")
        for driver_idx, driven_idx, ratio, driver_axis, driven_axis, hinge_pos in gear_couplings:
            emit(f"    ({driver_idx}, {driven_idx}, {ratio:.6f},")
            emit(f"     mathutils.Vector(({driver_axis[0]:.6f}, {driver_axis[1]:.6f}, {driver_axis[2]:.6f})),")
            emit(f"     mathutils.Vector(({driven_axis[0]:.6f}, {driven_axis[1]:.6f}, {driven_axis[2]:.6f})),")
            emit(f"     mathutils.Vector(({hinge_pos[0]:.6f}, {hinge_pos[1]:.6f}, {hinge_pos[2]:.6f}))),")
        emit("]")
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

    if gear_couplings:
        emit()
        emit("# ── Keyframe driven gears from baked physics ────────────────")
        emit("print('Applying gear couplings...')")
        emit("for _f in range(scene.frame_start, scene.frame_end + 1):")
        emit("    scene.frame_set(_f)")
        emit("    for driver_idx, driven_idx, ratio, driver_axis, driven_axis, pivot in _gear_couplings:")
        emit("        _drv = _units[driver_idx]")
        emit("        _drvn = _units[driven_idx]")
        emit("        # Read baked rotation of driving gear")
        emit("        _drv_euler = _drv.matrix_world.to_euler('XYZ')")
        emit("        _drv_rot = mathutils.Vector(_drv_euler).dot(driver_axis)")
        emit("        # Compute driven rotation around hinge axis")
        emit("        _driven_rot = _drv_rot * ratio")
        emit("        _rot_mat = mathutils.Matrix.Rotation(_driven_rot, 4, driven_axis)")
        emit("        _drvn.matrix_world = mathutils.Matrix.Translation(pivot) @ _rot_mat @ mathutils.Matrix.Translation(-pivot)")
        emit("        _drvn.keyframe_insert(data_path='location', frame=_f)")
        emit("        _drvn.keyframe_insert(data_path='rotation_euler', frame=_f)")
        emit("print('Gear coupling complete.')")

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
