"""Generate a Blender Python script that sets up a rigid-body physics scene.

The generated script is self-contained and can be executed inside Blender's
*Scripting* workspace (or via ``blender --background --python simulation.py``).
It will:

1. Delete any existing scene objects.
2. Enable the Blender Rigid Body world.
3. Create a proxy mesh object for every :class:`~lego_technic_sim.physics.model.Unit`.
4. Add Rigid Body Constraints (Empty objects) for every
   :class:`~lego_technic_sim.physics.model.Joint`.
5. Configure angular-motor parameters for every
   :class:`~lego_technic_sim.physics.model.Motor`.

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

from ..physics.model import Joint, JointType, Motor, PhysicsScene, Unit


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
    emit()

    # ------------------------------------------------------------------
    # Scene setup
    # ------------------------------------------------------------------
    emit("# ── Scene setup ──────────────────────────────────────────────")
    emit("bpy.ops.object.select_all(action='SELECT')")
    emit("bpy.ops.object.delete(use_global=False)")
    emit()
    emit("scene = bpy.context.scene")
    emit(f"scene.render.fps = {fps}")
    emit("if scene.rigidbody_world:")
    emit("    bpy.ops.rigidbody.world_remove()")
    emit("bpy.ops.rigidbody.world_add()")
    emit("scene.rigidbody_world.time_scale = 1.0")
    emit(
        f"scene.gravity = ({gravity[0]:.6f}, {gravity[1]:.6f}, {gravity[2]:.6f})"
    )
    emit()

    # ------------------------------------------------------------------
    # Units → rigid bodies
    # ------------------------------------------------------------------
    emit("# ── Units (rigid bodies) ─────────────────────────────────────")
    emit("_units = []")
    emit()

    for idx, unit in enumerate(scene.units):
        com_bl = _ldraw_to_blender(unit.center_of_mass)
        safe_name = unit.name.replace('"', "")
        emit(f"# Unit {idx}: {safe_name}")
        emit(
            f"bpy.ops.mesh.primitive_cube_add("
            f"size=0.02, "
            f"location=({com_bl[0]:.6f}, {com_bl[1]:.6f}, {com_bl[2]:.6f}))"
        )
        emit("_obj = bpy.context.active_object")
        emit(f"_obj.name = {safe_name!r}")
        emit("bpy.ops.rigidbody.object_add()")
        emit(f"_obj.rigid_body.mass = {unit.mass:.6f}")
        emit("_obj.rigid_body.type = 'ACTIVE'")
        emit("_obj.rigid_body.collision_shape = 'CONVEX_HULL'")
        emit("_units.append(_obj)")
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
    # Motors
    # ------------------------------------------------------------------
    if scene.motors:
        emit("# ── Motors ───────────────────────────────────────────────")
        emit()
        for midx, motor in enumerate(scene.motors):
            emit(f"# Motor {midx}: drives joint {motor.joint_index}")
            emit(f"_rbc = _joints[{motor.joint_index}].rigid_body_constraint")
            emit("_rbc.use_motor_ang = True")
            emit(f"_rbc.motor_ang_target_velocity = {motor.speed:.6f}")
            emit(f"_rbc.motor_ang_max_impulse = {motor.max_torque:.6f}")
            emit()

    # ------------------------------------------------------------------
    # Final viewport update
    # ------------------------------------------------------------------
    emit("# ── Finalise ─────────────────────────────────────────────────")
    emit("bpy.context.view_layer.update()")
    emit("print('LegoTechnicSimulation: scene ready.')")

    script = "\n".join(lines)

    if output_path is not None:
        Path(output_path).write_text(script, encoding="utf-8")

    return script
