"""Export a PhysicsScene to MuJoCo MJCF XML format.

MuJoCo supports closed kinematic loops via equality constraints, making it
suitable for simulating Technic walking mechanisms that Blender's Bullet
engine cannot handle.

The export strategy:
1. Build a spanning tree of bodies connected by joints (hinge for revolute,
   weld for fixed).
2. Joints that would close a loop are instead expressed as equality
   `connect` constraints (point-to-point at the joint position).
3. Gear meshes are expressed as equality `joint` constraints coupling
   rotation between two hinge joints at the appropriate ratio.
4. Motors become `<actuator><motor>` elements on their joint.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional
from collections import deque
from xml.etree.ElementTree import Element, SubElement, tostring, indent

import numpy as np

from lego_technic_sim.physics.model import (
    GearConstraint,
    Joint,
    JointType,
    Motor,
    PhysicsScene,
    Unit,
)
from lego_technic_sim.physics.mesh_properties import LDU_TO_METERS


def _ldraw_to_mujoco(pos: np.ndarray) -> np.ndarray:
    """Convert LDraw coordinate (in metres) to MuJoCo (Z-up, metres).

    LDraw: X right, Y down, Z towards viewer.
    MuJoCo: X right, Y into screen, Z up.
    """
    # LDraw Y is down → MuJoCo Z is up → Z_mj = -Y_ld
    # LDraw Z is towards viewer → MuJoCo Y is into screen → Y_mj = -Z_ld
    return np.array([pos[0], -pos[2], -pos[1]])


def _write_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    """Write a binary STL file from vertices and face indices."""
    import struct

    n_faces = len(faces)
    with open(path, 'wb') as f:
        # 80-byte header
        f.write(b'\x00' * 80)
        # Number of triangles
        f.write(struct.pack('<I', n_faces))
        for face in faces:
            v0 = vertices[face[0]]
            v1 = vertices[face[1]]
            v2 = vertices[face[2]]
            # Normal (cross product)
            edge1 = v1 - v0
            edge2 = v2 - v0
            normal = np.cross(edge1, edge2)
            norm_len = np.linalg.norm(normal)
            if norm_len > 0:
                normal = normal / norm_len
            # Write: normal, v0, v1, v2, attribute byte count
            f.write(struct.pack('<3f', *normal))
            f.write(struct.pack('<3f', *v0))
            f.write(struct.pack('<3f', *v1))
            f.write(struct.pack('<3f', *v2))
            f.write(struct.pack('<H', 0))


def _vec_str(v: np.ndarray) -> str:
    """Format a vector as space-separated string."""
    return f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}"


def _inertia_sphere_approx(mass: float, radius: float) -> np.ndarray:
    """Approximate inertia as solid sphere (diagonal)."""
    I = 0.4 * mass * radius * radius
    return np.array([I, I, I])


def generate_mjcf(
    scene: PhysicsScene,
    *,
    timestep: float = 0.001,
    motor_speed: Optional[float] = None,
    mesh_dir: Optional[Path] = None,
) -> str:
    """Generate MJCF XML string from a PhysicsScene.

    Parameters
    ----------
    scene : PhysicsScene
        The physics scene with units, joints, motors, gears.
    timestep : float
        Simulation timestep in seconds.
    motor_speed : float, optional
        Override motor target speed (rad/s). If None, uses scene motor speed.
    mesh_dir : Path, optional
        Directory to write STL mesh files for each unit. If provided,
        uses actual mesh geometry instead of box approximations.

    Returns
    -------
    str
        MJCF XML document as a string.
    """
    units = scene.units
    joints = scene.joints

    # --- Build spanning tree via BFS from chassis (unit 0) ---
    # Determine chassis
    chassis_unit = 0
    if scene.motors:
        motor_joint = joints[scene.motors[0].joint_index]
        chassis_unit = motor_joint.unit_a_index

    # Adjacency: unit → [(neighbor_unit, joint_index)]
    adj: dict[int, list[tuple[int, int]]] = {i: [] for i in range(len(units))}
    for ji, j in enumerate(joints):
        adj[j.unit_a_index].append((j.unit_b_index, ji))
        adj[j.unit_b_index].append((j.unit_a_index, ji))

    # BFS spanning tree
    parent: dict[int, Optional[int]] = {chassis_unit: None}
    parent_joint: dict[int, int] = {}  # unit → joint index connecting to parent
    queue: deque[int] = deque([chassis_unit])
    tree_joints: set[int] = set()  # joint indices in the spanning tree
    while queue:
        u = queue.popleft()
        for nb, ji in adj[u]:
            if nb not in parent:
                parent[nb] = u
                parent_joint[nb] = ji
                tree_joints.add(ji)
                queue.append(nb)

    # Joints not in spanning tree → equality constraints (loop closures)
    loop_joints = [ji for ji in range(len(joints)) if ji not in tree_joints]

    # --- Write mesh files and compute per-unit geometry ---
    use_mesh = mesh_dir is not None
    unit_mesh_files: dict[int, str] = {}  # uid → STL filename
    unit_half_ext: dict[int, np.ndarray] = {}  # uid → box half-extents (fallback)

    for uid, unit in enumerate(units):
        all_verts = []
        all_faces = []
        vert_offset = 0
        for brick in unit.bricks:
            for tri in brick.triangles:
                v0 = np.array(tri.v0) * LDU_TO_METERS
                v1 = np.array(tri.v1) * LDU_TO_METERS
                v2 = np.array(tri.v2) * LDU_TO_METERS
                # Convert to MuJoCo coords (X, -Z, -Y)
                all_verts.append([v0[0], -v0[2], -v0[1]])
                all_verts.append([v1[0], -v1[2], -v1[1]])
                all_verts.append([v2[0], -v2[2], -v2[1]])
                all_faces.append([vert_offset, vert_offset + 1, vert_offset + 2])
                vert_offset += 3

        if not all_verts:
            unit_half_ext[uid] = np.array([0.005, 0.005, 0.005])
            continue

        verts = np.array(all_verts)
        # Shift to body-local frame (relative to CoM)
        com_mj = _ldraw_to_mujoco(unit.center_of_mass)
        verts_local = verts - com_mj

        half_ext = (verts_local.max(axis=0) - verts_local.min(axis=0)) / 2.0
        half_ext = np.maximum(half_ext, 0.001)
        unit_half_ext[uid] = half_ext

        if use_mesh:
            # Write binary STL
            faces = np.array(all_faces)
            stl_name = f"unit_{uid}.stl"
            unit_mesh_files[uid] = stl_name
            _write_stl(mesh_dir / stl_name, verts_local, faces)

    # --- Build MJCF XML ---
    mujoco_el = Element("mujoco", model="lego_technic")

    # Compiler settings
    compiler_attrs = {
        "angle": "radian",
        "autolimits": "true",
        "balanceinertia": "true",
    }
    if use_mesh:
        compiler_attrs["meshdir"] = str(mesh_dir)
    SubElement(mujoco_el, "compiler", **compiler_attrs)

    # Options
    SubElement(mujoco_el, "option", timestep=str(timestep), gravity="0 0 -9.81",
               iterations="50", tolerance="1e-10")

    # Default settings for joints and geoms
    default_el = SubElement(mujoco_el, "default")
    SubElement(default_el, "joint", damping="0.01", armature="0.001")
    SubElement(default_el, "geom", rgba="0.8 0.2 0.2 1",
               condim="4", friction="1.0 0.005 0.0001")

    # Mesh assets
    if use_mesh and unit_mesh_files:
        asset_el = SubElement(mujoco_el, "asset")
        for uid in sorted(unit_mesh_files.keys()):
            SubElement(asset_el, "mesh", name=f"mesh_{uid}",
                       file=unit_mesh_files[uid])

    # --- Worldbody ---
    worldbody = SubElement(mujoco_el, "worldbody")

    # Ground plane
    SubElement(worldbody, "geom", name="ground", type="plane",
               size="1 1 0.01", pos="0 0 0", rgba="0.5 0.5 0.5 1",
               friction="1.0 0.005 0.0001")

    # Light
    SubElement(worldbody, "light", name="top", pos="0 0 0.5",
               dir="0 0 -1", diffuse="1 1 1")

    # --- Create bodies recursively ---
    # Joint names for referencing in actuators/equality
    joint_names: dict[int, str] = {}  # joint_index → name

    def _build_body(uid: int, parent_el: Element, parent_com_mj: np.ndarray):
        """Recursively build body XML for unit and its tree-children."""
        unit = units[uid]
        com_mj = _ldraw_to_mujoco(unit.center_of_mass)
        # Position relative to parent body frame
        rel_pos = com_mj - parent_com_mj

        body_name = f"unit_{uid}"
        body_el = SubElement(parent_el, "body", name=body_name, pos=_vec_str(rel_pos))

        # Inertial properties
        mass = max(unit.mass, 0.001)
        half_ext = unit_half_ext.get(uid, np.array([0.005, 0.005, 0.005]))
        # Use box inertia: I = m/12 * (b² + c²) etc.
        Ix = mass / 12.0 * (half_ext[1]**2 + half_ext[2]**2)
        Iy = mass / 12.0 * (half_ext[0]**2 + half_ext[2]**2)
        Iz = mass / 12.0 * (half_ext[0]**2 + half_ext[1]**2)
        # Ensure minimum inertia
        Ix = max(Ix, 1e-8)
        Iy = max(Iy, 1e-8)
        Iz = max(Iz, 1e-8)
        SubElement(body_el, "inertial", mass=f"{mass:.6f}",
                   pos="0 0 0",
                   diaginertia=f"{Ix:.8f} {Iy:.8f} {Iz:.8f}")

        # Joint connecting this body to parent
        if uid in parent_joint:
            ji = parent_joint[uid]
            j = joints[ji]
            jname = f"joint_{ji}"
            joint_names[ji] = jname

            # Joint position relative to this body's origin (com)
            jpos_mj = _ldraw_to_mujoco(j.position) - com_mj
            jaxis_mj = _ldraw_to_mujoco(j.axis)
            jaxis_norm = np.linalg.norm(jaxis_mj)
            if jaxis_norm > 1e-12:
                jaxis_mj = jaxis_mj / jaxis_norm
            else:
                jaxis_mj = np.array([0.0, 0.0, 1.0])

            if j.joint_type == JointType.REVOLUTE:
                SubElement(body_el, "joint", name=jname, type="hinge",
                           pos=_vec_str(jpos_mj), axis=_vec_str(jaxis_mj))
            elif j.joint_type == JointType.FIXED:
                pass  # MuJoCo: just attach rigidly (no joint element)
            elif j.joint_type == JointType.SLIDER:
                SubElement(body_el, "joint", name=jname, type="slide",
                           pos=_vec_str(jpos_mj), axis=_vec_str(jaxis_mj))

        # Visual/collision geom
        if use_mesh and uid in unit_mesh_files:
            SubElement(body_el, "geom", name=f"geom_{uid}", type="mesh",
                       mesh=f"mesh_{uid}", rgba=_unit_color(uid))
        else:
            SubElement(body_el, "geom", name=f"geom_{uid}", type="box",
                       size=_vec_str(half_ext), rgba=_unit_color(uid))

        # Recurse into tree children
        for child_uid in range(len(units)):
            if parent.get(child_uid) == uid:
                _build_body(child_uid, body_el, com_mj)

    def _unit_color(uid: int) -> str:
        """Generate a distinct color for each unit."""
        # Use golden ratio for hue distribution
        hue = (uid * 0.618033988749895) % 1.0
        # HSV to RGB (saturation=0.7, value=0.9)
        import colorsys
        r, g, b = colorsys.hsv_to_rgb(hue, 0.7, 0.9)
        return f"{r:.2f} {g:.2f} {b:.2f} 1"

    # Build chassis as freejoint body (root)
    chassis = units[chassis_unit]
    com_mj = _ldraw_to_mujoco(chassis.center_of_mass)

    chassis_body = SubElement(worldbody, "body", name="unit_0",
                              pos=_vec_str(com_mj))
    # Freejoint allows full 6-DOF movement
    SubElement(chassis_body, "freejoint", name="chassis_free")

    mass = max(chassis.mass, 0.001)
    half_ext = unit_half_ext.get(chassis_unit, np.array([0.005, 0.005, 0.005]))
    Ix = max(mass / 12.0 * (half_ext[1]**2 + half_ext[2]**2), 1e-8)
    Iy = max(mass / 12.0 * (half_ext[0]**2 + half_ext[2]**2), 1e-8)
    Iz = max(mass / 12.0 * (half_ext[0]**2 + half_ext[1]**2), 1e-8)
    SubElement(chassis_body, "inertial", mass=f"{mass:.6f}", pos="0 0 0",
               diaginertia=f"{Ix:.8f} {Iy:.8f} {Iz:.8f}")
    if use_mesh and chassis_unit in unit_mesh_files:
        SubElement(chassis_body, "geom", name="geom_0", type="mesh",
                   mesh=f"mesh_{chassis_unit}", rgba=_unit_color(chassis_unit))
    else:
        SubElement(chassis_body, "geom", name="geom_0", type="box",
                   size=_vec_str(half_ext), rgba=_unit_color(chassis_unit))

    # Build children of chassis
    for child_uid in range(len(units)):
        if parent.get(child_uid) == chassis_unit:
            _build_body(child_uid, chassis_body, com_mj)

    # --- Equality constraints (loop closures) ---
    if loop_joints or scene.gears:
        equality_el = SubElement(mujoco_el, "equality")

        # Loop-closing connect constraints
        for ji in loop_joints:
            j = joints[ji]
            if j.joint_type == JointType.FIXED:
                # Weld constraint
                SubElement(equality_el, "weld",
                           name=f"loop_weld_{ji}",
                           body1=f"unit_{j.unit_a_index}",
                           body2=f"unit_{j.unit_b_index}",
                           anchor=_vec_str(_ldraw_to_mujoco(j.position)))
            else:
                # Connect constraint (point-to-point, allows rotation)
                # Anchor in body1's frame
                com_a = _ldraw_to_mujoco(units[j.unit_a_index].center_of_mass)
                com_b = _ldraw_to_mujoco(units[j.unit_b_index].center_of_mass)
                jpos_mj = _ldraw_to_mujoco(j.position)
                anchor1 = jpos_mj - com_a
                anchor2 = jpos_mj - com_b
                SubElement(equality_el, "connect",
                           name=f"loop_{ji}",
                           body1=f"unit_{j.unit_a_index}",
                           body2=f"unit_{j.unit_b_index}",
                           anchor=_vec_str(anchor1))

        # Gear coupling via joint equality constraints
        for gi, gc in enumerate(scene.gears):
            # Find hinge joints for each gear unit
            joint_a_name = None
            joint_b_name = None
            for ji, jname in joint_names.items():
                j = joints[ji]
                if j.joint_type != JointType.REVOLUTE:
                    continue
                if j.unit_a_index == gc.unit_a_index or j.unit_b_index == gc.unit_a_index:
                    if joint_a_name is None:
                        joint_a_name = jname
                if j.unit_a_index == gc.unit_b_index or j.unit_b_index == gc.unit_b_index:
                    if joint_b_name is None:
                        joint_b_name = jname
            if joint_a_name and joint_b_name:
                # MuJoCo joint equality: joint1 * coef = joint2
                # ratio = teeth_a / teeth_b, so ω_b/ω_a = -ratio
                SubElement(equality_el, "joint",
                           name=f"gear_{gi}",
                           joint1=joint_a_name,
                           joint2=joint_b_name,
                           polycoef=f"0 {-gc.ratio:.6f} 0 0 0")

    # --- Actuators ---
    if scene.motors:
        actuator_el = SubElement(mujoco_el, "actuator")
        for midx, motor in enumerate(scene.motors):
            mj = joints[motor.joint_index]
            # Find joint name
            ji = motor.joint_index
            if ji in joint_names:
                jname = joint_names[ji]
            else:
                jname = f"joint_{ji}"
            speed = motor_speed if motor_speed is not None else motor.speed
            # Use velocity servo actuator
            # Use velocity servo actuator with sufficient gain
            max_torque = motor.max_torque if motor.max_torque > 0 else 1.0
            # Scale up gain to overcome gear train inertia
            kv = max(max_torque * 10.0, 5.0)
            SubElement(actuator_el, "velocity",
                       name=f"motor_{midx}",
                       joint=jname,
                       kv=f"{kv:.4f}",
                       ctrlrange=f"-{speed:.4f} {speed:.4f}")

    # --- Format output ---
    indent(mujoco_el, space="  ")
    xml_str = tostring(mujoco_el, encoding="unicode", xml_declaration=True)
    return xml_str


def simulate_mjcf(
    mjcf_xml: str,
    *,
    duration: float = 5.0,
    timestep: float = 0.001,
    motor_ctrl: Optional[float] = None,
) -> list[dict[str, np.ndarray]]:
    """Run MuJoCo simulation and return body transforms per frame.

    Parameters
    ----------
    mjcf_xml : str
        MJCF XML string.
    duration : float
        Total simulation time in seconds.
    timestep : float
        Simulation timestep.
    motor_ctrl : float, optional
        Control signal for the motor actuator (target velocity).

    Returns
    -------
    list of dicts
        Each entry is one frame (at 60fps sample rate), mapping body name
        to its 3D position (np.ndarray of shape (3,)).
    """
    import mujoco

    model = mujoco.MjModel.from_xml_string(mjcf_xml)
    data = mujoco.MjData(model)

    # Set motor control
    if motor_ctrl is not None and model.nu > 0:
        data.ctrl[0] = motor_ctrl

    frames: list[dict[str, np.ndarray]] = []
    n_steps = int(duration / timestep)
    sample_interval = int(1.0 / (60.0 * timestep))  # sample at 60fps

    for step in range(n_steps):
        # Maintain motor control
        if motor_ctrl is not None and model.nu > 0:
            data.ctrl[0] = motor_ctrl

        mujoco.mj_step(model, data)

        if step % sample_interval == 0:
            frame: dict[str, np.ndarray] = {}
            for i in range(model.nbody):
                name = model.body(i).name
                if name:
                    frame[name] = data.xpos[i].copy()
            frames.append(frame)

    return frames
