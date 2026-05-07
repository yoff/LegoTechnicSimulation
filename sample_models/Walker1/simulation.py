# Auto-generated Blender physics simulation script.
# Created by lego_technic_sim – do not edit by hand.

import bpy
import mathutils

# ── Scene setup ──────────────────────────────────────────────
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

scene = bpy.context.scene
scene.render.fps = 60
if scene.rigidbody_world:
    bpy.ops.rigidbody.world_remove()
bpy.ops.rigidbody.world_add()
scene.rigidbody_world.time_scale = 1.0
scene.gravity = (0.000000, 0.000000, -9.810000)

# ── Units (rigid bodies) ─────────────────────────────────────
_units = []

# Unit 0: unit_64178
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.057880, -0.015117, -0.001208))
_obj = bpy.context.active_object
_obj.name = 'unit_64178'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000082
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 1: unit_32269
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.522681, -0.008816, -0.008501))
_obj = bpy.context.active_object
_obj.name = 'unit_32269'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000001
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 2: unit_32270
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.032130, 0.000000, 0.000000))
_obj = bpy.context.active_object
_obj.name = 'unit_32270'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000414
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 3: unit_32270
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.032130, -0.000000, -0.000000))
_obj = bpy.context.active_object
_obj.name = 'unit_32270'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000414
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 4: unit_4265a
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.032771, 0.007813, -0.000000))
_obj = bpy.context.active_object
_obj.name = 'unit_4265a'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000188
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 5: unit_4143
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.033387, 0.005431, 0.000000))
_obj = bpy.context.active_object
_obj.name = 'unit_4143'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000171
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 6: unit_4265a
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.032771, -0.007813, -0.000000))
_obj = bpy.context.active_object
_obj.name = 'unit_4265a'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000188
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 7: unit_4143
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.033387, -0.005431, 0.000000))
_obj = bpy.context.active_object
_obj.name = 'unit_4143'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000171
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 8: unit_60483_32525_32524
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.046867, -0.029186, -0.011596))
_obj = bpy.context.active_object
_obj.name = 'unit_60483_32525_32524'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.020325
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 9: unit_60483_6536_6536
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.020624, 0.018136, -0.009263))
_obj = bpy.context.active_object
_obj.name = 'unit_60483_6536_6536'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.019663
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 10: unit_60483_6536_3713
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.013031, 0.024711, -0.018550))
_obj = bpy.context.active_object
_obj.name = 'unit_60483_6536_3713'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.014690
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 11: unit_60483_32525_32524
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.037435, -0.029210, -0.011617))
_obj = bpy.context.active_object
_obj.name = 'unit_60483_32525_32524'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.020325
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 12: unit_32270
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.016065, 0.000715, 0.015968))
_obj = bpy.context.active_object
_obj.name = 'unit_32270'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000414
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 13: unit_3713
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.013436, 0.000237, 0.010807))
_obj = bpy.context.active_object
_obj.name = 'unit_3713'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000182
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 14: unit_3648b
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.016000, -0.000000, 0.016000))
_obj = bpy.context.active_object
_obj.name = 'unit_3648b'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000000
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 15: unit_3713
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.013436, -0.000237, 0.010807))
_obj = bpy.context.active_object
_obj.name = 'unit_3713'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000182
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 16: unit_32526_32525_32140
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.001845, 0.000521, 0.044345))
_obj = bpy.context.active_object
_obj.name = 'unit_32526_32525_32140'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.094257
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 17: unit_3647
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.013976, 0.015057, 0.047344))
_obj = bpy.context.active_object
_obj.name = 'unit_3647'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000179
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 18: unit_3648b
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.016000, -0.000000, 0.040000))
_obj = bpy.context.active_object
_obj.name = 'unit_3648b'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000000
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 19: unit_32013
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.027737, -0.015196, 0.028007))
_obj = bpy.context.active_object
_obj.name = 'unit_32013'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000214
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 20: unit_3001
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.020400, 0.042000, -0.042600))
_obj = bpy.context.active_object
_obj.name = 'unit_3001'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.001720
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 21: unit_3001
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.020400, 0.078000, -0.042600))
_obj = bpy.context.active_object
_obj.name = 'unit_3001'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.001720
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 22: unit_3001
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.009600, 0.078000, -0.042600))
_obj = bpy.context.active_object
_obj.name = 'unit_3001'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.001720
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 23: unit_3001
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.009600, 0.042000, -0.042600))
_obj = bpy.context.active_object
_obj.name = 'unit_3001'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.001720
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 24: unit_32316
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.036000, -0.024000, -0.002700))
_obj = bpy.context.active_object
_obj.name = 'unit_32316'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000323
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 25: unit_3001
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.039600, -0.042000, -0.042600))
_obj = bpy.context.active_object
_obj.name = 'unit_3001'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.001720
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 26: unit_3001
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(0.039600, -0.078000, -0.042600))
_obj = bpy.context.active_object
_obj.name = 'unit_3001'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.001720
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 27: unit_32316
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.036000, -0.024000, -0.002700))
_obj = bpy.context.active_object
_obj.name = 'unit_32316'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.000323
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 28: unit_3001
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.050400, -0.042000, -0.042600))
_obj = bpy.context.active_object
_obj.name = 'unit_3001'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.001720
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# Unit 29: unit_3001
bpy.ops.mesh.primitive_cube_add(size=0.02, location=(-0.050400, -0.078000, -0.042600))
_obj = bpy.context.active_object
_obj.name = 'unit_3001'
bpy.ops.rigidbody.object_add()
_obj.rigid_body.mass = 0.001720
_obj.rigid_body.type = 'ACTIVE'
_obj.rigid_body.collision_shape = 'CONVEX_HULL'
_units.append(_obj)

# ── Joints (constraints) ─────────────────────────────────────
_joints = []

# Joint 0: REVOLUTE (unit 0 ↔ unit 1)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_0'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[0]
_con.rigid_body_constraint.object2 = _units[1]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 1: REVOLUTE (unit 0 ↔ unit 2)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_1'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[0]
_con.rigid_body_constraint.object2 = _units[2]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 2: REVOLUTE (unit 0 ↔ unit 3)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_2'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[0]
_con.rigid_body_constraint.object2 = _units[3]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 3: REVOLUTE (unit 1 ↔ unit 2)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_3'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[1]
_con.rigid_body_constraint.object2 = _units[2]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 4: REVOLUTE (unit 1 ↔ unit 3)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_4'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[1]
_con.rigid_body_constraint.object2 = _units[3]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 5: REVOLUTE (unit 2 ↔ unit 3)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_5'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[2]
_con.rigid_body_constraint.object2 = _units[3]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 6: REVOLUTE (unit 3 ↔ unit 4)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_6'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[3]
_con.rigid_body_constraint.object2 = _units[4]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 7: REVOLUTE (unit 3 ↔ unit 5)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_7'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[3]
_con.rigid_body_constraint.object2 = _units[5]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 8: REVOLUTE (unit 3 ↔ unit 10)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_8'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[3]
_con.rigid_body_constraint.object2 = _units[10]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 9: REVOLUTE (unit 3 ↔ unit 11)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_9'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[3]
_con.rigid_body_constraint.object2 = _units[11]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 10: REVOLUTE (unit 4 ↔ unit 5)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_10'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[4]
_con.rigid_body_constraint.object2 = _units[5]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 11: REVOLUTE (unit 4 ↔ unit 10)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_11'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[4]
_con.rigid_body_constraint.object2 = _units[10]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 12: REVOLUTE (unit 4 ↔ unit 11)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_12'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[4]
_con.rigid_body_constraint.object2 = _units[11]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 13: REVOLUTE (unit 5 ↔ unit 10)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_13'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[5]
_con.rigid_body_constraint.object2 = _units[10]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 14: REVOLUTE (unit 5 ↔ unit 11)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_14'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[5]
_con.rigid_body_constraint.object2 = _units[11]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 15: REVOLUTE (unit 10 ↔ unit 11)
bpy.ops.object.empty_add(location=(0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_15'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[10]
_con.rigid_body_constraint.object2 = _units[11]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 16: REVOLUTE (unit 2 ↔ unit 6)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_16'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[2]
_con.rigid_body_constraint.object2 = _units[6]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 17: REVOLUTE (unit 2 ↔ unit 7)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_17'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[2]
_con.rigid_body_constraint.object2 = _units[7]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 18: REVOLUTE (unit 2 ↔ unit 8)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_18'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[2]
_con.rigid_body_constraint.object2 = _units[8]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 19: REVOLUTE (unit 2 ↔ unit 9)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_19'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[2]
_con.rigid_body_constraint.object2 = _units[9]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 20: REVOLUTE (unit 6 ↔ unit 7)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_20'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[6]
_con.rigid_body_constraint.object2 = _units[7]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 21: REVOLUTE (unit 6 ↔ unit 8)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_21'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[6]
_con.rigid_body_constraint.object2 = _units[8]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 22: REVOLUTE (unit 6 ↔ unit 9)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_22'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[6]
_con.rigid_body_constraint.object2 = _units[9]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 23: REVOLUTE (unit 7 ↔ unit 8)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_23'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[7]
_con.rigid_body_constraint.object2 = _units[8]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 24: REVOLUTE (unit 7 ↔ unit 9)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_24'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[7]
_con.rigid_body_constraint.object2 = _units[9]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 25: REVOLUTE (unit 8 ↔ unit 9)
bpy.ops.object.empty_add(location=(-0.040000, -0.000000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_25'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[8]
_con.rigid_body_constraint.object2 = _units[9]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 26: REVOLUTE (unit 0 ↔ unit 9)
bpy.ops.object.empty_add(location=(-0.008000, 0.024000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_26'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[0]
_con.rigid_body_constraint.object2 = _units[9]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -0.000000, -1.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 27: REVOLUTE (unit 0 ↔ unit 10)
bpy.ops.object.empty_add(location=(-0.008000, 0.024000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_27'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[0]
_con.rigid_body_constraint.object2 = _units[10]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -0.000000, -1.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 28: REVOLUTE (unit 0 ↔ unit 16)
bpy.ops.object.empty_add(location=(-0.008000, 0.024000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_28'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[0]
_con.rigid_body_constraint.object2 = _units[16]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -0.000000, -1.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 29: REVOLUTE (unit 9 ↔ unit 10)
bpy.ops.object.empty_add(location=(-0.008000, 0.024000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_29'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[9]
_con.rigid_body_constraint.object2 = _units[10]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -0.000000, -1.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 30: REVOLUTE (unit 9 ↔ unit 16)
bpy.ops.object.empty_add(location=(-0.008000, 0.024000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_30'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[9]
_con.rigid_body_constraint.object2 = _units[16]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -0.000000, -1.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 31: REVOLUTE (unit 10 ↔ unit 16)
bpy.ops.object.empty_add(location=(-0.008000, 0.024000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_31'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[10]
_con.rigid_body_constraint.object2 = _units[16]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -0.000000, -1.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 32: REVOLUTE (unit 9 ↔ unit 12)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_32'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[9]
_con.rigid_body_constraint.object2 = _units[12]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 33: REVOLUTE (unit 9 ↔ unit 13)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_33'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[9]
_con.rigid_body_constraint.object2 = _units[13]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 34: REVOLUTE (unit 9 ↔ unit 15)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_34'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[9]
_con.rigid_body_constraint.object2 = _units[15]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 35: REVOLUTE (unit 10 ↔ unit 12)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_35'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[10]
_con.rigid_body_constraint.object2 = _units[12]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 36: REVOLUTE (unit 10 ↔ unit 13)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_36'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[10]
_con.rigid_body_constraint.object2 = _units[13]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 37: REVOLUTE (unit 10 ↔ unit 15)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_37'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[10]
_con.rigid_body_constraint.object2 = _units[15]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 38: REVOLUTE (unit 12 ↔ unit 13)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_38'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[12]
_con.rigid_body_constraint.object2 = _units[13]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 39: REVOLUTE (unit 12 ↔ unit 15)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_39'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[12]
_con.rigid_body_constraint.object2 = _units[15]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 40: REVOLUTE (unit 13 ↔ unit 15)
bpy.ops.object.empty_add(location=(0.000000, -0.000000, 0.016000))
_con = bpy.context.active_object
_con.name = 'joint_40'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[13]
_con.rigid_body_constraint.object2 = _units[15]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 41: REVOLUTE (unit 16 ↔ unit 17)
bpy.ops.object.empty_add(location=(0.012000, -0.000000, 0.056000))
_con = bpy.context.active_object
_con.name = 'joint_41'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[16]
_con.rigid_body_constraint.object2 = _units[17]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((1.000000, -0.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 42: REVOLUTE (unit 16 ↔ unit 19)
bpy.ops.object.empty_add(location=(-0.032000, -0.020000, 0.056000))
_con = bpy.context.active_object
_con.name = 'joint_42'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[16]
_con.rigid_body_constraint.object2 = _units[19]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 43: REVOLUTE (unit 16 ↔ unit 24)
bpy.ops.object.empty_add(location=(0.064000, -0.036000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_43'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[16]
_con.rigid_body_constraint.object2 = _units[24]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 44: REVOLUTE (unit 24 ↔ unit 11)
bpy.ops.object.empty_add(location=(0.032000, -0.036000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_44'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[24]
_con.rigid_body_constraint.object2 = _units[11]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 45: REVOLUTE (unit 16 ↔ unit 8)
bpy.ops.object.empty_add(location=(-0.040000, -0.036000, 0.032000))
_con = bpy.context.active_object
_con.name = 'joint_45'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[16]
_con.rigid_body_constraint.object2 = _units[8]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 46: REVOLUTE (unit 16 ↔ unit 27)
bpy.ops.object.empty_add(location=(-0.032000, -0.036000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_46'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[16]
_con.rigid_body_constraint.object2 = _units[27]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 47: REVOLUTE (unit 8 ↔ unit 27)
bpy.ops.object.empty_add(location=(-0.064000, -0.036000, -0.000000))
_con = bpy.context.active_object
_con.name = 'joint_47'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[8]
_con.rigid_body_constraint.object2 = _units[27]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# Joint 48: REVOLUTE (unit 16 ↔ unit 11)
bpy.ops.object.empty_add(location=(0.040000, -0.036000, 0.032000))
_con = bpy.context.active_object
_con.name = 'joint_48'
bpy.ops.rigidbody.constraint_add(type='HINGE')
_con.rigid_body_constraint.object1 = _units[16]
_con.rigid_body_constraint.object2 = _units[11]
_con.rigid_body_constraint.disable_collisions = True
_axis = mathutils.Vector((0.000000, -1.000000, -0.000000))
_up = mathutils.Vector((0.0, 0.0, 1.0))
_rot = _up.rotation_difference(_axis)
_con.rotation_mode = 'QUATERNION'
_con.rotation_quaternion = _rot
_joints.append(_con)

# ── Motors ───────────────────────────────────────────────

# Motor 0: drives joint 28
_rbc = _joints[28].rigid_body_constraint
_rbc.use_motor_ang = True
_rbc.motor_ang_target_velocity = 2.000000
_rbc.motor_ang_max_impulse = 0.400000

# ── Finalise ─────────────────────────────────────────────────
bpy.context.view_layer.update()
print('LegoTechnicSimulation: scene ready.')