"""Automatic motor detection from LDraw part IDs.

This module identifies known LEGO Technic motor parts by their LDraw file
names and provides default physical properties (speed, torque) for each.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from .model import Joint, JointType, Motor, PhysicsScene, Unit


@dataclass
class MotorSpec:
    """Default physical properties for a known LEGO motor part.

    Attributes:
        part_ids:   LDraw file names that correspond to this motor.
        speed:      Default target angular velocity in rad/s.
        max_torque: Default maximum torque in N·m.
        label:      Human-readable motor name.
    """

    part_ids: List[str]
    speed: float
    max_torque: float
    label: str


# Known LEGO Technic motor parts and their approximate specs.
# Speeds are rough estimates based on typical no-load RPM converted to rad/s.
# Torques are approximate stall torques.
KNOWN_MOTORS: List[MotorSpec] = [
    MotorSpec(
        part_ids=["58121.dat", "58121c01.dat"],
        speed=1.0,
        max_torque=0.4,
        label="XL Motor",
    ),
    MotorSpec(
        part_ids=["53787.dat", "53787c01.dat"],
        speed=3.0,
        max_torque=0.2,
        label="Medium Motor (9V)",
    ),
    MotorSpec(
        part_ids=["45503.dat", "45503c01.dat"],
        speed=2.5,
        max_torque=0.3,
        label="Power Functions L-Motor",
    ),
    MotorSpec(
        part_ids=["88008.dat", "88008c01.dat"],
        speed=3.5,
        max_torque=0.15,
        label="Power Functions M-Motor",
    ),
    MotorSpec(
        part_ids=["54696.dat", "54696c01.dat"],
        speed=2.0,
        max_torque=0.4,
        label="Power Functions XL-Motor",
    ),
    MotorSpec(
        part_ids=["26913.dat", "26913c01.dat"],
        speed=3.0,
        max_torque=0.25,
        label="Powered Up L-Motor",
    ),
    MotorSpec(
        part_ids=["22169.dat", "22169c01.dat"],
        speed=4.0,
        max_torque=0.15,
        label="Powered Up M-Motor",
    ),
    MotorSpec(
        part_ids=["bb0959c01.dat"],
        speed=3.5,
        max_torque=0.18,
        label="SPIKE Medium Angular Motor",
    ),
    MotorSpec(
        part_ids=["bb0960c01.dat"],
        speed=2.5,
        max_torque=0.35,
        label="SPIKE Large Angular Motor",
    ),
]


def _build_part_id_lookup() -> Dict[str, MotorSpec]:
    """Create a lookup dict mapping each part_id to its MotorSpec."""
    lookup: Dict[str, MotorSpec] = {}
    for spec in KNOWN_MOTORS:
        for pid in spec.part_ids:
            lookup[pid.lower()] = spec
    return lookup


_MOTOR_LOOKUP: Dict[str, MotorSpec] = _build_part_id_lookup()


def is_motor_part(part_id: str) -> bool:
    """Return True if *part_id* corresponds to a known motor."""
    return part_id.lower() in _MOTOR_LOOKUP


def get_motor_spec(part_id: str) -> MotorSpec | None:
    """Return the MotorSpec for a given part ID, or None if not a motor."""
    return _MOTOR_LOOKUP.get(part_id.lower())


def detect_motors(scene: PhysicsScene) -> List[Motor]:
    """Identify motors in a PhysicsScene by scanning unit parts for known motor IDs.

    For each motor part found, this function looks for revolute joints connected
    to the unit containing that motor and creates a Motor driving the first
    such joint.  If no revolute joint exists on that unit, the motor part is
    skipped (it cannot drive anything without a rotational degree of freedom).

    Returns a list of Motor objects (may be empty).
    """
    motors: List[Motor] = []
    driven_joints: set[int] = set()

    for unit_idx, unit in enumerate(scene.units):
        for brick in unit.bricks:
            spec = get_motor_spec(brick.part_id)
            if spec is None:
                continue

            # Find revolute joints connected to this unit that aren't
            # already driven by another motor.
            for joint_idx, joint in enumerate(scene.joints):
                if joint_idx in driven_joints:
                    continue
                if joint.joint_type != JointType.REVOLUTE:
                    continue
                if joint.unit_a_index != unit_idx and joint.unit_b_index != unit_idx:
                    continue

                motors.append(
                    Motor(
                        joint_index=joint_idx,
                        speed=spec.speed,
                        max_torque=spec.max_torque,
                    )
                )
                driven_joints.add(joint_idx)
                break  # one motor part drives one joint

    return motors
