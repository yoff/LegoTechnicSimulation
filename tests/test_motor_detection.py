"""Tests for motor detection logic."""

import numpy as np
import pytest

from lego_technic_sim.ldraw.model import LDrawPart, Triangle
from lego_technic_sim.physics.model import Joint, JointType, Motor, PhysicsScene, Unit
from lego_technic_sim.physics.motor_detection import (
    detect_motors,
    get_motor_spec,
    is_motor_part,
)


def _make_part(part_id: str) -> LDrawPart:
    """Create a minimal LDrawPart with the given part_id."""
    return LDrawPart(
        part_id=part_id,
        color=0,
        transform=np.eye(4),
        triangles=[],
    )


def _make_unit(part_ids: list[str]) -> Unit:
    """Create a Unit containing parts with the given IDs."""
    return Unit(
        bricks=[_make_part(pid) for pid in part_ids],
        mass=0.1,
        center_of_mass=np.zeros(3),
    )


def _revolute_joint(unit_a: int, unit_b: int) -> Joint:
    return Joint(
        unit_a_index=unit_a,
        unit_b_index=unit_b,
        joint_type=JointType.REVOLUTE,
        position=np.zeros(3),
        axis=np.array([0.0, 1.0, 0.0]),
    )


def _fixed_joint(unit_a: int, unit_b: int) -> Joint:
    return Joint(
        unit_a_index=unit_a,
        unit_b_index=unit_b,
        joint_type=JointType.FIXED,
        position=np.zeros(3),
        axis=np.array([0.0, 1.0, 0.0]),
    )


class TestIsMotorPart:
    def test_known_xl_motor(self):
        assert is_motor_part("58121.dat") is True

    def test_known_xl_motor_case_insensitive(self):
        assert is_motor_part("58121.DAT") is True

    def test_regular_brick_not_motor(self):
        assert is_motor_part("3001.dat") is False

    def test_empty_string(self):
        assert is_motor_part("") is False


class TestGetMotorSpec:
    def test_xl_motor_returns_spec(self):
        spec = get_motor_spec("58121.dat")
        assert spec is not None
        assert spec.label == "XL Motor"
        assert spec.speed > 0
        assert spec.max_torque > 0

    def test_unknown_part_returns_none(self):
        assert get_motor_spec("3001.dat") is None


class TestDetectMotors:
    def test_motor_detected_with_revolute_joint(self):
        """A motor part on a unit with a revolute joint should be detected."""
        unit_motor = _make_unit(["58121.dat", "32062.dat"])
        unit_chassis = _make_unit(["3001.dat"])
        joint = _revolute_joint(0, 1)

        scene = PhysicsScene(
            units=[unit_motor, unit_chassis],
            joints=[joint],
        )
        motors = detect_motors(scene)

        assert len(motors) == 1
        assert motors[0].joint_index == 0
        assert motors[0].speed > 0
        assert motors[0].max_torque > 0

    def test_no_motor_parts_means_no_motors(self):
        """No motor parts → no motors detected."""
        unit_a = _make_unit(["3001.dat"])
        unit_b = _make_unit(["3002.dat"])
        joint = _revolute_joint(0, 1)

        scene = PhysicsScene(units=[unit_a, unit_b], joints=[joint])
        motors = detect_motors(scene)

        assert len(motors) == 0

    def test_motor_part_without_revolute_joint_skipped(self):
        """Motor part exists but only fixed joints → no motor created."""
        unit_motor = _make_unit(["58121.dat"])
        unit_other = _make_unit(["3001.dat"])
        joint = _fixed_joint(0, 1)

        scene = PhysicsScene(units=[unit_motor, unit_other], joints=[joint])
        motors = detect_motors(scene)

        assert len(motors) == 0

    def test_motor_part_without_any_joint_skipped(self):
        """Motor part exists but no joints at all → no motor created."""
        unit_motor = _make_unit(["58121.dat"])
        unit_other = _make_unit(["3001.dat"])

        scene = PhysicsScene(units=[unit_motor, unit_other], joints=[])
        motors = detect_motors(scene)

        assert len(motors) == 0

    def test_motor_on_unit_b_side_of_joint(self):
        """Motor on unit_b side of joint is still detected."""
        unit_chassis = _make_unit(["3001.dat"])
        unit_motor = _make_unit(["58121.dat"])
        joint = _revolute_joint(0, 1)

        scene = PhysicsScene(
            units=[unit_chassis, unit_motor],
            joints=[joint],
        )
        motors = detect_motors(scene)

        assert len(motors) == 1
        assert motors[0].joint_index == 0

    def test_two_motors_drive_different_joints(self):
        """Two motor parts on different units each drive their own joint."""
        unit_a = _make_unit(["58121.dat"])
        unit_b = _make_unit(["3001.dat"])
        unit_c = _make_unit(["53787.dat"])
        joint_ab = _revolute_joint(0, 1)
        joint_bc = _revolute_joint(1, 2)

        scene = PhysicsScene(
            units=[unit_a, unit_b, unit_c],
            joints=[joint_ab, joint_bc],
        )
        motors = detect_motors(scene)

        assert len(motors) == 2
        driven = {m.joint_index for m in motors}
        assert driven == {0, 1}

    def test_same_joint_not_driven_twice(self):
        """Two motor parts on the same unit → only one motor created."""
        unit_motor = _make_unit(["58121.dat", "53787.dat"])
        unit_other = _make_unit(["3001.dat"])
        joint = _revolute_joint(0, 1)

        scene = PhysicsScene(
            units=[unit_motor, unit_other],
            joints=[joint],
        )
        motors = detect_motors(scene)

        # Only one revolute joint exists, so at most one motor
        assert len(motors) == 1

    def test_motor_uses_spec_values(self):
        """Motor gets speed/torque from the MotorSpec lookup."""
        unit_motor = _make_unit(["58121.dat"])
        unit_other = _make_unit(["3001.dat"])
        joint = _revolute_joint(0, 1)

        scene = PhysicsScene(
            units=[unit_motor, unit_other],
            joints=[joint],
        )
        motors = detect_motors(scene)

        spec = get_motor_spec("58121.dat")
        assert motors[0].speed == spec.speed
        assert motors[0].max_torque == spec.max_torque
