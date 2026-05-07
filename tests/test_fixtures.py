"""Integration tests using real .ldr fixture files.

Each test parses a minimal LDraw model and verifies the physics pipeline
produces the correct units, joints, gears, and motors.
"""

from pathlib import Path

import pytest

from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.physics.model import JointType
from lego_technic_sim.physics.unit_builder import build_units_and_joints

FIXTURES = Path(__file__).parent / "fixtures"
LDRAW_LIB = Path("/opt/ldraw/ldraw")


def _parse_fixture(name: str):
    """Parse a fixture .ldr and return (build, scene)."""
    parser = LDrawParser(parts_dir=LDRAW_LIB if LDRAW_LIB.exists() else None)
    build = parser.parse_build(FIXTURES / name)
    scene = build_units_and_joints(build)
    return build, scene


@pytest.mark.skipif(not LDRAW_LIB.exists(), reason="LDraw library not available")
class TestFrictionPin:
    """friction_pin.ldr: two beams + friction pin → 1 rigid unit."""

    def test_single_unit(self):
        _, scene = _parse_fixture("friction_pin.ldr")
        assert len(scene.units) == 1

    def test_no_joints(self):
        _, scene = _parse_fixture("friction_pin.ldr")
        assert len(scene.joints) == 0

    def test_both_beams_in_unit(self):
        build, scene = _parse_fixture("friction_pin.ldr")
        beam_ids = [b.part_id for b in scene.units[0].bricks]
        assert beam_ids.count("32523.dat") == 2


@pytest.mark.skipif(not LDRAW_LIB.exists(), reason="LDraw library not available")
class TestFrictionlessPin:
    """frictionless_pin.ldr: two beams + frictionless pin → 2 units, 1 revolute."""

    def test_two_units(self):
        _, scene = _parse_fixture("frictionless_pin.ldr")
        assert len(scene.units) == 2

    def test_one_revolute_joint(self):
        _, scene = _parse_fixture("frictionless_pin.ldr")
        assert len(scene.joints) == 1
        assert scene.joints[0].joint_type == JointType.REVOLUTE

    def test_each_unit_has_one_beam(self):
        _, scene = _parse_fixture("frictionless_pin.ldr")
        for unit in scene.units:
            assert len(unit.bricks) == 1
            assert unit.bricks[0].part_id == "32523.dat"


@pytest.mark.skipif(not LDRAW_LIB.exists(), reason="LDraw library not available")
class TestMotorGear:
    """motor_gear.ldr: motor + axle + gear → 2 units, 1 revolute, 1 motor."""

    def test_two_units(self):
        _, scene = _parse_fixture("motor_gear.ldr")
        assert len(scene.units) == 2

    def test_one_revolute_joint(self):
        _, scene = _parse_fixture("motor_gear.ldr")
        revolute = [j for j in scene.joints if j.joint_type == JointType.REVOLUTE]
        assert len(revolute) == 1

    def test_one_motor(self):
        _, scene = _parse_fixture("motor_gear.ldr")
        assert len(scene.motors) == 1

    def test_motor_unit_is_motor_part(self):
        _, scene = _parse_fixture("motor_gear.ldr")
        motor = scene.motors[0]
        joint = scene.joints[motor.joint_index]
        motor_unit = scene.units[joint.unit_a_index]
        part_ids = [b.part_id for b in motor_unit.bricks]
        assert "58121.dat" in part_ids

    def test_gear_unit_has_gear(self):
        _, scene = _parse_fixture("motor_gear.ldr")
        motor = scene.motors[0]
        joint = scene.joints[motor.joint_index]
        other_idx = joint.unit_b_index if joint.unit_a_index != joint.unit_b_index else joint.unit_a_index
        gear_unit = scene.units[other_idx]
        part_ids = [b.part_id for b in gear_unit.bricks]
        assert "3647.dat" in part_ids


@pytest.mark.skipif(not LDRAW_LIB.exists(), reason="LDraw library not available")
class TestGearMesh:
    """gear_mesh.ldr: 8T + 24T gears → gear mesh with ratio 0.333."""

    def test_four_units(self):
        _, scene = _parse_fixture("gear_mesh.ldr")
        assert len(scene.units) == 4

    def test_one_gear_mesh(self):
        _, scene = _parse_fixture("gear_mesh.ldr")
        assert len(scene.gears) == 1

    def test_gear_ratio(self):
        _, scene = _parse_fixture("gear_mesh.ldr")
        gc = scene.gears[0]
        assert abs(gc.ratio - 8 / 24) < 0.01

    def test_gear_units_contain_gears(self):
        _, scene = _parse_fixture("gear_mesh.ldr")
        gc = scene.gears[0]
        parts_a = [b.part_id for b in scene.units[gc.unit_a_index].bricks]
        parts_b = [b.part_id for b in scene.units[gc.unit_b_index].bricks]
        assert "3647.dat" in parts_a or "3647.dat" in parts_b
        assert "3648b.dat" in parts_a or "3648b.dat" in parts_b
