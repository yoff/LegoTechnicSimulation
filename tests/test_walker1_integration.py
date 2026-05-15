"""Walker1 integration test.

Parses the full Walker1.ldr model and asserts high-level invariants about
the physics scene: unit count, joint count, gear meshes, motors, and
correct separation of key parts (e.g. gears on revolute joints, not merged
into the chassis).

This single test would have caught every bug fixed in this session:
- 42003.dat misclassification (wrong unit count)
- Missing stud/anti-stud merging (wrong unit count)
- Missing axle hole primitives (wrong unit count)
- Axle-pin connection type errors (wrong unit/joint count)
- Motor bearing ROUND_HOLEs incorrectly rigid (gear merged into chassis)
"""

from pathlib import Path

import numpy as np
import pytest

from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.physics.connectors import is_connector
from lego_technic_sim.physics.gears import GEAR_CATALOG
from lego_technic_sim.physics.model import JointType
from lego_technic_sim.physics.motor_detection import is_motor_part
from lego_technic_sim.physics.unit_builder import build_units_and_joints

WALKER1 = Path(__file__).parent.parent / "sample_models" / "Walker1" / "Walker1.ldr"
LDRAW_LIB = Path("/opt/ldraw/ldraw")

_skip = pytest.mark.skipif(
    not (LDRAW_LIB.exists() and WALKER1.exists()),
    reason="LDraw library or Walker1.ldr not available",
)


@pytest.fixture(scope="module")
def walker1():
    """Parse Walker1 once and return (build, scene)."""
    parser = LDrawParser(parts_dir=LDRAW_LIB)
    build = parser.parse_build(WALKER1)
    scene = build_units_and_joints(build)
    return build, scene


def _find_unit(build, scene, part_index):
    """Return the unit index containing the given part index."""
    target = build.parts[part_index]
    for uid, unit in enumerate(scene.units):
        if any(b is target for b in unit.bricks):
            return uid
    return None


# ---------------------------------------------------------------------------
# High-level counts
# ---------------------------------------------------------------------------
@_skip
class TestWalker1Counts:
    def test_unit_count(self, walker1):
        _, scene = walker1
        assert len(scene.units) == 31, (
            f"Expected 31 units, got {len(scene.units)}"
        )

    def test_joint_count(self, walker1):
        _, scene = walker1
        assert len(scene.joints) == 50, (
            f"Expected 50 joints, got {len(scene.joints)}"
        )

    def test_gear_mesh_count(self, walker1):
        _, scene = walker1
        assert len(scene.gears) == 5

    def test_motor_count(self, walker1):
        _, scene = walker1
        assert len(scene.motors) == 1

    def test_all_joints_are_revolute(self, walker1):
        """Walker1 uses only pin/axle connections — all joints are revolute."""
        _, scene = walker1
        for j in scene.joints:
            assert j.joint_type == JointType.REVOLUTE


# ---------------------------------------------------------------------------
# Structural integrity
# ---------------------------------------------------------------------------
@_skip
class TestWalker1Structure:
    def test_every_structural_part_in_a_unit(self, walker1):
        """Every non-connector part must belong to exactly one unit."""
        build, scene = walker1
        assigned = set()
        for unit in scene.units:
            for brick in unit.bricks:
                assigned.add(id(brick))

        for i, p in enumerate(build.parts):
            if not is_connector(p.part_id):
                assert id(p) in assigned, (
                    f"Structural part [{i}] {p.part_id} not in any unit"
                )

    def test_no_empty_units(self, walker1):
        _, scene = walker1
        for uid, unit in enumerate(scene.units):
            assert len(unit.bricks) > 0, f"Unit {uid} is empty"


# ---------------------------------------------------------------------------
# Motor and gear separation
# ---------------------------------------------------------------------------
@_skip
class TestWalker1MotorGears:
    def test_motor_not_merged_with_gears(self, walker1):
        """No gear part should be in the same unit as the motor."""
        build, scene = walker1
        motor_units = set()
        gear_units = set()
        for uid, unit in enumerate(scene.units):
            for brick in unit.bricks:
                if is_motor_part(brick.part_id):
                    motor_units.add(uid)
                if brick.part_id.lower() in GEAR_CATALOG:
                    gear_units.add(uid)

        overlap = motor_units & gear_units
        assert not overlap, (
            f"Gear parts merged into motor unit(s): {overlap}"
        )

    def test_each_gear_on_revolute_joint(self, walker1):
        """Every unit containing a gear must have at least one revolute joint."""
        build, scene = walker1
        gear_units = set()
        for uid, unit in enumerate(scene.units):
            for brick in unit.bricks:
                if brick.part_id.lower() in GEAR_CATALOG:
                    gear_units.add(uid)

        for uid in gear_units:
            has_revolute = any(
                (j.unit_a_index == uid or j.unit_b_index == uid)
                and j.joint_type == JointType.REVOLUTE
                for j in scene.joints
            )
            assert has_revolute, (
                f"Gear unit {uid} has no revolute joint — "
                f"gear is rigidly merged into another body"
            )

    def test_motor_drives_gear_chain(self, walker1):
        """Motor's joint connects to a unit that participates in gear meshes."""
        _, scene = walker1
        motor = scene.motors[0]
        joint = scene.joints[motor.joint_index]
        motor_unit = joint.unit_a_index
        driven_unit = joint.unit_b_index

        gear_mesh_units = set()
        for gc in scene.gears:
            gear_mesh_units.add(gc.unit_a_index)
            gear_mesh_units.add(gc.unit_b_index)

        assert driven_unit in gear_mesh_units, (
            f"Motor-driven unit {driven_unit} not in any gear mesh"
        )
