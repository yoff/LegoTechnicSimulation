"""Tests for the Blender script exporter."""

from __future__ import annotations

import ast
from pathlib import Path

import numpy as np
import pytest

from lego_technic_sim.physics.model import Joint, JointType, Motor, PhysicsScene, Unit
from lego_technic_sim.blender.exporter import generate_blender_script, _ldraw_to_blender


# ---------------------------------------------------------------------------
# Coordinate conversion helper
# ---------------------------------------------------------------------------


def test_ldraw_to_blender_origin():
    np.testing.assert_allclose(_ldraw_to_blender(np.zeros(3)), [0.0, 0.0, 0.0])


def test_ldraw_to_blender_x_unchanged():
    v = np.array([5.0, 0.0, 0.0])
    result = _ldraw_to_blender(v)
    assert result[0] == pytest.approx(5.0)
    assert result[1] == pytest.approx(0.0)
    assert result[2] == pytest.approx(0.0)


def test_ldraw_to_blender_y_negated_to_z():
    """LDraw Y (down) → Blender -Z."""
    v = np.array([0.0, 1.0, 0.0])
    result = _ldraw_to_blender(v)
    assert result[0] == pytest.approx(0.0)
    assert result[1] == pytest.approx(0.0)
    assert result[2] == pytest.approx(-1.0)


def test_ldraw_to_blender_z_to_neg_y():
    """LDraw Z (toward viewer) → Blender -Y."""
    v = np.array([0.0, 0.0, 1.0])
    result = _ldraw_to_blender(v)
    assert result[0] == pytest.approx(0.0)
    assert result[1] == pytest.approx(-1.0)
    assert result[2] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Script generation
# ---------------------------------------------------------------------------


def _make_scene() -> PhysicsScene:
    unit_a = Unit(mass=0.01, center_of_mass=np.array([0.0, 0.0, 0.0]))
    unit_b = Unit(mass=0.02, center_of_mass=np.array([0.008, 0.0, 0.0]))
    joint = Joint(
        unit_a_index=0,
        unit_b_index=1,
        joint_type=JointType.REVOLUTE,
        position=np.array([0.004, 0.0, 0.0]),
        axis=np.array([0.0, 1.0, 0.0]),
    )
    motor = Motor(joint_index=0, speed=3.14, max_torque=0.1)
    return PhysicsScene(units=[unit_a, unit_b], joints=[joint], motors=[motor])


def test_generate_returns_string():
    scene = _make_scene()
    script = generate_blender_script(scene)
    assert isinstance(script, str)
    assert len(script) > 0


def test_generated_script_is_valid_python():
    """The generated script must be parseable as Python."""
    scene = _make_scene()
    script = generate_blender_script(scene)
    # ast.parse raises SyntaxError for invalid Python
    ast.parse(script)


def test_script_contains_import_bpy():
    scene = _make_scene()
    script = generate_blender_script(scene)
    assert "import bpy" in script


def test_script_contains_rigidbody_world_add():
    scene = _make_scene()
    script = generate_blender_script(scene)
    assert "rigidbody.world_add" in script


def test_script_contains_unit_mass():
    scene = _make_scene()
    script = generate_blender_script(scene)
    assert "0.010000" in script or "0.01" in script


def test_script_contains_revolute_as_hinge():
    scene = _make_scene()
    script = generate_blender_script(scene)
    assert "'HINGE'" in script or '"HINGE"' in script


def test_script_motor_section_present():
    scene = _make_scene()
    script = generate_blender_script(scene)
    assert "use_motor_ang" in script
    assert "3.14" in script or "3.140000" in script


def test_script_no_motor_section_when_no_motors():
    scene = PhysicsScene(
        units=[Unit(mass=0.01, center_of_mass=np.zeros(3))],
        joints=[],
        motors=[],
    )
    script = generate_blender_script(scene)
    assert "use_motor_ang" not in script


def test_script_fixed_joint_type():
    unit_a = Unit(mass=0.01, center_of_mass=np.zeros(3))
    unit_b = Unit(mass=0.01, center_of_mass=np.array([0.01, 0.0, 0.0]))
    joint = Joint(
        unit_a_index=0,
        unit_b_index=1,
        joint_type=JointType.FIXED,
        position=np.array([0.005, 0.0, 0.0]),
        axis=np.array([1.0, 0.0, 0.0]),
    )
    scene = PhysicsScene(units=[unit_a, unit_b], joints=[joint])
    script = generate_blender_script(scene)
    assert "'FIXED'" in script or '"FIXED"' in script


def test_script_slider_joint_type():
    unit_a = Unit(mass=0.01, center_of_mass=np.zeros(3))
    unit_b = Unit(mass=0.01, center_of_mass=np.array([0.01, 0.0, 0.0]))
    joint = Joint(
        unit_a_index=0,
        unit_b_index=1,
        joint_type=JointType.SLIDER,
        position=np.array([0.005, 0.0, 0.0]),
        axis=np.array([1.0, 0.0, 0.0]),
    )
    scene = PhysicsScene(units=[unit_a, unit_b], joints=[joint])
    script = generate_blender_script(scene)
    assert "'SLIDER'" in script or '"SLIDER"' in script


def test_script_write_to_file(tmp_path):
    scene = _make_scene()
    out = tmp_path / "sim.py"
    script = generate_blender_script(scene, output_path=out)
    assert out.exists()
    assert out.read_text(encoding="utf-8") == script


def test_empty_scene_script_still_valid_python():
    script = generate_blender_script(PhysicsScene())
    ast.parse(script)


def test_custom_fps():
    scene = PhysicsScene()
    script = generate_blender_script(scene, fps=120)
    assert "120" in script


def test_script_sets_gravity():
    scene = PhysicsScene()
    script = generate_blender_script(scene)
    # Default gravity should appear in the script
    assert "gravity" in script
    assert "-9.81" in script


def test_script_multiple_units():
    units = [Unit(mass=0.01 * i, center_of_mass=np.array([i * 0.01, 0.0, 0.0])) for i in range(4)]
    scene = PhysicsScene(units=units)
    script = generate_blender_script(scene)
    ast.parse(script)
    # All four units must appear
    assert "_units.append" in script
    assert script.count("rigidbody.object_add") == 5  # 4 units + 1 ground
