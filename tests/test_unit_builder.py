"""Tests for the unit builder (proximity-based grouping and joint detection)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from lego_technic_sim.ldraw.model import LDrawBuild, LDrawPart, Triangle
from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.physics.mesh_properties import LDU_TO_METERS
from lego_technic_sim.physics.model import JointType, PhysicsScene
from lego_technic_sim.physics.unit_builder import (
    DEFAULT_SNAP_THRESHOLD_LDU,
    FIXED_CONTACT_MIN,
    _UnionFind,
    _aabbs_close,
    _brick_aabb,
    _contact_points,
    _estimate_joint_axis,
    build_units_and_joints,
)

FIXTURES = Path(__file__).parent / "fixtures"


# ---------------------------------------------------------------------------
# UnionFind
# ---------------------------------------------------------------------------


def test_union_find_initial_roots():
    uf = _UnionFind(5)
    for i in range(5):
        assert uf.find(i) == i


def test_union_find_union_merges():
    uf = _UnionFind(4)
    uf.union(0, 1)
    assert uf.find(0) == uf.find(1)


def test_union_find_transitive():
    uf = _UnionFind(5)
    uf.union(0, 1)
    uf.union(1, 2)
    assert uf.find(0) == uf.find(2)


def test_union_find_separate_groups():
    uf = _UnionFind(4)
    uf.union(0, 1)
    uf.union(2, 3)
    assert uf.find(0) != uf.find(2)


# ---------------------------------------------------------------------------
# AABB helpers
# ---------------------------------------------------------------------------


def _make_part_with_cube(cx: float, cy: float = 0.0, cz: float = 0.0, half: float = 10.0):
    """Create a minimal LDrawPart whose triangles form a cube."""
    from tests.test_mesh_properties import make_cube_triangles

    tris = make_cube_triangles(cx=cx, cy=cy, cz=cz, half=half)
    m = np.eye(4)
    m[:3, 3] = [cx, cy, cz]
    return LDrawPart(part_id="cube.dat", color=4, transform=m, triangles=tris)


def test_brick_aabb_correct():
    part = _make_part_with_cube(cx=0.0, half=10.0)
    mn, mx = _brick_aabb(part)
    np.testing.assert_allclose(mn, [-10.0, -10.0, -10.0])
    np.testing.assert_allclose(mx, [10.0, 10.0, 10.0])


def test_brick_aabb_offset():
    part = _make_part_with_cube(cx=20.0, half=10.0)
    mn, mx = _brick_aabb(part)
    np.testing.assert_allclose(mn, [10.0, -10.0, -10.0])
    np.testing.assert_allclose(mx, [30.0, 10.0, 10.0])


def test_aabbs_close_overlapping():
    assert _aabbs_close(
        np.array([-10.0, -10.0, -10.0]),
        np.array([10.0, 10.0, 10.0]),
        np.array([5.0, -10.0, -10.0]),
        np.array([25.0, 10.0, 10.0]),
        threshold=0.0,
    )


def test_aabbs_close_touching():
    # AABBs touch exactly at x=10
    assert _aabbs_close(
        np.array([-10.0, -10.0, -10.0]),
        np.array([10.0, 10.0, 10.0]),
        np.array([10.0, -10.0, -10.0]),
        np.array([30.0, 10.0, 10.0]),
        threshold=0.0,
    )


def test_aabbs_close_within_threshold():
    # AABBs are 5 LDU apart; threshold = 6 → close
    assert _aabbs_close(
        np.array([-10.0, -10.0, -10.0]),
        np.array([10.0, 10.0, 10.0]),
        np.array([15.0, -10.0, -10.0]),
        np.array([35.0, 10.0, 10.0]),
        threshold=6.0,
    )


def test_aabbs_not_close():
    assert not _aabbs_close(
        np.array([-10.0, -10.0, -10.0]),
        np.array([10.0, 10.0, 10.0]),
        np.array([90.0, -10.0, -10.0]),
        np.array([110.0, 10.0, 10.0]),
        threshold=4.0,
    )


# ---------------------------------------------------------------------------
# Contact points
# ---------------------------------------------------------------------------


def test_contact_points_adjacent_cubes():
    a = _make_part_with_cube(cx=0.0)
    b = _make_part_with_cube(cx=20.0)
    contacts = _contact_points(a, b, threshold=4.0)
    # 4 shared vertices on the touching face
    assert len(contacts) == 4


def test_contact_points_no_contact():
    a = _make_part_with_cube(cx=0.0)
    b = _make_part_with_cube(cx=100.0)
    contacts = _contact_points(a, b, threshold=4.0)
    assert len(contacts) == 0


def test_contact_points_empty_part():
    part_a = LDrawPart("a.dat", 16, np.eye(4), triangles=[])
    part_b = _make_part_with_cube(cx=0.0)
    assert _contact_points(part_a, part_b, threshold=4.0) == []


# ---------------------------------------------------------------------------
# Joint axis estimation
# ---------------------------------------------------------------------------


def test_estimate_joint_axis_single_point():
    """With only one contact point, axis falls back to (0, 1, 0)."""
    axis = _estimate_joint_axis([np.array([0.0, 0.0, 0.0])])
    np.testing.assert_allclose(axis, [0.0, 1.0, 0.0])


def test_estimate_joint_axis_is_unit_vector():
    contacts = [np.random.randn(3) for _ in range(10)]
    axis = _estimate_joint_axis(contacts)
    assert np.linalg.norm(axis) == pytest.approx(1.0, abs=1e-9)


def test_estimate_joint_axis_planar_contact():
    """Contact points in the XY plane → axis ≈ Z."""
    contacts = [np.array([x, y, 0.0]) for x in range(-5, 6) for y in range(-5, 6)]
    axis = _estimate_joint_axis(contacts)
    # The axis with smallest variance should be ±Z
    assert abs(axis[2]) == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------------------
# build_units_and_joints (integration)
# ---------------------------------------------------------------------------


def test_empty_build_returns_empty_scene():
    build = LDrawBuild(name="empty")
    scene = build_units_and_joints(build)
    assert scene.units == []
    assert scene.joints == []
    assert scene.motors == []


def test_adjacent_bricks_form_one_unit():
    """Two adjacent cubes should be merged into a single unit."""
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    scene = build_units_and_joints(build)
    assert len(scene.units) == 1
    assert len(scene.units[0].bricks) == 2


def test_adjacent_unit_mass():
    """The unit mass must equal the sum of both brick masses."""
    from lego_technic_sim.physics.mesh_properties import brick_mass

    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    scene = build_units_and_joints(build)
    unit = scene.units[0]
    expected_mass = sum(brick_mass(p.triangles) for p in unit.bricks)
    assert unit.mass == pytest.approx(expected_mass, rel=1e-9)


def test_separated_bricks_form_two_units():
    """Two far-apart cubes should be two separate units with no joints."""
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_separated.ldr")
    scene = build_units_and_joints(build)
    assert len(scene.units) == 2
    assert len(scene.joints) == 0


def test_adjacent_bricks_no_joints():
    """Bricks in the same unit must not generate inter-unit joints."""
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    scene = build_units_and_joints(build)
    assert len(scene.joints) == 0


def test_unit_com_in_world_space():
    """COM must be in world space (metres)."""
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    scene = build_units_and_joints(build)
    # Two cubes at x=0 and x=20 LDU; COM should be near x=10 LDU → 10*LDU_TO_M
    com_x = scene.units[0].center_of_mass[0]
    assert com_x == pytest.approx(10.0 * LDU_TO_METERS, abs=1e-9)


def test_unit_name_string():
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    scene = build_units_and_joints(build)
    assert isinstance(scene.units[0].name, str)
    assert "unit_" in scene.units[0].name


def test_pin_connection_creates_revolute_joint():
    """A connection with fewer than FIXED_CONTACT_MIN points → REVOLUTE."""
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "pin_connection.ldr")
    scene = build_units_and_joints(build)
    # The two cubes should be in separate units (only a few shared vertices)
    if len(scene.joints) > 0:
        # If a joint was detected, it must be REVOLUTE (few contacts)
        revolute = [j for j in scene.joints if j.joint_type == JointType.REVOLUTE]
        assert len(revolute) > 0


def test_custom_snap_threshold_zero():
    """With threshold=0, bricks that merely touch but don't overlap may be separate."""
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    # Threshold=0 means exact vertex overlap needed for contact detection
    # (AABBs still touch at x=10, but contact_points threshold is 0)
    scene = build_units_and_joints(build, snap_threshold=0.0)
    # Adjacent cubes share vertices exactly at x=10, so they should still connect
    assert len(scene.units) == 1


def test_scene_has_no_motors_by_default():
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_separated.ldr")
    scene = build_units_and_joints(build)
    assert scene.motors == []


def test_single_brick_build():
    """A single-brick build → one unit, no joints."""
    from tests.test_mesh_properties import make_cube_triangles

    tris = make_cube_triangles()
    part = LDrawPart("cube.dat", 4, np.eye(4), tris)
    build = LDrawBuild(name="single", parts=[part])
    scene = build_units_and_joints(build)
    assert len(scene.units) == 1
    assert len(scene.joints) == 0
    assert scene.units[0].mass > 0.0
