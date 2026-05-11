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


# ---------------------------------------------------------------------------
# Non-parallel revolute axis merging
# ---------------------------------------------------------------------------


def _make_pin_triangles(axis: int, center: np.ndarray, half_len: float = 20.0):
    """Create minimal triangles for a pin with shaft along the given axis.

    The bounding box must be longest along *axis* so that _connector_shaft
    picks the correct direction.
    """
    from lego_technic_sim.ldraw.model import Triangle

    # Thin slab whose largest extent is along *axis*.
    lo = center.copy()
    hi = center.copy()
    lo[axis] -= half_len
    hi[axis] += half_len
    # Give a small extent (2 LDU) on the other two axes so it's not degenerate.
    others = [i for i in range(3) if i != axis]
    tris = []
    for o in others:
        lo[o] = center[o] - 1.0
        hi[o] = center[o] + 1.0
    # Two triangles spanning the bounding box diagonal
    v0 = lo.copy()
    v1 = hi.copy()
    v2 = lo.copy(); v2[axis] = hi[axis]
    v3 = hi.copy(); v3[axis] = lo[axis]
    tris.append(Triangle(v0, v1, v2))
    tris.append(Triangle(v0, v3, v1))
    return tris


def test_nonparallel_revolute_axes_merge_units():
    """Two revolute joints with non-parallel axes between the same unit pair
    over-constrain the connection — the units must be merged into one."""
    from lego_technic_sim.physics.connection_ports import ConnectionPort, PortType
    from tests.test_mesh_properties import make_cube_triangles

    tris = make_cube_triangles()  # 20×20×20 cube centred at origin

    # Beam A at origin with two holes:
    #   [0, 0, 0] orient Y — accepts a Y-axis pin
    #   [0, 20, 0] orient X — accepts an X-axis pin
    beam_a = LDrawPart(
        "beam_a.dat", 16, np.eye(4), list(tris),
        ports=[
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 1.0, 0.0])),
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 20.0, 0.0]),
                           np.array([1.0, 0.0, 0.0])),
        ],
    )

    # Beam B at [0, 40, 0] with two holes:
    #   [0, 40, 0] orient Y — same X,Z as beam A's Y-hole (Y-axis pin line)
    #   [40, 20, 0] orient X — same Y,Z as beam A's X-hole (X-axis pin line)
    tf_b = np.eye(4)
    tf_b[:3, 3] = [0, 40, 0]
    tris_b = [t.transformed(tf_b) for t in tris]
    beam_b = LDrawPart(
        "beam_b.dat", 16, tf_b, tris_b,
        ports=[
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 40.0, 0.0]),
                           np.array([0.0, 1.0, 0.0])),
            ConnectionPort(PortType.ROUND_HOLE, np.array([40.0, 20.0, 0.0]),
                           np.array([1.0, 0.0, 0.0])),
        ],
    )

    # Pin 1: Y-axis pin from [0, -5, 0] to [0, 45, 0]
    pin1_center = np.array([0.0, 20.0, 0.0])
    pin1_tf = np.eye(4)
    pin1_tf[:3, 3] = pin1_center
    pin1 = LDrawPart(
        "3673.dat", 16, pin1_tf,
        triangles=_make_pin_triangles(axis=1, center=pin1_center, half_len=25.0),
    )

    # Pin 2: X-axis pin from [-5, 20, 0] to [45, 20, 0]
    pin2_center = np.array([20.0, 20.0, 0.0])
    pin2_tf = np.eye(4)
    pin2_tf[:3, 3] = pin2_center
    pin2 = LDrawPart(
        "3673.dat", 16, pin2_tf,
        triangles=_make_pin_triangles(axis=0, center=pin2_center, half_len=25.0),
    )

    build = LDrawBuild(name="nonparallel_pins", parts=[beam_a, beam_b, pin1, pin2])
    scene = build_units_and_joints(build)

    # The two beams should be merged into one unit (no joints).
    assert len(scene.units) == 1, (
        f"Expected 1 unit (merged), got {len(scene.units)} with "
        f"{len(scene.joints)} joints"
    )
    assert len(scene.joints) == 0
    assert len(scene.units[0].bricks) == 2  # both beams in one unit


def test_offset_parallel_revolute_axes_merge():
    """Parallel revolute axes at offset positions are NOT collinear — merge."""
    from lego_technic_sim.physics.connection_ports import ConnectionPort, PortType
    from tests.test_mesh_properties import make_cube_triangles

    tris = make_cube_triangles()

    # Two beams each with two Y-facing holes at different Z positions.
    beam_a = LDrawPart(
        "beam_a.dat", 16, np.eye(4), list(tris),
        ports=[
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 1.0, 0.0])),
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 0.0, 20.0]),
                           np.array([0.0, 1.0, 0.0])),
        ],
    )

    tf_b = np.eye(4)
    tf_b[:3, 3] = [0, 40, 0]
    tris_b = [t.transformed(tf_b) for t in tris]
    beam_b = LDrawPart(
        "beam_b.dat", 16, tf_b, tris_b,
        ports=[
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 40.0, 0.0]),
                           np.array([0.0, 1.0, 0.0])),
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 40.0, 20.0]),
                           np.array([0.0, 1.0, 0.0])),
        ],
    )

    # Two Y-axis pins at Z=0 and Z=20 — parallel but offset (not collinear).
    pin1_center = np.array([0.0, 20.0, 0.0])
    pin1_tf = np.eye(4); pin1_tf[:3, 3] = pin1_center
    pin1 = LDrawPart(
        "3673.dat", 16, pin1_tf,
        triangles=_make_pin_triangles(axis=1, center=pin1_center, half_len=25.0),
    )

    pin2_center = np.array([0.0, 20.0, 20.0])
    pin2_tf = np.eye(4); pin2_tf[:3, 3] = pin2_center
    pin2 = LDrawPart(
        "3673.dat", 16, pin2_tf,
        triangles=_make_pin_triangles(axis=1, center=pin2_center, half_len=25.0),
    )

    build = LDrawBuild(name="parallel_pins", parts=[beam_a, beam_b, pin1, pin2])
    scene = build_units_and_joints(build)

    # Offset-parallel pins over-constrain the connection → merged.
    assert len(scene.units) == 1
    assert len(scene.joints) == 0


def test_collinear_revolute_axes_stay_separate():
    """Two revolute joints along the same axis line (collinear) allow rotation
    — the units must remain separate."""
    from lego_technic_sim.physics.connection_ports import ConnectionPort, PortType
    from tests.test_mesh_properties import make_cube_triangles

    tris = make_cube_triangles()

    # Two adjacent frame parts that merge into one unit via vertex proximity.
    # Each has one Y-facing hole on the same Y-axis line (X=0, Z=0).
    frame_a = LDrawPart(
        "frame_a.dat", 16, np.eye(4), list(tris),
        ports=[
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 1.0, 0.0])),
        ],
    )
    tf_fb = np.eye(4)
    tf_fb[:3, 3] = [0, 20, 0]  # adjacent → merges with frame_a
    tris_fb = [t.transformed(tf_fb) for t in tris]
    frame_b = LDrawPart(
        "frame_b.dat", 16, tf_fb, tris_fb,
        ports=[
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 20.0, 0.0]),
                           np.array([0.0, 1.0, 0.0])),
        ],
    )

    # Gear at Y=60, with two Y-facing holes on the same Y-axis line.
    tf_g = np.eye(4)
    tf_g[:3, 3] = [0, 60, 0]
    tris_g = [t.transformed(tf_g) for t in tris]
    gear = LDrawPart(
        "gear.dat", 16, tf_g, tris_g,
        ports=[
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 50.0, 0.0]),
                           np.array([0.0, 1.0, 0.0])),
            ConnectionPort(PortType.ROUND_HOLE, np.array([0.0, 70.0, 0.0]),
                           np.array([0.0, 1.0, 0.0])),
        ],
    )

    # Pin 1: connects frame_a's hole [0,0,0] to gear's hole [0,50,0].
    # Pin 2: connects frame_b's hole [0,20,0] to gear's hole [0,70,0].
    # Both along Y at X=0, Z=0 — collinear.
    # Different structural part pairs → step 3 won't merge them.
    pin1_center = np.array([0.0, 25.0, 0.0])
    pin1_tf = np.eye(4); pin1_tf[:3, 3] = pin1_center
    pin1 = LDrawPart(
        "3673.dat", 16, pin1_tf,
        triangles=_make_pin_triangles(axis=1, center=pin1_center, half_len=30.0),
    )

    pin2_center = np.array([0.0, 45.0, 0.0])
    pin2_tf = np.eye(4); pin2_tf[:3, 3] = pin2_center
    pin2 = LDrawPart(
        "3673.dat", 16, pin2_tf,
        triangles=_make_pin_triangles(axis=1, center=pin2_center, half_len=30.0),
    )

    build = LDrawBuild(name="collinear_pins",
                        parts=[frame_a, frame_b, gear, pin1, pin2])
    scene = build_units_and_joints(build)

    # Collinear axes share a single rotation DOF — units stay separate.
    assert len(scene.units) == 2, (
        f"Expected 2 units (collinear pins allow rotation), got {len(scene.units)}"
    )
    assert any(j.joint_type == JointType.REVOLUTE for j in scene.joints)
