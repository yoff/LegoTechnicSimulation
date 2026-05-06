"""Tests for connector-based joint detection."""

import numpy as np
import pytest

from lego_technic_sim.ldraw.model import LDrawBuild, LDrawPart, Triangle
from lego_technic_sim.physics.connection_ports import ConnectionPort, PortType
from lego_technic_sim.physics.connectors import (
    ConnectorType,
    classify_connector,
    creates_revolute_connection,
    creates_rigid_connection,
    is_connector,
)
from lego_technic_sim.physics.model import JointType
from lego_technic_sim.physics.unit_builder import build_units_and_joints


def _cube_triangles(center: np.ndarray, size: float = 10.0):
    """Create 12 triangles forming a cube around *center*."""
    hs = size / 2.0
    c = center
    corners = [
        c + np.array([dx, dy, dz]) * hs
        for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)
    ]
    # 6 faces × 2 triangles
    faces = [
        (0, 1, 3, 2), (4, 6, 7, 5),  # -x, +x
        (0, 4, 5, 1), (2, 3, 7, 6),  # -y, +y
        (0, 2, 6, 4), (1, 5, 7, 3),  # -z, +z
    ]
    triangles = []
    for f in faces:
        triangles.append(Triangle(corners[f[0]], corners[f[1]], corners[f[2]]))
        triangles.append(Triangle(corners[f[0]], corners[f[2]], corners[f[3]]))
    return triangles


def _box_triangles(center: np.ndarray, half_extents: np.ndarray):
    """Create 12 triangles forming a box with given half-extents."""
    c = center
    hx, hy, hz = half_extents
    corners = [
        c + np.array([dx * hx, dy * hy, dz * hz])
        for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)
    ]
    faces = [
        (0, 1, 3, 2), (4, 6, 7, 5),
        (0, 4, 5, 1), (2, 3, 7, 6),
        (0, 2, 6, 4), (1, 5, 7, 3),
    ]
    triangles = []
    for f in faces:
        triangles.append(Triangle(corners[f[0]], corners[f[1]], corners[f[2]]))
        triangles.append(Triangle(corners[f[0]], corners[f[2]], corners[f[3]]))
    return triangles


def _make_part(part_id: str, position: np.ndarray, size: float = 10.0,
               port_axis: int = 2):
    """Create a part with cube geometry and a round-hole port at center.

    The port is oriented along *port_axis* (0=X, 1=Y, 2=Z) so that a
    connector along that axis can match it.
    """
    transform = np.eye(4)
    transform[:3, 3] = position
    orientation = np.zeros(3)
    orientation[port_axis] = 1.0
    port = ConnectionPort(
        port_type=PortType.ROUND_HOLE,
        position=position.copy(),
        orientation=orientation,
    )
    return LDrawPart(
        part_id=part_id,
        color=0,
        transform=transform,
        triangles=_cube_triangles(position, size),
        ports=[port],
    )


def _make_connector(part_id: str, position: np.ndarray, shaft_axis: int = 2,
                    length: float = 24.0, cross: float = 8.0):
    """Create a connector with elongated geometry along *shaft_axis*."""
    transform = np.eye(4)
    transform[:3, 3] = position
    half_extents = np.array([cross / 2.0, cross / 2.0, cross / 2.0])
    half_extents[shaft_axis] = length / 2.0
    return LDrawPart(
        part_id=part_id,
        color=0,
        transform=transform,
        triangles=_box_triangles(position, half_extents),
    )


class TestConnectorClassification:
    def test_friction_pin(self):
        assert classify_connector("4459.dat") == ConnectorType.FRICTION_PIN

    def test_frictionless_pin(self):
        assert classify_connector("3673.dat") == ConnectorType.FRICTIONLESS_PIN

    def test_axle(self):
        assert classify_connector("3705.dat") == ConnectorType.AXLE

    def test_axle_pin(self):
        assert classify_connector("3749.dat") == ConnectorType.AXLE_PIN

    def test_structural_part(self):
        assert classify_connector("3001.dat") is None

    def test_is_connector_true(self):
        assert is_connector("4459.dat") is True

    def test_is_connector_false(self):
        assert is_connector("3001.dat") is False

    def test_case_insensitive(self):
        assert is_connector("4459.DAT") is True

    def test_friction_creates_rigid(self):
        assert creates_rigid_connection("4459.dat") is True
        assert creates_rigid_connection("3673.dat") is False

    def test_frictionless_creates_revolute(self):
        assert creates_revolute_connection("3673.dat") is True
        assert creates_revolute_connection("4459.dat") is False

    def test_axle_creates_rigid(self):
        # Axles pass through cross-shaped holes → rigid connection
        assert creates_rigid_connection("3706.dat") is True
        assert creates_revolute_connection("3706.dat") is False

    def test_axle_pin_creates_revolute(self):
        assert creates_revolute_connection("3749.dat") is True


class TestConnectorBasedJoints:
    def test_frictionless_pin_creates_revolute_joint(self):
        """Two beams joined by a frictionless pin → revolute joint."""
        # Beams are size=10 so corner vertices are ~7 LDU from shaft axis
        beam_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=10.0)
        beam_b = _make_part("3002.dat", np.array([0.0, 0.0, 20.0]), size=10.0)
        # Pin elongated along Z spanning both beams
        pin = _make_connector("3673.dat", np.array([0.0, 0.0, 10.0]),
                              shaft_axis=2, length=20.0, cross=6.0)

        build = LDrawBuild(name="test", parts=[beam_a, beam_b, pin])
        scene = build_units_and_joints(build)

        assert len(scene.units) == 2
        assert len(scene.joints) == 1
        assert scene.joints[0].joint_type == JointType.REVOLUTE

    def test_friction_pin_creates_single_unit(self):
        """Two beams joined by a friction pin → same rigid unit."""
        beam_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=10.0)
        beam_b = _make_part("3002.dat", np.array([0.0, 0.0, 20.0]), size=10.0)
        # Friction pin elongated along Z
        pin = _make_connector("4459.dat", np.array([0.0, 0.0, 10.0]),
                              shaft_axis=2, length=20.0, cross=6.0)

        build = LDrawBuild(name="test", parts=[beam_a, beam_b, pin])
        scene = build_units_and_joints(build)

        assert len(scene.units) == 1
        assert len(scene.joints) == 0

    def test_axle_pin_creates_revolute_joint(self):
        """Two beams joined by an axle pin → revolute joint."""
        beam_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=10.0)
        beam_b = _make_part("3002.dat", np.array([0.0, 0.0, 20.0]), size=10.0)
        axle_pin = _make_connector("3749.dat", np.array([0.0, 0.0, 10.0]),
                                   shaft_axis=2, length=20.0, cross=6.0)

        build = LDrawBuild(name="test", parts=[beam_a, beam_b, axle_pin])
        scene = build_units_and_joints(build)

        assert len(scene.units) == 2
        assert len(scene.joints) == 1
        assert scene.joints[0].joint_type == JointType.REVOLUTE

    def test_motor_with_axle_creates_driven_joint(self):
        """Motor connected to a beam via axle → revolute joint + motor."""
        motor = _make_part("58121.dat", np.array([0.0, 0.0, 0.0]), size=10.0)
        beam = _make_part("3001.dat", np.array([0.0, 0.0, 20.0]), size=10.0)
        axle = _make_connector("3749.dat", np.array([0.0, 0.0, 10.0]),
                               shaft_axis=2, length=20.0, cross=6.0)

        build = LDrawBuild(name="test", parts=[motor, beam, axle])
        scene = build_units_and_joints(build)

        assert len(scene.units) == 2
        assert len(scene.joints) == 1
        assert scene.joints[0].joint_type == JointType.REVOLUTE
        assert len(scene.motors) == 1
        assert scene.motors[0].joint_index == 0

    def test_no_connectors_falls_back_to_distance(self):
        """Build with no connector parts uses distance-based fallback."""
        # Two touching cubes (no connectors)
        brick_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=10.0)
        brick_b = _make_part("3002.dat", np.array([10.0, 0.0, 0.0]), size=10.0)

        build = LDrawBuild(name="test", parts=[brick_a, brick_b])
        scene = build_units_and_joints(build)

        # Distance-based: touching parts → merged into one unit
        assert len(scene.units) == 1

    def test_connector_not_overlapping_two_parts_no_joint(self):
        """A connector that only overlaps one structural part creates no joint."""
        beam_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=10.0)
        beam_b = _make_part("3002.dat", np.array([0.0, 0.0, 100.0]), size=10.0)
        # Pin shaft along Z near beam_a only; beam_b is far away
        pin = _make_connector("3673.dat", np.array([0.0, 0.0, 5.0]),
                              shaft_axis=2, length=10.0, cross=8.0)

        build = LDrawBuild(name="test", parts=[beam_a, beam_b, pin])
        scene = build_units_and_joints(build)

        # Two separate units (not connected), no joints
        assert len(scene.units) == 2
        assert len(scene.joints) == 0

    def test_6536_is_structural_not_connector(self):
        """6536.dat (Axle Joiner Perpendicular 3L) must be treated as structural."""
        assert is_connector("6536.dat") is False
        assert classify_connector("6536.dat") is None

    def test_frictionless_3l_pin_through_joiner_creates_revolute(self):
        """Frictionless 3L pin (42003.dat) connecting two sub-assemblies via
        a 6536.dat joiner must produce a revolute joint, not merge them.

        Layout (along Z axis):
          beam_a ←friction pin→ joiner(6536) ←frictionless pin(42003)→ beam_b

        Expected: beam_a + joiner form one rigid unit, beam_b forms another,
        with a revolute joint between them.
        """
        # Body-side beam
        beam_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=10.0)
        # 6536 joiner (structural) adjacent to beam_a
        joiner = _make_part("6536.dat", np.array([0.0, 0.0, 20.0]), size=10.0)
        # Friction pin merging beam_a and joiner into one rigid unit
        friction_pin = _make_connector("4459.dat", np.array([0.0, 0.0, 10.0]),
                                       shaft_axis=2, length=20.0, cross=6.0)
        # Leg-side beam
        beam_b = _make_part("3002.dat", np.array([0.0, 0.0, 40.0]), size=10.0)
        # Frictionless 3L pin connecting joiner and beam_b (revolute)
        free_pin = _make_connector("42003.dat", np.array([0.0, 0.0, 30.0]),
                                   shaft_axis=2, length=20.0, cross=6.0)

        build = LDrawBuild(
            name="test_walker_leg",
            parts=[beam_a, joiner, friction_pin, beam_b, free_pin],
        )
        scene = build_units_and_joints(build)

        # beam_a + joiner = 1 unit, beam_b = 1 unit → 2 units total
        assert len(scene.units) == 2
        assert len(scene.joints) == 1
        assert scene.joints[0].joint_type == JointType.REVOLUTE
