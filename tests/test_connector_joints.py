"""Tests for connector-based joint detection."""

import numpy as np
import pytest

from lego_technic_sim.ldraw.model import LDrawBuild, LDrawPart, Triangle
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


def _make_part(part_id: str, position: np.ndarray, size: float = 10.0):
    """Create a part with cube geometry at the given position."""
    transform = np.eye(4)
    transform[:3, 3] = position
    return LDrawPart(
        part_id=part_id,
        color=0,
        transform=transform,
        triangles=_cube_triangles(position, size),
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

    def test_axle_creates_revolute(self):
        assert creates_revolute_connection("3749.dat") is True


class TestConnectorBasedJoints:
    def test_frictionless_pin_creates_revolute_joint(self):
        """Two beams joined by a frictionless pin → revolute joint."""
        beam_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=20.0)
        beam_b = _make_part("3002.dat", np.array([0.0, 0.0, 30.0]), size=20.0)
        # Pin overlaps both beams (its bbox must encompass vertices of both)
        pin = _make_part("3673.dat", np.array([0.0, 0.0, 15.0]), size=24.0)

        build = LDrawBuild(name="test", parts=[beam_a, beam_b, pin])
        scene = build_units_and_joints(build)

        assert len(scene.units) == 2
        assert len(scene.joints) == 1
        assert scene.joints[0].joint_type == JointType.REVOLUTE

    def test_friction_pin_creates_single_unit(self):
        """Two beams joined by a friction pin → same rigid unit."""
        beam_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=20.0)
        beam_b = _make_part("3002.dat", np.array([0.0, 0.0, 30.0]), size=20.0)
        # Friction pin overlaps both
        pin = _make_part("4459.dat", np.array([0.0, 0.0, 15.0]), size=24.0)

        build = LDrawBuild(name="test", parts=[beam_a, beam_b, pin])
        scene = build_units_and_joints(build)

        assert len(scene.units) == 1
        assert len(scene.joints) == 0

    def test_axle_pin_creates_revolute_joint(self):
        """Two beams joined by an axle pin → revolute joint."""
        beam_a = _make_part("3001.dat", np.array([0.0, 0.0, 0.0]), size=20.0)
        beam_b = _make_part("3002.dat", np.array([0.0, 0.0, 30.0]), size=20.0)
        axle_pin = _make_part("3749.dat", np.array([0.0, 0.0, 15.0]), size=24.0)

        build = LDrawBuild(name="test", parts=[beam_a, beam_b, axle_pin])
        scene = build_units_and_joints(build)

        assert len(scene.units) == 2
        assert len(scene.joints) == 1
        assert scene.joints[0].joint_type == JointType.REVOLUTE

    def test_motor_with_axle_creates_driven_joint(self):
        """Motor connected to a beam via axle → revolute joint + motor."""
        motor = _make_part("58121.dat", np.array([0.0, 0.0, 0.0]), size=20.0)
        beam = _make_part("3001.dat", np.array([0.0, 0.0, 30.0]), size=20.0)
        axle = _make_part("3749.dat", np.array([0.0, 0.0, 15.0]), size=24.0)

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
        # Pin only reaches beam_a, not beam_b
        pin = _make_part("3673.dat", np.array([0.0, 0.0, 5.0]), size=8.0)

        build = LDrawBuild(name="test", parts=[beam_a, beam_b, pin])
        scene = build_units_and_joints(build)

        # Two separate units (not connected), no joints
        assert len(scene.units) == 2
        assert len(scene.joints) == 0
