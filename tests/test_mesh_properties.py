"""Tests for mesh physical-property calculations."""

from __future__ import annotations

import math
from pathlib import Path
from typing import List

import numpy as np
import pytest

from lego_technic_sim.ldraw.model import Triangle
from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.physics.mesh_properties import (
    ABS_DENSITY_KG_PER_M3,
    LDU_TO_METERS,
    brick_mass,
    mesh_volume_and_com,
)

FIXTURES = Path(__file__).parent / "fixtures"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_cube_triangles(
    cx: float = 0.0,
    cy: float = 0.0,
    cz: float = 0.0,
    half: float = 10.0,
) -> List[Triangle]:
    """Return the 12 triangles of a cube centred at (cx, cy, cz).

    Winding: counter-clockwise from outside (outward normals).
    """
    lo, hi = -half, half

    def v(x: float, y: float, z: float) -> np.ndarray:
        return np.array([x + cx, y + cy, z + cz], dtype=float)

    quads = [
        # -Z face
        (v(lo, lo, lo), v(lo, hi, lo), v(hi, hi, lo), v(hi, lo, lo)),
        # +Z face
        (v(lo, lo, hi), v(hi, lo, hi), v(hi, hi, hi), v(lo, hi, hi)),
        # -X face
        (v(lo, lo, lo), v(lo, lo, hi), v(lo, hi, hi), v(lo, hi, lo)),
        # +X face
        (v(hi, lo, lo), v(hi, hi, lo), v(hi, hi, hi), v(hi, lo, hi)),
        # -Y face
        (v(lo, lo, lo), v(hi, lo, lo), v(hi, lo, hi), v(lo, lo, hi)),
        # +Y face
        (v(lo, hi, lo), v(lo, hi, hi), v(hi, hi, hi), v(hi, hi, lo)),
    ]
    triangles: List[Triangle] = []
    for q in quads:
        triangles.append(Triangle(q[0], q[1], q[2]))
        triangles.append(Triangle(q[0], q[2], q[3]))
    return triangles


# ---------------------------------------------------------------------------
# mesh_volume_and_com
# ---------------------------------------------------------------------------


def test_empty_mesh_returns_zeros():
    vol, com = mesh_volume_and_com([])
    assert vol == 0.0
    np.testing.assert_array_equal(com, [0.0, 0.0, 0.0])


def test_cube_volume_at_origin():
    """20×20×20 LDU cube → volume = (20 LDU)³ × (LDU→m)³."""
    triangles = make_cube_triangles(half=10.0)
    vol, _ = mesh_volume_and_com(triangles)
    expected = (20.0 * LDU_TO_METERS) ** 3  # 8000 LDU³ → m³
    assert vol == pytest.approx(expected, rel=1e-6)


def test_cube_volume_offset():
    """Volume should be the same regardless of position."""
    tris_origin = make_cube_triangles(cx=0.0, half=10.0)
    tris_offset = make_cube_triangles(cx=500.0, cy=300.0, cz=-200.0, half=10.0)
    vol_o, _ = mesh_volume_and_com(tris_origin)
    vol_s, _ = mesh_volume_and_com(tris_offset)
    assert vol_o == pytest.approx(vol_s, rel=1e-6)


def test_cube_com_at_origin():
    """Centre of mass of a cube centred at origin must be (0, 0, 0)."""
    triangles = make_cube_triangles(half=10.0)
    _, com = mesh_volume_and_com(triangles)
    np.testing.assert_allclose(com, [0.0, 0.0, 0.0], atol=1e-12)


def test_cube_com_offset():
    """COM of a cube centred at (50, 30, -20) LDU must be that point in metres."""
    cx, cy, cz = 50.0, 30.0, -20.0
    triangles = make_cube_triangles(cx=cx, cy=cy, cz=cz, half=10.0)
    _, com = mesh_volume_and_com(triangles)
    expected = np.array([cx, cy, cz]) * LDU_TO_METERS
    np.testing.assert_allclose(com, expected, atol=1e-10)


def test_larger_cube_has_larger_volume():
    small = make_cube_triangles(half=5.0)
    large = make_cube_triangles(half=20.0)
    vol_s, _ = mesh_volume_and_com(small)
    vol_l, _ = mesh_volume_and_com(large)
    # Volume scales as (side length)³; ratio = (40/10)³ = 64
    assert vol_l == pytest.approx(vol_s * 64.0, rel=1e-6)


# ---------------------------------------------------------------------------
# brick_mass
# ---------------------------------------------------------------------------


def test_brick_mass_positive():
    triangles = make_cube_triangles(half=10.0)
    mass = brick_mass(triangles)
    assert mass > 0.0


def test_brick_mass_correct_formula():
    """mass = density × volume."""
    triangles = make_cube_triangles(half=10.0)
    vol, _ = mesh_volume_and_com(triangles)
    expected = ABS_DENSITY_KG_PER_M3 * vol
    assert brick_mass(triangles) == pytest.approx(expected, rel=1e-9)


def test_brick_mass_empty_mesh():
    assert brick_mass([]) == 0.0


def test_brick_mass_custom_density():
    triangles = make_cube_triangles(half=10.0)
    m1 = brick_mass(triangles, density=1000.0)
    m2 = brick_mass(triangles, density=2000.0)
    assert m2 == pytest.approx(2.0 * m1, rel=1e-9)


# ---------------------------------------------------------------------------
# Integration: parse cube.dat and compute properties
# ---------------------------------------------------------------------------


def test_parsed_cube_volume():
    parser = LDrawParser()
    triangles = parser.parse_part(FIXTURES / "cube.dat")
    vol, _ = mesh_volume_and_com(triangles)
    expected = (20.0 * LDU_TO_METERS) ** 3
    assert vol == pytest.approx(expected, rel=1e-6)


def test_parsed_cube_com_near_origin():
    """The cube fixture is centred at origin, so COM ≈ (0, 0, 0)."""
    parser = LDrawParser()
    triangles = parser.parse_part(FIXTURES / "cube.dat")
    _, com = mesh_volume_and_com(triangles)
    np.testing.assert_allclose(com, [0.0, 0.0, 0.0], atol=1e-10)
