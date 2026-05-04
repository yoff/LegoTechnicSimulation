"""Tests for the LDraw file parser."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from lego_technic_sim.ldraw.model import LDrawBuild, LDrawPart, Triangle
from lego_technic_sim.ldraw.parser import LDrawParser

FIXTURES = Path(__file__).parent / "fixtures"


# ---------------------------------------------------------------------------
# Triangle.transformed
# ---------------------------------------------------------------------------


def test_triangle_identity_transform():
    tri = Triangle(
        v0=np.array([1.0, 0.0, 0.0]),
        v1=np.array([0.0, 1.0, 0.0]),
        v2=np.array([0.0, 0.0, 1.0]),
    )
    result = tri.transformed(np.eye(4))
    np.testing.assert_allclose(result.v0, tri.v0)
    np.testing.assert_allclose(result.v1, tri.v1)
    np.testing.assert_allclose(result.v2, tri.v2)


def test_triangle_translation():
    tri = Triangle(
        v0=np.array([0.0, 0.0, 0.0]),
        v1=np.array([1.0, 0.0, 0.0]),
        v2=np.array([0.0, 1.0, 0.0]),
    )
    m = np.eye(4)
    m[:3, 3] = [10.0, 20.0, 30.0]
    result = tri.transformed(m)
    np.testing.assert_allclose(result.v0, [10.0, 20.0, 30.0])
    np.testing.assert_allclose(result.v1, [11.0, 20.0, 30.0])
    np.testing.assert_allclose(result.v2, [10.0, 21.0, 30.0])


def test_triangle_color_preserved():
    tri = Triangle(np.zeros(3), np.zeros(3), np.zeros(3), color=4)
    result = tri.transformed(np.eye(4))
    assert result.color == 4


# ---------------------------------------------------------------------------
# Parse a simple .dat part
# ---------------------------------------------------------------------------


def test_parse_part_returns_triangles():
    parser = LDrawParser()
    triangles = parser.parse_part(FIXTURES / "cube.dat")
    # A cube has 6 faces; each quad → 2 triangles → 12 total
    assert len(triangles) == 12


def test_parse_part_cached(tmp_path):
    # Second call must return the same list object (from cache)
    dat = FIXTURES / "cube.dat"
    parser = LDrawParser()
    tris1 = parser.parse_part(dat)
    tris2 = parser.parse_part(dat)
    assert tris1 is tris2


def test_parse_part_triangle_dtype():
    parser = LDrawParser()
    triangles = parser.parse_part(FIXTURES / "cube.dat")
    for tri in triangles:
        assert tri.v0.shape == (3,)
        assert tri.v0.dtype == float or np.issubdtype(tri.v0.dtype, np.floating)


# ---------------------------------------------------------------------------
# Parse a .ldr build
# ---------------------------------------------------------------------------


def test_parse_build_part_count():
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    assert isinstance(build, LDrawBuild)
    assert len(build.parts) == 2


def test_parse_build_name():
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    assert build.name == "two_bricks_adjacent"


def test_parse_build_part_positions():
    """The second part should be offset 20 LDU along X."""
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    pos_a = build.parts[0].position
    pos_b = build.parts[1].position
    np.testing.assert_allclose(pos_a, [0.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(pos_b, [20.0, 0.0, 0.0], atol=1e-9)


def test_parse_build_triangles_transformed():
    """Triangles in the build should be in world space (shifted by position)."""
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    part_b = build.parts[1]
    # All triangle vertices of part_b must have floating-point coordinates
    for tri in part_b.triangles:
        for v in (tri.v0, tri.v1, tri.v2):
            assert np.issubdtype(type(v[0]), np.floating) or isinstance(v[0], float)
    # The x-range of part_b should be centred around x=20
    all_x = [v[0] for tri in part_b.triangles for v in (tri.v0, tri.v1, tri.v2)]
    assert min(all_x) == pytest.approx(10.0, abs=1e-6)
    assert max(all_x) == pytest.approx(30.0, abs=1e-6)


def test_parse_build_missing_part_non_fatal(tmp_path):
    """A reference to a missing part should produce an empty triangles list."""
    ldr = tmp_path / "missing.ldr"
    ldr.write_text("1 4 0 0 0 1 0 0 0 1 0 0 0 1 nonexistent_part.dat\n")
    parser = LDrawParser()
    build = parser.parse_build(ldr)
    assert len(build.parts) == 1
    assert build.parts[0].triangles == []


def test_parse_build_part_color():
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    assert build.parts[0].color == 4
    assert build.parts[1].color == 4


def test_parse_build_part_id():
    parser = LDrawParser(parts_dir=FIXTURES)
    build = parser.parse_build(FIXTURES / "two_bricks_adjacent.ldr")
    assert build.parts[0].part_id == "cube.dat"


def test_parse_type3_triangle(tmp_path):
    """Type-3 (triangle) lines should be parsed correctly."""
    dat = tmp_path / "tri.dat"
    dat.write_text("3 16  0 0 0  10 0 0  0 10 0\n")
    parser = LDrawParser()
    tris = parser.parse_part(dat)
    assert len(tris) == 1
    np.testing.assert_allclose(tris[0].v0, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(tris[0].v1, [10.0, 0.0, 0.0])
    np.testing.assert_allclose(tris[0].v2, [0.0, 10.0, 0.0])


def test_parse_type4_quad_split(tmp_path):
    """Type-4 (quad) lines should be split into two triangles."""
    dat = tmp_path / "quad.dat"
    dat.write_text("4 16  0 0 0  10 0 0  10 10 0  0 10 0\n")
    parser = LDrawParser()
    tris = parser.parse_part(dat)
    assert len(tris) == 2


def test_parse_nested_subfile(tmp_path):
    """A .dat that references another .dat should be resolved recursively."""
    inner = tmp_path / "inner.dat"
    inner.write_text("3 16  0 0 0  1 0 0  0 1 0\n")
    outer = tmp_path / "outer.dat"
    outer.write_text("1 16  5 0 0  1 0 0  0 1 0  0 0 1  inner.dat\n")
    parser = LDrawParser()
    tris = parser.parse_part(outer)
    assert len(tris) == 1
    # The triangle should be offset by (5, 0, 0)
    np.testing.assert_allclose(tris[0].v0, [5.0, 0.0, 0.0])
