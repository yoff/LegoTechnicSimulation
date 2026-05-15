"""Tests for port extraction on real LDraw parts.

These tests parse actual .dat part files from the LDraw library and verify
that the expected port types and counts are extracted.  They would have
caught several bugs:

- 42003.dat misclassified as a frictionless pin connector (it has connhole
  ports → structural).
- 32013.dat (Angle Connector) missing AXLE_HOLE ports due to unrecognised
  axl2hol* primitives in sub-parts.
- stud3.dat / stud4.dat incorrectly classified as STUD instead of ANTI_STUD.
- Motor 58121.dat must expose both AXLE_HOLE (output shaft) and ROUND_HOLE
  (bearing) ports along its output axis.
"""

from pathlib import Path

import numpy as np
import pytest

from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.physics.connection_ports import ConnectionPort, PortType

LDRAW_LIB = Path("/opt/ldraw/ldraw")

_skip_no_ldraw = pytest.mark.skipif(
    not LDRAW_LIB.exists(), reason="LDraw library not available"
)


@pytest.fixture(scope="module")
def parser():
    return LDrawParser(parts_dir=LDRAW_LIB)


def _ports(parser, part_id):
    """Return ports for a real LDraw part."""
    return parser._load_part_ports(part_id, LDRAW_LIB / "parts")


def _count(ports, port_type):
    return sum(1 for p in ports if p.port_type == port_type)


# ---------------------------------------------------------------------------
# 42003.dat — Technic Cross Block 1×3 (Axle/Pin/Pin)
# Was misclassified as a frictionless pin connector.  It is structural
# and must expose connhole (ROUND_HOLE) and axle hole ports.
# ---------------------------------------------------------------------------
@_skip_no_ldraw
class Test42003CrossBlock:
    def test_has_ports(self, parser):
        ports = _ports(parser, "42003.dat")
        assert len(ports) >= 2

    def test_has_round_holes(self, parser):
        ports = _ports(parser, "42003.dat")
        assert _count(ports, PortType.ROUND_HOLE) >= 2

    def test_has_axle_hole(self, parser):
        ports = _ports(parser, "42003.dat")
        assert _count(ports, PortType.AXLE_HOLE) >= 1


# ---------------------------------------------------------------------------
# 32013.dat — Technic Angle Connector #1
# Had zero AXLE_HOLE ports because axl2hol2.dat etc. were missing from
# _AXLE_HOLE_PRIMITIVES.  Sub-part s/32013s01.dat contains the axle holes.
# ---------------------------------------------------------------------------
@_skip_no_ldraw
class Test32013AngleConnector:
    def test_has_ports(self, parser):
        ports = _ports(parser, "32013.dat")
        assert len(ports) >= 2

    def test_has_axle_holes(self, parser):
        ports = _ports(parser, "32013.dat")
        assert _count(ports, PortType.AXLE_HOLE) >= 2, (
            "Angle connector must have AXLE_HOLE ports (requires recursive "
            "extraction through sub-parts with axl2hol* primitives)"
        )

    def test_has_round_hole(self, parser):
        ports = _ports(parser, "32013.dat")
        assert _count(ports, PortType.ROUND_HOLE) >= 1


# ---------------------------------------------------------------------------
# 58121.dat — Technic Motor (9V)
# Must have AXLE_HOLE ports (output shaft) and ROUND_HOLE ports along the
# output axis (internal bearings).  The output-axis ROUND_HOLEs were
# incorrectly treated as structural mounting holes.
# ---------------------------------------------------------------------------
@_skip_no_ldraw
class Test58121Motor:
    def test_has_axle_holes(self, parser):
        ports = _ports(parser, "58121.dat")
        assert _count(ports, PortType.AXLE_HOLE) >= 2

    def test_has_round_holes(self, parser):
        ports = _ports(parser, "58121.dat")
        assert _count(ports, PortType.ROUND_HOLE) >= 4

    def test_output_axis_round_holes_exist(self, parser):
        """ROUND_HOLEs coaxial with AXLE_HOLEs are output bearings."""
        ports = _ports(parser, "58121.dat")
        axle_holes = [p for p in ports if p.port_type == PortType.AXLE_HOLE]
        assert axle_holes, "Motor must have AXLE_HOLE ports"
        output_ori = axle_holes[0].orientation

        coaxial_round = [
            p for p in ports
            if p.port_type == PortType.ROUND_HOLE
            and abs(float(np.dot(p.orientation, output_ori))) > 0.9
        ]
        assert len(coaxial_round) >= 2, (
            "Motor must have ROUND_HOLE ports along the output axis "
            "(internal bearings)"
        )

    def test_perpendicular_mounting_holes_exist(self, parser):
        """ROUND_HOLEs perpendicular to output axis are mounting holes."""
        ports = _ports(parser, "58121.dat")
        axle_holes = [p for p in ports if p.port_type == PortType.AXLE_HOLE]
        output_ori = axle_holes[0].orientation

        perp_round = [
            p for p in ports
            if p.port_type == PortType.ROUND_HOLE
            and abs(float(np.dot(p.orientation, output_ori))) < 0.3
        ]
        assert len(perp_round) >= 2, (
            "Motor must have perpendicular ROUND_HOLE ports (body mounting)"
        )


# ---------------------------------------------------------------------------
# Gears — 3648b.dat (24T) and 3647.dat (8T)
# Must have AXLE_HOLE ports for the central axle.
# ---------------------------------------------------------------------------
@_skip_no_ldraw
class TestGearPorts:
    def test_24t_gear_has_axle_hole(self, parser):
        ports = _ports(parser, "3648b.dat")
        assert _count(ports, PortType.AXLE_HOLE) >= 1

    def test_8t_gear_has_axle_hole(self, parser):
        ports = _ports(parser, "3647.dat")
        assert _count(ports, PortType.AXLE_HOLE) >= 1


# ---------------------------------------------------------------------------
# 32523.dat — Technic Beam 3
# Basic structural beam with round holes.
# ---------------------------------------------------------------------------
@_skip_no_ldraw
class Test32523Beam:
    def test_has_round_holes(self, parser):
        ports = _ports(parser, "32523.dat")
        assert _count(ports, PortType.ROUND_HOLE) == 3

    def test_no_studs(self, parser):
        ports = _ports(parser, "32523.dat")
        assert _count(ports, PortType.STUD) == 0
        assert _count(ports, PortType.ANTI_STUD) == 0


# ---------------------------------------------------------------------------
# 3001.dat — Brick 2×4
# Must have STUD (top) and ANTI_STUD (bottom) ports.  stud3.dat and
# stud4.dat were incorrectly classified as STUD instead of ANTI_STUD.
# ---------------------------------------------------------------------------
@_skip_no_ldraw
class Test3001Brick:
    def test_has_studs(self, parser):
        ports = _ports(parser, "3001.dat")
        assert _count(ports, PortType.STUD) >= 4

    def test_has_anti_studs(self, parser):
        ports = _ports(parser, "3001.dat")
        assert _count(ports, PortType.ANTI_STUD) >= 1, (
            "Brick must have ANTI_STUD ports (underside tubes). "
            "stud3.dat/stud4.dat are anti-studs, not studs."
        )

    def test_studs_and_anti_studs_have_opposite_orientations(self, parser):
        """Studs point up, anti-studs point down (opposite along same axis)."""
        ports = _ports(parser, "3001.dat")
        studs = [p for p in ports if p.port_type == PortType.STUD]
        antis = [p for p in ports if p.port_type == PortType.ANTI_STUD]
        assert studs and antis
        # Pick first of each and check orientations are roughly anti-parallel
        dot = float(np.dot(studs[0].orientation, antis[0].orientation))
        assert dot < -0.5, (
            f"Stud and anti-stud orientations should oppose (dot={dot:.2f})"
        )
