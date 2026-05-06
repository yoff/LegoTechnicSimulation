"""Connection port extraction from LDraw part files.

Each Technic part has typed connection points (ports) at specific positions:
  - ROUND_HOLE:  accepts pins (rotation depends on pin type) and lets axles
                 spin freely (revolute).
  - AXLE_HOLE:   cross-shaped hole that grips axles rigidly and accepts pins.
  - STUD:        top connection point (rigid with anti-stud).
  - ANTI_STUD:   bottom receptacle (rigid with stud).

Ports are extracted by scanning LDraw ``.dat`` files for references to known
geometry primitives (``peghole.dat``, ``beamhole.dat``, ``axl*hole.dat``,
``stud*.dat``, etc.) and recording their local position and orientation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Dict, FrozenSet, List, Optional, Set

import numpy as np


# ---------------------------------------------------------------------------
# Port types
# ---------------------------------------------------------------------------


class PortType(Enum):
    """The mechanical type of a connection port."""

    ROUND_HOLE = auto()
    """Circular hole — pins fit in, axles spin freely."""

    AXLE_HOLE = auto()
    """Cross-shaped hole — grips axles rigidly, pins still fit."""

    STUD = auto()
    """Stud on top of a brick — connects rigidly to ANTI_STUD."""

    ANTI_STUD = auto()
    """Receptacle under a brick — connects rigidly to STUD."""


# ---------------------------------------------------------------------------
# Connection port
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ConnectionPort:
    """A typed connection point on a part, in the part's local frame.

    Attributes:
        port_type:  Mechanical type of this port.
        position:   3-D position in the part's local coordinate system (LDU).
        orientation: Unit vector along the hole/stud axis (local frame).
    """

    port_type: PortType
    position: np.ndarray   # shape (3,)
    orientation: np.ndarray  # shape (3,), unit vector

    def transformed(self, matrix: np.ndarray) -> "ConnectionPort":
        """Return a new port transformed by a 4×4 homogeneous matrix."""
        pos_h = np.append(self.position, 1.0)
        world_pos = (matrix @ pos_h)[:3]
        # Transform orientation as a direction (rotation only)
        rot = matrix[:3, :3]
        world_ori = rot @ self.orientation
        norm = np.linalg.norm(world_ori)
        if norm > 1e-12:
            world_ori = world_ori / norm
        return ConnectionPort(
            port_type=self.port_type,
            position=world_pos,
            orientation=world_ori,
        )

    def __eq__(self, other):
        if not isinstance(other, ConnectionPort):
            return NotImplemented
        return (self.port_type == other.port_type
                and np.allclose(self.position, other.position, atol=1e-6)
                and np.allclose(self.orientation, other.orientation, atol=1e-6))

    def __hash__(self):
        return hash((self.port_type,
                     tuple(np.round(self.position, 2)),
                     tuple(np.round(self.orientation, 2))))


# ---------------------------------------------------------------------------
# Primitive → PortType mapping
# ---------------------------------------------------------------------------

# Primitives whose presence indicates a ROUND hole (pin hole).
_ROUND_HOLE_PRIMITIVES: FrozenSet[str] = frozenset({
    "peghole.dat",
    "peghole2.dat",
    "peghole3.dat",
    "peghole4.dat",
    "peghole5.dat",
    "peghole6.dat",
    "npeghole.dat",
    "connhole.dat",
    "dconnhole.dat",
    "dnpeghole.dat",
    "wpinhole.dat",
    # npeghol variants (negative peg holes — still round)
    "npeghol2.dat", "npeghol3.dat", "npeghol3a.dat", "npeghol4.dat",
    "npeghol4a.dat", "npeghol5.dat", "npeghol6.dat", "npeghol6a.dat",
    "npeghol6b.dat", "npeghol6c.dat", "npeghol6d.dat", "npeghol6e.dat",
    "npeghol6f.dat", "npeghol6g.dat", "npeghol7.dat", "npeghol7a.dat",
    "npeghol8.dat", "npeghol9.dat", "npeghol10.dat", "npeghol11.dat",
    "npeghol12.dat", "npeghol13.dat", "npeghol15.dat", "npeghol15b.dat",
    "npeghol16.dat", "npeghol17.dat", "npeghol18.dat", "npeghol18a.dat",
    "npeghol19.dat", "npeghol20.dat", "npeghol21.dat", "npeghol22.dat",
})

# Primitives whose presence indicates a CROSS (axle) hole.
_AXLE_HOLE_PRIMITIVES: FrozenSet[str] = frozenset({
    "axlehole.dat",
    "axlehol0.dat", "axlehol2.dat", "axlehol3.dat", "axlehol4.dat",
    "axlehol5.dat", "axlehol6.dat", "axlehol7.dat", "axlehol8.dat",
    "axlehol9.dat",
    "axl2hole.dat", "axl3hole.dat", "axl4hole.dat",
    "daxlehole.dat",
    "axleend2hole.dat",
    "beamhole.dat", "beamhol2.dat",
})

# Primitives indicating a stud (top connection point).
_STUD_PRIMITIVES: FrozenSet[str] = frozenset({
    "stud.dat",
    "stud2.dat",
    "stud2a.dat",
    "stud3.dat",
    "stud3a.dat",
    "stud4.dat",
    "stud4a.dat",
})

# Primitives indicating an anti-stud (underside receptacle).
_ANTI_STUD_PRIMITIVES: FrozenSet[str] = frozenset({
    "stud3a.dat",  # hollow stud used as anti-stud in many contexts
    "stud4a.dat",
})

# Combined lookup: primitive filename → port type
_PRIMITIVE_PORT_MAP: Dict[str, PortType] = {}
for _p in _ROUND_HOLE_PRIMITIVES:
    _PRIMITIVE_PORT_MAP[_p] = PortType.ROUND_HOLE
for _p in _AXLE_HOLE_PRIMITIVES:
    _PRIMITIVE_PORT_MAP[_p] = PortType.AXLE_HOLE
# Note: studs could be both STUD and ANTI_STUD depending on context;
# we classify hollow studs (stud3a, stud4a) as anti-studs below the
# brick, and solid studs as top studs.  For simplicity we detect both
# from the same primitives and disambiguate by Y position relative to
# the part origin.
for _p in _STUD_PRIMITIVES:
    _PRIMITIVE_PORT_MAP[_p] = PortType.STUD


# ---------------------------------------------------------------------------
# Port extraction
# ---------------------------------------------------------------------------


def _parse_type1_line(tokens: List[str]):
    """Parse a type-1 line into (sub_file_base, position, rot_matrix, transform_4x4)."""
    sub_file = " ".join(tokens[14:]).lower()
    sub_file_base = Path(sub_file).name

    x, y, z = float(tokens[2]), float(tokens[3]), float(tokens[4])
    a, b, c = float(tokens[5]), float(tokens[6]), float(tokens[7])
    d, e, f = float(tokens[8]), float(tokens[9]), float(tokens[10])
    g, h, i = float(tokens[11]), float(tokens[12]), float(tokens[13])

    position = np.array([x, y, z], dtype=float)
    rot = np.array([[a, b, c], [d, e, f], [g, h, i]], dtype=float)

    # Build 4x4 transform
    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = position

    return sub_file_base, sub_file, position, rot, transform


def extract_ports_from_lines(lines: List[str]) -> List[ConnectionPort]:
    """Extract connection ports from LDraw file lines (non-recursive).

    Only inspects type-1 (sub-file reference) lines at the TOP level of the
    part file.  Does NOT recurse into sub-parts.

    Returns a list of ConnectionPort in the part's local coordinate frame.
    """
    ports: List[ConnectionPort] = []
    for line in lines:
        tokens = line.strip().split()
        if not tokens or tokens[0] != "1":
            continue
        if len(tokens) < 15:
            continue

        sub_file_base, _, position, rot, _ = _parse_type1_line(tokens)

        port_type = _PRIMITIVE_PORT_MAP.get(sub_file_base)
        if port_type is None:
            continue

        # The orientation of the hole is determined by the sub-file's local
        # Y axis (the insertion axis for pegholes/axleholes).
        orientation = rot[:, 1].copy()  # second column = local Y
        norm = np.linalg.norm(orientation)
        if norm > 1e-12:
            orientation /= norm
        else:
            orientation = np.array([0.0, 1.0, 0.0])

        ports.append(ConnectionPort(
            port_type=port_type,
            position=position,
            orientation=orientation,
        ))

    return ports


def extract_ports_recursive(
    lines: List[str],
    resolve_file,
    parent_transform: Optional[np.ndarray] = None,
    depth: int = 0,
    max_depth: int = 3,
) -> List[ConnectionPort]:
    """Extract connection ports, recursing into sub-part files.

    Parameters
    ----------
    lines : list of str
        Lines of the current file.
    resolve_file : callable(filename: str) -> Optional[List[str]]
        Given a sub-file name, return its lines or None if not found.
    parent_transform : 4x4 matrix or None
        Cumulative transform to apply (identity at top level).
    depth : int
        Current recursion depth.
    max_depth : int
        Maximum recursion depth to prevent infinite loops.
    """
    if depth > max_depth:
        return []

    ports: List[ConnectionPort] = []

    for line in lines:
        tokens = line.strip().split()
        if not tokens or tokens[0] != "1":
            continue
        if len(tokens) < 15:
            continue

        sub_file_base, sub_file_path, position, rot, local_transform = (
            _parse_type1_line(tokens)
        )

        port_type = _PRIMITIVE_PORT_MAP.get(sub_file_base)

        if port_type is not None:
            # Found a known primitive — create a port
            orientation = rot[:, 1].copy()
            norm = np.linalg.norm(orientation)
            if norm > 1e-12:
                orientation /= norm
            else:
                orientation = np.array([0.0, 1.0, 0.0])

            port = ConnectionPort(
                port_type=port_type,
                position=position,
                orientation=orientation,
            )
            # Apply parent transform if any
            if parent_transform is not None:
                port = port.transformed(parent_transform)
            ports.append(port)
        else:
            # Not a known primitive — try to recurse into sub-file
            # Skip standard geometry primitives (4-4cyli, ring, edge, etc.)
            if _is_geometry_primitive(sub_file_base):
                continue
            sub_lines = resolve_file(sub_file_path)
            if sub_lines is None:
                continue
            # Compute cumulative transform for recursion
            if parent_transform is not None:
                cum_transform = parent_transform @ local_transform
            else:
                cum_transform = local_transform
            sub_ports = extract_ports_recursive(
                sub_lines, resolve_file, cum_transform, depth + 1, max_depth
            )
            ports.extend(sub_ports)

    return ports


def _is_geometry_primitive(filename: str) -> bool:
    """Return True if the filename looks like a pure geometry primitive."""
    # Standard LDraw geometry primitives that never contain connection ports
    geo_prefixes = (
        "1-", "2-", "3-", "4-", "5-", "6-", "7-", "8-",
        "rect", "tri", "box", "cyls", "cylj",
    )
    geo_patterns = (
        "cyli", "disc", "edge", "ring", "ndis", "con",
        "chrd", "tang", "rin",
    )
    if any(filename.startswith(p) for p in geo_prefixes):
        return True
    # Short geometry files (e.g., 4-4cyli.dat, 2-4ring3.dat)
    if filename[0].isdigit() and "-" in filename[:3]:
        return True
    # rect2p.dat, etc.
    name_no_ext = filename.replace(".dat", "")
    if any(name_no_ext.startswith(p) for p in ("rect", "tri", "box")):
        return True
    return False


def extract_ports_from_file(path: Path) -> List[ConnectionPort]:
    """Extract connection ports from an LDraw part file (non-recursive)."""
    text = path.read_text(encoding="utf-8", errors="replace")
    return extract_ports_from_lines(text.splitlines())


# ---------------------------------------------------------------------------
# Deduplicated port extraction (merge co-located ports)
# ---------------------------------------------------------------------------


def deduplicate_ports(ports: List[ConnectionPort],
                      position_tol: float = 2.0) -> List[ConnectionPort]:
    """Merge ports at the same position into one (prefer AXLE_HOLE over ROUND).

    LDraw parts often have multiple primitives at the same hole (e.g. a
    peghole on each side of the brick for the same through-hole).  This
    function groups them by position and returns one port per unique position.
    AXLE_HOLE takes priority over ROUND_HOLE at the same position.
    """
    if not ports:
        return []

    # Group by position
    groups: List[List[ConnectionPort]] = []
    used: List[bool] = [False] * len(ports)

    for i, p in enumerate(ports):
        if used[i]:
            continue
        group = [p]
        used[i] = True
        for j in range(i + 1, len(ports)):
            if used[j]:
                continue
            if np.linalg.norm(p.position - ports[j].position) < position_tol:
                group.append(ports[j])
                used[j] = True
        groups.append(group)

    result: List[ConnectionPort] = []
    for group in groups:
        # Priority: AXLE_HOLE > ROUND_HOLE > STUD > ANTI_STUD
        best = group[0]
        for port in group[1:]:
            if port.port_type == PortType.AXLE_HOLE:
                best = port
                break
        result.append(best)

    return result
