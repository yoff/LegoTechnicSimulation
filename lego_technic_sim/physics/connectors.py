"""Connector part classification for Technic joint detection.

LDraw Technic models use connector parts (pins, axles) to join structural
parts (beams, bricks, motors).  This module classifies connector parts by
type and friction properties, which determines whether a connection is rigid
(same unit) or articulated (joint).
"""

from __future__ import annotations

from enum import Enum, auto
from typing import FrozenSet


class ConnectorType(Enum):
    """How a connector part behaves mechanically."""

    FRICTION_PIN = auto()
    """Pin with friction ridges — creates a rigid (or very stiff) connection."""

    FRICTIONLESS_PIN = auto()
    """Smooth pin — allows free rotation (revolute joint)."""

    AXLE = auto()
    """Axle — passes through cross-shaped holes creating a rigid connection.
    Round holes (bearings) allow free rotation, but the majority of Technic
    connections use cross holes so we default to rigid."""

    AXLE_PIN = auto()
    """Combination axle + pin — axle end transmits rotation, pin end may
    be friction or frictionless depending on the specific part."""


# ──────────────────────────────────────────────────────────────────────────────
# Known connector part IDs grouped by type.
# These cover the most common Technic connector parts in LDraw.
# ──────────────────────────────────────────────────────────────────────────────

FRICTION_PIN_IDS: FrozenSet[str] = frozenset({
    "2780.dat",      # Technic Pin with Friction and Slots
    "4459.dat",      # Technic Pin with Friction Ridges
    "6558.dat",      # Technic Pin Long with Friction Ridges
    "32054.dat",     # Technic Pin Long with Stop Bush
    "32138.dat",     # Technic Pin Double
    "65304.dat",     # Technic Pin Long with Friction Ridges (2L)
    "18651.dat",     # Technic Pin with Friction Ridges and Centre Slot
    "60169.dat",     # Technic Pin 1/2 with Friction Ridges
})

FRICTIONLESS_PIN_IDS: FrozenSet[str] = frozenset({
    "3673.dat",      # Technic Pin without Friction Ridges
    "43093.dat",     # Technic Pin without Friction Ridges (Lengthwise)
    "3673a.dat",     # Technic Pin without Friction Ridges (variant)
    "32002.dat",     # Technic Pin 3/4
    "4274.dat",      # Technic Pin 1/2 without Friction Ridges
    "11214.dat",     # Technic Pin Long without Friction (3L)
    "42003.dat",     # Cross Block with 2 Pins (treated as frictionless connector)
    "32556.dat",     # Technic Pin Long without Friction with Single Slot (3L)
})

AXLE_IDS: FrozenSet[str] = frozenset({
    "3704.dat",      # Technic Axle 2
    "3705.dat",      # Technic Axle 4
    "3706.dat",      # Technic Axle 6
    "3707.dat",      # Technic Axle 8
    "3737.dat",      # Technic Axle 10
    "3708.dat",      # Technic Axle 12
    "32073.dat",     # Technic Axle 5
    "44294.dat",     # Technic Axle 7
    "55013.dat",     # Technic Axle 8 with Stop
    "60485.dat",     # Technic Axle 9
    "32209.dat",     # Technic Axle 5.5 with Stop
    "87083.dat",     # Technic Axle 4 with Stop
    "99008.dat",     # Technic Axle 4 with Centre Stop
    "24316.dat",     # Technic Axle 3
    "32062.dat",     # Technic Axle 2 Notched
})

AXLE_PIN_IDS: FrozenSet[str] = frozenset({
    "3749.dat",      # Technic Axle Pin
    "11214.dat",     # Technic Axle Pin (Long variant)
    "43093.dat",     # Technic Axle Pin with Friction (variant)
    "65098.dat",     # Technic Axle Pin with Friction Ridges
    "15462.dat",     # Technic Axle Pin 3L with Friction Ridges and Centre Pin Hole
})

# Combined set of all connector part IDs.
ALL_CONNECTOR_IDS: FrozenSet[str] = (
    FRICTION_PIN_IDS | FRICTIONLESS_PIN_IDS | AXLE_IDS | AXLE_PIN_IDS
)


def classify_connector(part_id: str) -> ConnectorType | None:
    """Return the ConnectorType for a known connector, or None if structural."""
    pid = part_id.lower()
    if pid in _LOOKUP:
        return _LOOKUP[pid]
    return None


def is_connector(part_id: str) -> bool:
    """Return True if the part is a known connector (pin, axle, etc.)."""
    return part_id.lower() in _LOOKUP


def creates_rigid_connection(part_id: str) -> bool:
    """Return True if this connector creates a rigid (non-articulated) bond.

    Friction pins lock parts together.  Axles pass through cross-shaped holes
    in gears, bushings, and joiners — gripping tightly without rotation.
    """
    ctype = classify_connector(part_id)
    return ctype in (ConnectorType.FRICTION_PIN, ConnectorType.AXLE)


def creates_revolute_connection(part_id: str) -> bool:
    """Return True if this connector allows rotation (revolute joint)."""
    ctype = classify_connector(part_id)
    return ctype in (
        ConnectorType.FRICTIONLESS_PIN,
        ConnectorType.AXLE_PIN,
    )


# ──────────────────────────────────────────────────────────────────────────────
# Internal lookup (case-insensitive)
# ──────────────────────────────────────────────────────────────────────────────

def _build_lookup() -> dict[str, ConnectorType]:
    lookup: dict[str, ConnectorType] = {}
    for pid in FRICTION_PIN_IDS:
        lookup[pid.lower()] = ConnectorType.FRICTION_PIN
    for pid in FRICTIONLESS_PIN_IDS:
        lookup[pid.lower()] = ConnectorType.FRICTIONLESS_PIN
    for pid in AXLE_IDS:
        lookup[pid.lower()] = ConnectorType.AXLE
    for pid in AXLE_PIN_IDS:
        lookup[pid.lower()] = ConnectorType.AXLE_PIN
    return lookup


_LOOKUP: dict[str, ConnectorType] = _build_lookup()
