"""Physics data model: Units, Joints, Motors, and the complete scene.

Terminology
-----------
Unit
    A *rigid body* formed by one or more LEGO bricks that are snapped
    together so tightly that they move as one piece.  The simulation
    treats each unit as a single mass with a single centre of mass.

Joint
    A *constraint* between two units.  Three types are supported:

    FIXED    – no relative motion (e.g. two beams bolted together).
    REVOLUTE – rotation around a single axis (e.g. a Technic pin in a hole,
               or an axle through a bearing).
    SLIDER   – translation along a single axis (e.g. a linear actuator).

Motor
    A driven revolute joint.  A motor applies torque to maintain a target
    angular velocity.

PhysicsScene
    The top-level container that gathers all units, joints and motors for
    hand-off to an external simulator (e.g. Blender Rigid Body).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List

import numpy as np

from ..ldraw.model import LDrawPart


class JointType(Enum):
    """Classification of a constraint between two rigid bodies."""

    FIXED = auto()
    """No relative motion between the two units."""

    REVOLUTE = auto()
    """Rotation about a single axis – the classic Technic frictionless snap."""

    SLIDER = auto()
    """Translation along a single axis (e.g. linear actuator)."""


@dataclass
class Unit:
    """A rigid body composed of one or more bricks that move as one.

    Attributes:
        bricks:         The constituent :class:`~lego_technic_sim.ldraw.model.LDrawPart`
                        objects.
        mass:           Total mass in kg.
        center_of_mass: Centre of mass in the build's world coordinate frame,
                        expressed in metres.
    """

    bricks: List[LDrawPart] = field(default_factory=list)
    mass: float = 0.0
    center_of_mass: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @property
    def name(self) -> str:
        """A human-readable name derived from the first few brick IDs."""
        ids = "_".join(
            p.part_id.replace(".dat", "").replace(".DAT", "")
            for p in self.bricks[:3]
        )
        return f"unit_{ids}"


@dataclass
class Joint:
    """A constraint between two :class:`Unit` objects.

    Attributes:
        unit_a_index: Index of the first unit in
                      :attr:`PhysicsScene.units`.
        unit_b_index: Index of the second unit.
        joint_type:   Kind of constraint.
        position:     World position of the joint pivot, in metres.
        axis:         Rotation / translation axis in world space
                      (unit vector).  Relevant for REVOLUTE and SLIDER
                      joints; ignored for FIXED.
    """

    unit_a_index: int
    unit_b_index: int
    joint_type: JointType
    position: np.ndarray
    axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 1.0, 0.0]))


@dataclass
class Motor:
    """A rotary motor that drives a revolute joint.

    Attributes:
        joint_index: Index into :attr:`PhysicsScene.joints`.  The
                     referenced joint *should* be of type REVOLUTE.
        speed:       Target angular velocity in rad/s.
        max_torque:  Maximum torque in N·m.  ``0`` means no limit.
    """

    joint_index: int
    speed: float = 0.0
    max_torque: float = 0.0


@dataclass
class GearConstraint:
    """A gear mesh constraint between two units.

    When unit_a rotates by angle θ about axis_a, unit_b rotates by
    −θ × ratio about axis_b (sign encodes external mesh reversal).

    Attributes:
        unit_a_index: Index of the first gear's unit in PhysicsScene.units.
        unit_b_index: Index of the second gear's unit.
        ratio:        Gear ratio (teeth_a / teeth_b).
        axis_a:       Rotation axis of gear A in world space (unit vector).
        axis_b:       Rotation axis of gear B in world space (unit vector).
        position:     Mesh point in world space (metres).
    """

    unit_a_index: int
    unit_b_index: int
    ratio: float
    axis_a: np.ndarray
    axis_b: np.ndarray
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class PhysicsScene:
    """Complete physics scene ready for hand-off to a simulator.

    Attributes:
        units:   All rigid bodies in the scene.
        joints:  Constraints between units.
        motors:  Active motor drives.
        gravity: Gravity acceleration vector in m/s² (world space).
                 Default is ``[0, -9.81, 0]`` (LDraw Y-down convention,
                 i.e. the floor is at high Y values; gravity pulls toward
                 lower Y, which is upward in LDraw space – callers should
                 adjust this to match their build orientation).
    """

    units: List[Unit] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)
    motors: List[Motor] = field(default_factory=list)
    gears: List[GearConstraint] = field(default_factory=list)
    gravity: np.ndarray = field(
        default_factory=lambda: np.array([0.0, -9.81, 0.0])
    )
