"""Drive train tree: traverse gear meshes from a motor/crank root.

Builds a tree of gear-connected units starting from the motor unit (or a
designated crank input). Each node carries the accumulated gear ratio from
the root, enabling downstream animation of correct rotation speeds.

If no motor or crank is present in the scene, `build_drive_train` returns
None – callers should bail gracefully.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from .model import GearConstraint, PhysicsScene


# Known crank/hand-input part IDs (these serve as manual drive root)
CRANK_PART_IDS: Set[str] = {
    "32551.dat",   # Technic Crank / Rotation Joint Disk
    "14720.dat",   # Technic Turntable Large Type 3
    "99010.dat",   # Technic Knob Cog / Gear 4 Tooth (hand crank)
}


@dataclass
class DriveNode:
    """A node in the drive train tree.

    Attributes:
        unit_index:      Index into PhysicsScene.units.
        parent_index:    Parent unit index (None for root).
        gear_constraint: The GearConstraint linking this node to its parent.
        accumulated_ratio: Product of gear ratios from root to this node.
        axis:            Rotation axis of this node's gear (world space).
        depth:           Tree depth (0 for root).
        children:        Child nodes driven by this node.
    """

    unit_index: int
    parent_index: Optional[int] = None
    gear_constraint: Optional[GearConstraint] = None
    accumulated_ratio: float = 1.0
    axis: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    depth: int = 0
    children: List["DriveNode"] = field(default_factory=list)


@dataclass
class DriveTree:
    """The complete drive train tree from motor/crank root.

    Attributes:
        root:       Root node (motor or crank unit).
        all_nodes:  Flat list of all nodes in BFS order.
        scene:      Reference to the physics scene.
    """

    root: DriveNode
    all_nodes: List[DriveNode] = field(default_factory=list)
    scene: Optional[PhysicsScene] = None


def _find_root_unit(scene: PhysicsScene) -> Optional[int]:
    """Find the drive root: the first gear on the motor output shaft.

    Among gear-participating units that have a revolute joint to the motor
    body, picks the leaf in the gear-mesh graph (only 1 gear connection)
    with the fewest bricks (smallest gear).  Falls back to motor joint's
    unit_b or crank unit.

    Returns the unit index, or None if no motor/crank found.
    """
    from .motor_detection import is_motor_part
    from .model import JointType

    if scene.motors:
        motor = scene.motors[0]
        motor_joint = scene.joints[motor.joint_index]
        motor_body = motor_joint.unit_a_index

        # Build gear-mesh degree map
        gear_degree: Dict[int, int] = {}
        for gc in scene.gears:
            gear_degree[gc.unit_a_index] = gear_degree.get(gc.unit_a_index, 0) + 1
            gear_degree[gc.unit_b_index] = gear_degree.get(gc.unit_b_index, 0) + 1

        # Find gear units connected to motor body via revolute joint
        candidates = []
        for j in scene.joints:
            if j.joint_type != JointType.REVOLUTE:
                continue
            if j.unit_a_index == motor_body and j.unit_b_index in gear_degree:
                candidates.append(j.unit_b_index)
            elif j.unit_b_index == motor_body and j.unit_a_index in gear_degree:
                candidates.append(j.unit_a_index)

        if candidates:
            # Prefer leaf nodes (degree 1) in the gear mesh graph
            leaves = [u for u in candidates if gear_degree.get(u, 0) == 1]
            if leaves:
                # Among leaves, pick the smallest gear (fewest bricks)
                leaves.sort(key=lambda u: len(scene.units[u].bricks))
                return leaves[0]
            # No leaves — pick candidate with fewest bricks
            candidates.sort(key=lambda u: len(scene.units[u].bricks))
            return candidates[0]

        # Fallback: motor joint's unit_b
        return motor_joint.unit_b_index

    # Look for a unit containing a motor part (may drive via gear mesh)
    for unit_idx, unit in enumerate(scene.units):
        for brick in unit.bricks:
            if is_motor_part(brick.part_id):
                return unit_idx

    # Fall back to crank parts
    for unit_idx, unit in enumerate(scene.units):
        for brick in unit.bricks:
            if brick.part_id.lower() in CRANK_PART_IDS:
                return unit_idx

    return None


def build_drive_train(scene: PhysicsScene) -> Optional[DriveTree]:
    """Build the drive train tree from motor/crank through gear meshes.

    Returns None if no motor or crank is present (caller should bail).
    """
    root_unit = _find_root_unit(scene)
    if root_unit is None:
        return None

    if not scene.gears:
        return None

    # Build adjacency from gear constraints
    # Each gear connects two units; store both directions
    adjacency: Dict[int, List[Tuple[int, GearConstraint]]] = {}
    for gc in scene.gears:
        adjacency.setdefault(gc.unit_a_index, []).append((gc.unit_b_index, gc))
        adjacency.setdefault(gc.unit_b_index, []).append((gc.unit_a_index, gc))

    # BFS from root
    root_node = DriveNode(
        unit_index=root_unit,
        accumulated_ratio=1.0,
        axis=_get_gear_axis(scene, root_unit),
    )

    visited: Set[int] = {root_unit}
    queue: deque[DriveNode] = deque([root_node])
    all_nodes: List[DriveNode] = [root_node]

    while queue:
        node = queue.popleft()
        for neighbor_idx, gc in adjacency.get(node.unit_index, []):
            if neighbor_idx in visited:
                continue
            visited.add(neighbor_idx)

            # Compute ratio: if node is unit_a in constraint, ratio is teeth_a/teeth_b
            # Direction matters: driving from A→B means B spins at speed/ratio
            if gc.unit_a_index == node.unit_index:
                local_ratio = gc.ratio  # teeth_a / teeth_b
                child_axis = gc.axis_b
            else:
                local_ratio = 1.0 / gc.ratio  # teeth_b / teeth_a
                child_axis = gc.axis_a

            child = DriveNode(
                unit_index=neighbor_idx,
                parent_index=node.unit_index,
                gear_constraint=gc,
                accumulated_ratio=node.accumulated_ratio * local_ratio,
                axis=child_axis,
                depth=node.depth + 1,
            )
            node.children.append(child)
            all_nodes.append(child)
            queue.append(child)

    tree = DriveTree(root=root_node, all_nodes=all_nodes, scene=scene)
    return tree


def _get_gear_axis(scene: PhysicsScene, unit_index: int) -> np.ndarray:
    """Get the rotation axis of the first gear in a unit."""
    from .gears import _gear_info

    unit = scene.units[unit_index]
    for brick in unit.bricks:
        info = _gear_info(brick)
        if info is not None:
            _, _, _, axis = info
            return axis
    # Default: use first joint axis connected to this unit
    for joint in scene.joints:
        if joint.unit_a_index == unit_index or joint.unit_b_index == unit_index:
            return joint.axis
    return np.array([1.0, 0.0, 0.0])
