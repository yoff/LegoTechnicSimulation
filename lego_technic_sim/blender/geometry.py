"""Shared geometry helpers for Blender script generation.

Provides coordinate-system conversion and triangle collection used by both
the simulation exporter and the assembly-animation exporter.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

from ..physics.mesh_properties import LDU_TO_METERS


def ldraw_to_blender(v: np.ndarray) -> np.ndarray:
    """Convert a 3-D point from LDraw space to Blender space.

    LDraw: X right, Y down, Z toward viewer.
    Blender: X right, Y toward viewer, Z up.
    """
    return np.array([v[0], -v[2], -v[1]], dtype=float)


def collect_geometry(
    bricks: Sequence,
) -> Tuple[List[List[float]], List[List[int]]]:
    """Return (vertices, faces) for a set of bricks, in Blender coordinates.

    Each element of *bricks* must have a ``triangles`` attribute containing
    triangle objects with ``v0``, ``v1``, ``v2`` numpy-array attributes
    (in LDraw world space).

    Returns lists suitable for Blender's
    ``mesh.from_pydata(vertices, [], faces)``.
    """
    vertices: List[List[float]] = []
    faces: List[List[int]] = []
    vi = 0
    for brick in bricks:
        for tri in brick.triangles:
            v0 = ldraw_to_blender(tri.v0) * LDU_TO_METERS
            v1 = ldraw_to_blender(tri.v1) * LDU_TO_METERS
            v2 = ldraw_to_blender(tri.v2) * LDU_TO_METERS
            vertices.append(
                [round(float(v0[0]), 7), round(float(v0[1]), 7), round(float(v0[2]), 7)]
            )
            vertices.append(
                [round(float(v1[0]), 7), round(float(v1[1]), 7), round(float(v1[2]), 7)]
            )
            vertices.append(
                [round(float(v2[0]), 7), round(float(v2[1]), 7), round(float(v2[2]), 7)]
            )
            faces.append([vi, vi + 1, vi + 2])
            vi += 3
    return vertices, faces
