"""LDraw data model.

An LDraw *build* (.ldr / .mpd) is a collection of placed *parts*.  Each part
carries its colour, a 4 × 4 homogeneous transformation matrix, and the
triangulated mesh that represents its geometry.

LDraw coordinate system (all distances in LDraw Units, 1 LDU = 0.4 mm):
  • X – rightward
  • Y – downward  (positive Y is toward the ground)
  • Z – toward the viewer
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class Triangle:
    """A single triangle in a mesh, given by three 3-D vertices.

    Vertices are stored in LDraw Units (LDU).  The winding order follows the
    LDraw BFC convention: counter-clockwise when viewed from outside the solid
    (outward normals, right-hand rule).
    """

    v0: np.ndarray  # shape (3,)
    v1: np.ndarray  # shape (3,)
    v2: np.ndarray  # shape (3,)
    color: Optional[int] = None

    def transformed(self, matrix: np.ndarray) -> "Triangle":
        """Return a new Triangle transformed by a 4 × 4 homogeneous matrix."""

        def _apply(v: np.ndarray) -> np.ndarray:
            h = np.append(v, 1.0)
            return (matrix @ h)[:3]

        return Triangle(_apply(self.v0), _apply(self.v1), _apply(self.v2), self.color)


@dataclass
class LDrawPart:
    """An LDraw part (brick) placed in a build.

    Attributes:
        part_id:   File name of the part definition, e.g. ``"3001.dat"``.
        color:     LDraw colour code.
        transform: 4 × 4 homogeneous transformation matrix that places the
                   part in the build's coordinate frame.
        triangles: Triangulated mesh in the *world* (build) coordinate frame,
                   i.e. the mesh has already been transformed by *transform*.
    """

    part_id: str
    color: int
    transform: np.ndarray  # 4 × 4
    triangles: List[Triangle] = field(default_factory=list)

    @property
    def position(self) -> np.ndarray:
        """World position of the part origin (column 3 of the transform)."""
        return self.transform[:3, 3].copy()


@dataclass
class LDrawBuild:
    """A complete LDraw build loaded from a ``.ldr`` or ``.mpd`` file.

    Attributes:
        name:  Model name (typically the file stem).
        parts: All placed parts in the build.
    """

    name: str
    parts: List[LDrawPart] = field(default_factory=list)
