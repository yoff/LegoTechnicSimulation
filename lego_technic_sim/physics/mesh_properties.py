"""Calculate physical properties of a triangulated mesh.

Theory
------
Volume and centre of mass are derived from the *divergence theorem* applied to
a closed, consistently-wound triangle mesh:

    V = (1/6) · Σ  v₀ · (v₁ × v₂)                   (signed sum)
    r_com · V = (1/24) · Σ  (v₀ + v₁ + v₂) · (v₀ · (v₁ × v₂))  (weighted)

The formula is exact for any closed polyhedron, regardless of where the
coordinate origin lies, because contributions from "outside" faces cancel.

This gives the volume and centre of mass of the *solid material* enclosed by
the mesh, which is a good approximation for LEGO bricks (thick-walled ABS
shells are well-approximated by a solid for the purposes of rigid-body sims).

Unit conventions
----------------
* Input vertices are in LDraw Units (LDU), where **1 LDU = 0.4 mm**.
* Output volume is in m³; output mass is in kg; output COM is in metres.
* Default material: ABS plastic, density ≈ 1050 kg/m³.
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np

from ..ldraw.model import Triangle

# 1 LDraw Unit = 0.4 mm = 0.0004 m
LDU_TO_METERS: float = 0.0004

# ABS plastic density (kg/m³)
ABS_DENSITY_KG_PER_M3: float = 1050.0


def _signed_tet_volume(tri: Triangle) -> float:
    """Signed volume of the tetrahedron (origin, v0, v1, v2).

    Sign encodes winding: positive when the face normal points away from the
    origin (outward), consistent with the divergence-theorem derivation.
    """
    return float(np.dot(tri.v0, np.cross(tri.v1, tri.v2))) / 6.0


def mesh_volume_and_com(
    triangles: List[Triangle],
    ldu_to_meters: float = LDU_TO_METERS,
) -> Tuple[float, np.ndarray]:
    """Compute volume (m³) and centre of mass (m) of a closed triangle mesh.

    The mesh must be closed (water-tight) and consistently wound (all face
    normals outward *or* all inward).  LDraw BFC-certified parts satisfy this.
    For non-BFC parts the magnitude of the volume is still correct; only the
    sign of the raw sum may be negative.

    Args:
        triangles:     List of :class:`~lego_technic_sim.ldraw.model.Triangle`
                       objects forming the mesh, in LDraw Units.
        ldu_to_meters: Scale factor from LDU to metres (default 0.0004).

    Returns:
        ``(volume_m3, com_m)`` where *com_m* is a ``(3,)`` numpy array.
        Both are zero for an empty mesh.
    """
    if not triangles:
        return 0.0, np.zeros(3)

    total_signed_vol: float = 0.0
    weighted_com = np.zeros(3, dtype=float)

    for tri in triangles:
        svol = _signed_tet_volume(tri)
        total_signed_vol += svol
        # Centroid of the tetrahedron (origin counts as 4th vertex at 0,0,0):
        # centroid = (v0 + v1 + v2 + 0) / 4
        tet_centroid = (tri.v0 + tri.v1 + tri.v2) / 4.0
        weighted_com += svol * tet_centroid

    if abs(total_signed_vol) < 1e-30:
        return 0.0, np.zeros(3)

    com_ldu = weighted_com / total_signed_vol
    volume_m3 = abs(total_signed_vol) * (ldu_to_meters**3)
    com_m = com_ldu * ldu_to_meters
    return volume_m3, com_m


def brick_mass(
    triangles: List[Triangle],
    density: float = ABS_DENSITY_KG_PER_M3,
    ldu_to_meters: float = LDU_TO_METERS,
) -> float:
    """Compute the mass (kg) of a brick from its triangulated mesh.

    Args:
        triangles:     Mesh triangles in LDraw Units.
        density:       Material density in kg/m³ (default: ABS plastic).
        ldu_to_meters: Scale factor from LDU to metres.

    Returns:
        Mass in kg.
    """
    volume_m3, _ = mesh_volume_and_com(triangles, ldu_to_meters)
    return density * volume_m3
