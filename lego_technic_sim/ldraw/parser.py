"""LDraw file parser.

Supports:
  * ``.dat`` – part / primitive definition files.
  * ``.ldr`` – simple model files (a flat list of placed parts).
  * ``.mpd`` – multi-part documents (multiple sub-models in one file).

LDraw line types
----------------
  0  Comment or META command  (ignored, except BFC winding directives)
  1  Sub-file reference       → placed part or sub-model
  2  Line segment             (ignored – decorative only)
  3  Triangle                 → mesh face
  4  Quadrilateral            → split into two triangles
  5  Optional line            (ignored – decorative only)

Reference
---------
https://www.ldraw.org/article/218.html
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .model import LDrawBuild, LDrawPart, Triangle


class LDrawParser:
    """Parse LDraw ``.ldr`` / ``.dat`` files into :class:`LDrawBuild` objects.

    Args:
        parts_dir: Optional path to an LDraw parts library root.  When set,
            sub-file references are resolved from the library's ``parts/`` and
            ``p/`` sub-directories in addition to the file's own directory.
    """

    def __init__(self, parts_dir: Optional[Path] = None) -> None:
        self.parts_dir = Path(parts_dir) if parts_dir is not None else None
        # Cache parsed triangles keyed by resolved (lower-case) file path.
        self._cache: Dict[str, List[Triangle]] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def parse_build(self, path: Path) -> LDrawBuild:
        """Parse an LDraw model file and return an :class:`LDrawBuild`.

        The returned build contains one :class:`LDrawPart` per ``type-1`` line
        found in the model file.  Each part's ``triangles`` list holds the mesh
        already transformed into the build's coordinate frame.

        Args:
            path: Path to the ``.ldr`` or ``.mpd`` file.

        Returns:
            The parsed build.
        """
        path = Path(path)
        text = path.read_text(encoding="utf-8", errors="replace")
        build = LDrawBuild(name=path.stem)
        identity = np.eye(4)
        for part in self._parse_parts(text.splitlines(), identity, path.parent):
            build.parts.append(part)
        return build

    def parse_part(self, path: Path) -> List[Triangle]:
        """Parse an LDraw part file and return its triangulated mesh.

        The mesh is in the part's *local* coordinate frame (identity transform).

        Args:
            path: Path to the ``.dat`` file.

        Returns:
            List of :class:`Triangle` objects.
        """
        path = Path(path)
        key = str(path.resolve()).lower()
        if key not in self._cache:
            text = path.read_text(encoding="utf-8", errors="replace")
            self._cache[key] = self._parse_triangles(
                text.splitlines(), np.eye(4), path.parent
            )
        return self._cache[key]

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _parse_parts(
        self,
        lines: List[str],
        parent_transform: np.ndarray,
        base_dir: Path,
    ) -> List[LDrawPart]:
        """Collect all top-level ``type-1`` lines as :class:`LDrawPart` objects."""
        parts: List[LDrawPart] = []
        for line in lines:
            tokens = line.strip().split()
            if not tokens or tokens[0] != "1":
                continue
            if len(tokens) < 15:
                continue
            color, local_t, part_file = self._parse_type1(tokens)
            world_t = parent_transform @ local_t
            raw_tris = self._load_part_triangles(part_file, base_dir)
            world_tris = [tri.transformed(world_t) for tri in raw_tris]
            parts.append(
                LDrawPart(
                    part_id=part_file,
                    color=color,
                    transform=world_t,
                    triangles=world_tris,
                )
            )
        return parts

    def _parse_triangles(
        self,
        lines: List[str],
        parent_transform: np.ndarray,
        base_dir: Path,
    ) -> List[Triangle]:
        """Recursively parse triangle primitives from an LDraw file."""
        triangles: List[Triangle] = []
        for line in lines:
            tokens = line.strip().split()
            if not tokens:
                continue
            line_type = tokens[0]

            if line_type == "1":
                if len(tokens) < 15:
                    continue
                _, local_t, sub_file = self._parse_type1(tokens)
                combined = parent_transform @ local_t
                sub_tris = self._load_part_triangles(sub_file, base_dir)
                triangles.extend(tri.transformed(combined) for tri in sub_tris)

            elif line_type == "3":
                # 3 colour x1 y1 z1 x2 y2 z2 x3 y3 z3
                if len(tokens) < 11:
                    continue
                color = int(tokens[1])
                v0 = np.array(tokens[2:5], dtype=float)
                v1 = np.array(tokens[5:8], dtype=float)
                v2 = np.array(tokens[8:11], dtype=float)
                triangles.append(
                    Triangle(v0, v1, v2, color).transformed(parent_transform)
                )

            elif line_type == "4":
                # 4 colour x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4
                if len(tokens) < 14:
                    continue
                color = int(tokens[1])
                v0 = np.array(tokens[2:5], dtype=float)
                v1 = np.array(tokens[5:8], dtype=float)
                v2 = np.array(tokens[8:11], dtype=float)
                v3 = np.array(tokens[11:14], dtype=float)
                triangles.append(
                    Triangle(v0, v1, v2, color).transformed(parent_transform)
                )
                triangles.append(
                    Triangle(v0, v2, v3, color).transformed(parent_transform)
                )

        return triangles

    @staticmethod
    def _parse_type1(
        tokens: List[str],
    ) -> Tuple[int, np.ndarray, str]:
        """Parse a type-1 token list into (colour, 4×4 matrix, filename)."""
        color = int(tokens[1])
        x, y, z = float(tokens[2]), float(tokens[3]), float(tokens[4])
        a, b, c = float(tokens[5]), float(tokens[6]), float(tokens[7])
        d, e, f = float(tokens[8]), float(tokens[9]), float(tokens[10])
        g, h, i = float(tokens[11]), float(tokens[12]), float(tokens[13])
        part_file = " ".join(tokens[14:])
        matrix = np.array(
            [
                [a, b, c, x],
                [d, e, f, y],
                [g, h, i, z],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return color, matrix, part_file

    def _load_part_triangles(self, part_file: str, base_dir: Path) -> List[Triangle]:
        """Load and cache the triangles for *part_file*.

        Searches *base_dir* first, then the configured LDraw library.
        Returns an empty list if the file cannot be found (non-fatal).
        """
        cache_key = part_file.lower()
        if cache_key in self._cache:
            return self._cache[cache_key]

        search_dirs: List[Path] = [base_dir]
        if self.parts_dir is not None:
            search_dirs += [
                self.parts_dir,
                self.parts_dir / "parts",
                self.parts_dir / "p",
                self.parts_dir / "parts" / "s",
            ]

        for directory in search_dirs:
            candidate = directory / part_file
            if candidate.exists():
                text = candidate.read_text(encoding="utf-8", errors="replace")
                tris = self._parse_triangles(text.splitlines(), np.eye(4), candidate.parent)
                self._cache[cache_key] = tris
                return tris
            # Case-insensitive fallback (useful on case-sensitive file-systems)
            try:
                for entry in directory.iterdir():
                    if entry.name.lower() == part_file.lower():
                        text = entry.read_text(encoding="utf-8", errors="replace")
                        tris = self._parse_triangles(
                            text.splitlines(), np.eye(4), entry.parent
                        )
                        self._cache[cache_key] = tris
                        return tris
            except (OSError, NotADirectoryError):
                pass

        # Part not found – return empty mesh but keep entry to avoid re-scanning.
        self._cache[cache_key] = []
        return []
