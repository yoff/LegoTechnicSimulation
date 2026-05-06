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

from ..physics.connection_ports import (
    ConnectionPort,
    extract_ports_from_lines,
    extract_ports_recursive,
    deduplicate_ports,
)


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
        # Cache parsed ports keyed by lower-case part filename.
        self._port_cache: Dict[str, List[ConnectionPort]] = {}
        # Inline sub-model definitions from MPD files (keyed by lower-case name).
        self._inline_models: Dict[str, List[str]] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def parse_build(self, path: Path) -> LDrawBuild:
        """Parse an LDraw model file and return an :class:`LDrawBuild`.

        The returned build contains one :class:`LDrawPart` per ``type-1`` line
        found in the model file.  Each part's ``triangles`` list holds the mesh
        already transformed into the build's coordinate frame.

        Handles MPD (multi-part document) files by splitting on ``0 FILE``
        directives and resolving inline sub-model references.

        Args:
            path: Path to the ``.ldr`` or ``.mpd`` file.

        Returns:
            The parsed build.
        """
        path = Path(path)
        text = path.read_text(encoding="utf-8", errors="replace")
        all_lines = text.splitlines()

        # Split MPD into sub-models; use first sub-model as main.
        models = self._split_mpd(all_lines)
        if models:
            main_name, main_lines = models[0]
            # Register all inline sub-models for resolution
            for name, lines in models[1:]:
                self._inline_models[name.lower()] = lines
        else:
            main_name = path.stem
            main_lines = all_lines

        build = LDrawBuild(name=main_name)
        identity = np.eye(4)
        for part in self._parse_parts(main_lines, identity, path.parent):
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

    @staticmethod
    def _split_mpd(lines: List[str]) -> List[Tuple[str, List[str]]]:
        """Split an MPD file into named sub-models.

        Returns a list of (name, lines) tuples.  If the file has no ``0 FILE``
        directives, returns an empty list (caller should treat the whole file
        as a single model).
        """
        models: List[Tuple[str, List[str]]] = []
        current_name: Optional[str] = None
        current_lines: List[str] = []

        for line in lines:
            stripped = line.strip()
            upper = stripped.upper()
            if upper.startswith("0 FILE ") or upper.startswith("0 FILE\t"):
                # Start of a new sub-model
                if current_name is not None:
                    models.append((current_name, current_lines))
                current_name = stripped[7:].strip()
                current_lines = []
            elif upper == "0 NOFILE":
                if current_name is not None:
                    models.append((current_name, current_lines))
                    current_name = None
                    current_lines = []
            else:
                current_lines.append(line)

        # Final sub-model
        if current_name is not None:
            models.append((current_name, current_lines))

        return models

    def _parse_parts(
        self,
        lines: List[str],
        parent_transform: np.ndarray,
        base_dir: Path,
    ) -> List[LDrawPart]:
        """Collect all top-level ``type-1`` lines as :class:`LDrawPart` objects.

        If a type-1 line references an inline sub-model (from an MPD file),
        recursively resolves its parts with the composed transform.
        """
        parts: List[LDrawPart] = []
        for line in lines:
            tokens = line.strip().split()
            if not tokens or tokens[0] != "1":
                continue
            if len(tokens) < 15:
                continue
            color, local_t, part_file = self._parse_type1(tokens)
            world_t = parent_transform @ local_t

            # Check if this references an inline sub-model
            inline_lines = self._inline_models.get(part_file.lower())
            if inline_lines is not None:
                # Recursively parse sub-model parts with composed transform
                for sub_part in self._parse_parts(inline_lines, world_t, base_dir):
                    parts.append(sub_part)
            else:
                raw_tris = self._load_part_triangles(part_file, base_dir)
                world_tris = [tri.transformed(world_t) for tri in raw_tris]
                # Load connection ports for this part
                local_ports = self._load_part_ports(part_file, base_dir)
                world_ports = [p.transformed(world_t) for p in local_ports]
                parts.append(
                    LDrawPart(
                        part_id=part_file,
                        color=color,
                        transform=world_t,
                        triangles=world_tris,
                        ports=world_ports,
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

                # Check inline models for triangle resolution too
                inline_lines = self._inline_models.get(sub_file.lower())
                if inline_lines is not None:
                    triangles.extend(
                        self._parse_triangles(inline_lines, combined, base_dir)
                    )
                else:
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

    def _load_part_ports(
        self, part_file: str, base_dir: Path
    ) -> List[ConnectionPort]:
        """Load and cache connection ports for *part_file*.

        Uses recursive extraction to follow sub-part references (e.g.,
        ``s/32316s01.dat``) and find hole primitives at any nesting depth.
        Returns deduplicated ports in the part's LOCAL coordinate frame.
        """
        cache_key = part_file.lower()
        if cache_key in self._port_cache:
            return self._port_cache[cache_key]

        search_dirs: List[Path] = [base_dir]
        if self.parts_dir is not None:
            search_dirs += [
                self.parts_dir,
                self.parts_dir / "parts",
                self.parts_dir / "p",
                self.parts_dir / "parts" / "s",
            ]

        def resolve_file(filename: str):
            """Resolve a sub-file reference to its lines."""
            # Normalize path separators
            filename_norm = filename.replace("\\", "/")
            for directory in search_dirs:
                candidate = directory / filename_norm
                if candidate.exists():
                    return candidate.read_text(
                        encoding="utf-8", errors="replace"
                    ).splitlines()
                # Try case-insensitive match
                try:
                    for entry in directory.iterdir():
                        if entry.name.lower() == Path(filename_norm).name.lower():
                            return entry.read_text(
                                encoding="utf-8", errors="replace"
                            ).splitlines()
                except (OSError, NotADirectoryError):
                    pass
            return None

        # Find and read the part file
        part_lines = resolve_file(part_file)
        if part_lines is not None:
            raw = extract_ports_recursive(part_lines, resolve_file)
            ports = deduplicate_ports(raw)
            self._port_cache[cache_key] = ports
            return ports

        self._port_cache[cache_key] = []
        return []
