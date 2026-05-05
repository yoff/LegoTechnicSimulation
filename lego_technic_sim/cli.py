"""Command-line interface for lego-technic-sim.

Usage::

    lego-technic-sim INPUT_MODEL OUTPUT_SCRIPT [--ldraw-library PATH]
    lego-technic-sim INPUT_MODEL OUTPUT_SCRIPT --ldraw-library PATH --assembly

Example::

    lego-technic-sim sample_models/Walker1/Walker1.ldr \\
                     sample_models/Walker1/simulation.py \\
                     --ldraw-library /path/to/ldraw

    lego-technic-sim sample_models/Walker1/Walker1.ldr \\
                     sample_models/Walker1/assembly.py \\
                     --ldraw-library /path/to/ldraw --assembly
"""

from __future__ import annotations

import argparse
from pathlib import Path


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="lego-technic-sim",
        description=(
            "Parse an LDraw model, build rigid units and joints, "
            "and generate a Blender Python simulation script."
        ),
    )
    p.add_argument(
        "input_model",
        type=Path,
        help="Path to the input .ldr model file.",
    )
    p.add_argument(
        "output_script",
        type=Path,
        help="Destination path for the generated Blender Python script.",
    )
    p.add_argument(
        "--ldraw-library",
        type=Path,
        default=None,
        metavar="PATH",
        help=(
            "Path to a local LDraw parts library root.  "
            "When provided, the parser also searches PATH/parts, PATH/p, "
            "and PATH/parts/s for sub-file references."
        ),
    )
    p.add_argument(
        "--assembly",
        action="store_true",
        help=(
            "Generate an assembly animation script instead of a physics "
            "simulation.  Units appear one by one and the result is rendered."
        ),
    )
    p.add_argument(
        "--frames-per-unit",
        type=int,
        default=10,
        metavar="N",
        help="Frames between each unit appearing in assembly mode (default: 10).",
    )
    return p


def main(argv: list[str] | None = None) -> None:
    """Entry point for the ``lego-technic-sim`` command."""
    args = _build_parser().parse_args(argv)

    # Lazy imports so the module is importable even without heavy deps at
    # import time (useful for --help and for testing argument parsing).
    from lego_technic_sim.ldraw.parser import LDrawParser
    from lego_technic_sim.physics.unit_builder import build_units_and_joints

    parser = LDrawParser(parts_dir=args.ldraw_library)
    build = parser.parse_build(args.input_model)
    scene = build_units_and_joints(build)

    if args.assembly:
        from lego_technic_sim.blender.assembly_animation import (
            generate_assembly_animation,
        )

        generate_assembly_animation(
            scene,
            output_path=args.output_script,
            frames_per_unit=args.frames_per_unit,
        )
    else:
        from lego_technic_sim.blender.exporter import generate_blender_script

        generate_blender_script(scene, output_path=args.output_script)

    print(f"Parsed {len(build.parts)} parts")
    print(f"Built {len(scene.units)} rigid units")
    print(f"Detected {len(scene.joints)} joints")
    print(f"Detected {len(scene.motors)} motors")
    print(f"Blender script written to {args.output_script}")


if __name__ == "__main__":
    main()
