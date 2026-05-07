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
    p.add_argument(
        "--drivetrain",
        action="store_true",
        help=(
            "Generate a drive train animation showing gears spinning in "
            "sequence from the motor/crank root outward through the gear chain."
        ),
    )
    p.add_argument(
        "--simulate",
        action="store_true",
        help=(
            "Generate and render a rigid-body physics simulation with real "
            "meshes, joints, motors, ground plane, and gravity."
        ),
    )
    p.add_argument(
        "--sim-frames",
        type=int,
        default=120,
        metavar="N",
        help="Number of frames to simulate (default: 120, i.e. 2s at 60fps).",
    )
    p.add_argument(
        "--follow-unit",
        type=int,
        default=None,
        metavar="IDX",
        help="Camera follows the specified unit index during simulation.",
    )
    p.add_argument(
        "--follow-motor",
        nargs="?",
        const=0,
        type=int,
        default=None,
        metavar="IDX",
        help=(
            "Camera follows the motor's unit. Optional motor index "
            "(default: 0, i.e. first motor)."
        ),
    )
    p.add_argument(
        "--anchor-motor",
        action="store_true",
        help="Make motor units passive (fixed in space) for testing the drive train.",
    )
    p.add_argument(
        "--fast",
        action="store_true",
        help=(
            "Optimize for rendering speed: 1 frame per unit, low resolution "
            "(480×270), minimal Cycles samples (4), and short hold time.  "
            "Uses Cycles engine for headless compatibility."
        ),
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

    if args.drivetrain:
        from lego_technic_sim.physics.drive_train import build_drive_train
        from lego_technic_sim.blender.drivetrain_animation import (
            generate_drivetrain_animation,
        )

        tree = build_drive_train(scene)
        if tree is None:
            print("No motor or crank found – cannot build drive train.")
            print(f"Parsed {len(build.parts)} parts")
            print(f"Built {len(scene.units)} rigid units")
            print(f"Detected {len(scene.gears)} gear meshes")
            return

        kwargs = {}
        if args.fast:
            kwargs.update(
                resolution_x=480,
                resolution_y=270,
                cycles_samples=4,
                spin_frames=12,
                appear_frames=2,
            )

        generate_drivetrain_animation(
            tree,
            output_path=args.output_script,
            **kwargs,
        )
        print(f"Drive train: {len(tree.all_nodes)} gear units in chain")

    elif args.assembly:
        from lego_technic_sim.blender.assembly_animation import (
            generate_assembly_animation,
        )

        frames_per_unit = args.frames_per_unit
        kwargs = {}
        if args.fast:
            frames_per_unit = 1
            kwargs.update(
                hold_frames=5,
                resolution_x=480,
                resolution_y=270,
                cycles_samples=4,
            )

        generate_assembly_animation(
            scene,
            output_path=args.output_script,
            frames_per_unit=frames_per_unit,
            **kwargs,
        )
    elif args.simulate:
        from lego_technic_sim.blender.exporter import generate_blender_script

        sim_frames = args.sim_frames
        kwargs = dict(
            render=True,
            sim_frames=sim_frames,
        )
        if args.fast:
            kwargs.update(
                resolution_x=480,
                resolution_y=270,
                cycles_samples=4,
                sim_frames=min(sim_frames, 60),
            )

        # Determine follow target
        follow_unit = args.follow_unit
        if args.follow_motor is not None:
            motor_idx = args.follow_motor
            if motor_idx < len(scene.motors):
                motor = scene.motors[motor_idx]
                joint = scene.joints[motor.joint_index]
                follow_unit = joint.unit_a_index
            else:
                print(f"Warning: motor index {motor_idx} out of range, ignoring --follow-motor")

        if follow_unit is not None:
            kwargs["follow_unit"] = follow_unit

        if args.anchor_motor:
            kwargs["anchor_motor"] = True

        generate_blender_script(scene, output_path=args.output_script, **kwargs)

    else:
        from lego_technic_sim.blender.exporter import generate_blender_script

        generate_blender_script(scene, output_path=args.output_script)

    print(f"Parsed {len(build.parts)} parts")
    print(f"Built {len(scene.units)} rigid units")
    print(f"Detected {len(scene.joints)} joints")
    print(f"Detected {len(scene.gears)} gear meshes")
    print(f"Detected {len(scene.motors)} motors")
    print(f"Blender script written to {args.output_script}")


if __name__ == "__main__":
    main()
