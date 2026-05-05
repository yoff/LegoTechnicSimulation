"""Tests for the lego-technic-sim CLI entrypoint."""

from __future__ import annotations

from pathlib import Path

import pytest

from lego_technic_sim.cli import _build_parser, main

FIXTURES = Path(__file__).parent / "fixtures"


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------


def test_parser_required_positionals():
    p = _build_parser()
    args = p.parse_args(["model.ldr", "out.py"])
    assert args.input_model == Path("model.ldr")
    assert args.output_script == Path("out.py")
    assert args.ldraw_library is None


def test_parser_ldraw_library_option():
    p = _build_parser()
    args = p.parse_args(["model.ldr", "out.py", "--ldraw-library", "/lib/ldraw"])
    assert args.ldraw_library == Path("/lib/ldraw")


def test_parser_missing_positionals_exits():
    p = _build_parser()
    with pytest.raises(SystemExit):
        p.parse_args([])


# ---------------------------------------------------------------------------
# End-to-end: run CLI on existing fixtures
# ---------------------------------------------------------------------------


def test_main_generates_script(tmp_path):
    """Running the CLI on a fixture model must create an output script."""
    output = tmp_path / "simulation.py"
    main(
        [
            str(FIXTURES / "two_bricks_adjacent.ldr"),
            str(output),
            "--ldraw-library",
            str(FIXTURES),
        ]
    )
    assert output.exists()
    content = output.read_text()
    assert "bpy" in content


def test_main_output_is_valid_python(tmp_path):
    """The generated script must be parseable as Python."""
    import ast

    output = tmp_path / "simulation.py"
    main(
        [
            str(FIXTURES / "two_bricks_adjacent.ldr"),
            str(output),
            "--ldraw-library",
            str(FIXTURES),
        ]
    )
    ast.parse(output.read_text())


def test_main_prints_summary(tmp_path, capsys):
    """The CLI must print a summary with part/unit/joint counts."""
    output = tmp_path / "simulation.py"
    main(
        [
            str(FIXTURES / "two_bricks_adjacent.ldr"),
            str(output),
            "--ldraw-library",
            str(FIXTURES),
        ]
    )
    captured = capsys.readouterr()
    assert "Parsed" in captured.out
    assert "rigid units" in captured.out
    assert "joints" in captured.out
    assert str(output) in captured.out


def test_main_without_ldraw_library(tmp_path):
    """CLI works without --ldraw-library when parts are beside the model."""
    import shutil

    model_dir = tmp_path / "model"
    model_dir.mkdir()
    # Copy fixture files so parts are resolvable from the model directory.
    shutil.copy(FIXTURES / "two_bricks_adjacent.ldr", model_dir / "two_bricks_adjacent.ldr")
    shutil.copy(FIXTURES / "cube.dat", model_dir / "cube.dat")

    output = tmp_path / "simulation.py"
    main([str(model_dir / "two_bricks_adjacent.ldr"), str(output)])
    assert output.exists()
