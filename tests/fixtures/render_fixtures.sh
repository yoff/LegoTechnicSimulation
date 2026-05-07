#!/usr/bin/env bash
# Render assembly thumbnails for all test fixture .ldr files.
# Usage: ./tests/fixtures/render_fixtures.sh [--ldraw-library PATH]
#
# Produces a .png next to each .ldr file.
# Requires Blender and the lego_technic_sim package installed.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LDRAW="${LDRAW_LIBRARY:-/opt/ldraw/ldraw}"
BLENDER="${BLENDER_BIN:-blender}"

# Accept --ldraw-library override
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ldraw-library) LDRAW="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

for ldr in "$SCRIPT_DIR"/*.ldr; do
    name="$(basename "$ldr" .ldr)"
    out_png="$SCRIPT_DIR/${name}.png"
    tmp_script="/tmp/fixture_${name}.py"

    echo "=== Rendering $name ==="

    # Generate assembly animation script (single frame, low quality)
    python -c "
from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.blender.assembly_animation import generate_assembly_animation
from lego_technic_sim.physics.unit_builder import build_units_and_joints

parser = LDrawParser(parts_dir='${LDRAW}')
build = parser.parse_build('${ldr}')
scene = build_units_and_joints(build)
generate_assembly_animation(
    scene,
    output_path='${tmp_script}',
    frames_per_unit=1,
    hold_frames=1,
    render_output='/tmp/fixture_${name}_',
    resolution_x=640,
    resolution_y=480,
    render_format='PNG',
    cycles_samples=8,
)
print(f'{len(scene.units)} units, {len(scene.joints)} joints, {len(scene.gears)} gears, {len(scene.motors)} motors')
"

    # Render last frame only (all units visible)
    "$BLENDER" --background --python "$tmp_script" 2>&1 | tail -2

    # Find the rendered PNG and copy it
    last_png=$(ls /tmp/fixture_${name}_*.png 2>/dev/null | sort | tail -1)
    if [[ -n "$last_png" ]]; then
        cp "$last_png" "$out_png"
        echo "  -> $out_png"
    else
        echo "  WARNING: no render output found"
    fi

    # Clean up temp files
    rm -f /tmp/fixture_${name}_*.png "$tmp_script"
done

echo "=== Done ==="
