#!/usr/bin/env bash
# Generate and render a Walker1 assembly animation.
# Usage: ./render_assembly.sh [--fast]
#
# Requirements:
#   - pip install -e . (this project)
#   - LDraw library at /opt/ldraw/ldraw (or set LDRAW_LIBRARY env var)
#   - Blender at BLENDER_PATH (default: blender on PATH)

set -euo pipefail

LDRAW_LIBRARY="${LDRAW_LIBRARY:-/opt/ldraw/ldraw}"
BLENDER_PATH="${BLENDER_PATH:-/home/codespace/apps/blender/blender-4.1.0-linux-x64/blender}"
MODEL="${MODEL:-sample_models/Walker1/Walker1.ldr}"
SCRIPT="/tmp/walker1_assembly_anim.py"
OUTPUT="/tmp/walker1_assembly.mp4"

CLI_ARGS=("--ldraw-library" "${LDRAW_LIBRARY}" "--assembly")
if [[ "${1:-}" == "--fast" ]]; then
    CLI_ARGS+=("--fast")
fi
CLI_ARGS+=("${MODEL}" "${SCRIPT}")

echo "=== Generating Blender script ==="
python -m lego_technic_sim.cli "${CLI_ARGS[@]}"

echo ""
echo "=== Rendering with Blender ==="
rm -f /tmp/assembly_*.mp4
"${BLENDER_PATH}" --background --python "${SCRIPT}" 2>&1 | \
    grep -E "Fra:|Saved|Error" | tail -20

# Find the rendered file
RENDERED=$(ls -t /tmp/assembly_*.mp4 2>/dev/null | head -1)
if [[ -n "${RENDERED}" && -s "${RENDERED}" ]]; then
    cp "${RENDERED}" "${OUTPUT}"
    echo ""
    echo "=== Done ==="
    echo "Animation: ${OUTPUT} ($(du -h "${OUTPUT}" | cut -f1))"
else
    echo ""
    echo "ERROR: No rendered output found."
    exit 1
fi
