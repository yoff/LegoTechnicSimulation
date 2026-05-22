#!/usr/bin/env bash
# Generate three looping GIF animations of Walker1's leg cycle.
#
# Outputs:
#   walker1_cycle_side.gif  – orthographic side view
#   walker1_cycle_persp.gif – perspective side view (near legs larger)
#   walker1_cycle_front.gif – perspective from slightly in front and above
#
# Requirements:
#   - pip install -e . (this project)
#   - pip install Pillow
#   - LDraw library at /opt/ldraw/ldraw (or set LDRAW_LIBRARY env var)
#   - Blender at BLENDER_PATH
#   - gifsicle (apt install gifsicle) for optimisation

set -euo pipefail

LDRAW_LIBRARY="${LDRAW_LIBRARY:-/opt/ldraw/ldraw}"
BLENDER_PATH="${BLENDER_PATH:-/home/codespace/apps/blender/blender-4.1.0-linux-x64/blender}"
MODEL="${MODEL:-sample_models/Walker1/Walker1.ldr}"
FRAMES_DIR="/tmp/walker_gif_frames"
BASE_SCRIPT="/tmp/walker1_gif_base.py"

# One full crank rotation at 60fps ≈ 96 frames (motor 23.04 rad/s, ratio 0.171)
SIM_FRAMES=96

echo "=== Generating base Blender script (${SIM_FRAMES} frames) ==="
python -m lego_technic_sim.cli \
    "${MODEL}" "${BASE_SCRIPT}" \
    --ldraw-library "${LDRAW_LIBRARY}" \
    --simulate --anchor-motor --sim-frames "${SIM_FRAMES}"

# --- Helper: patch script and render ---
render_variant() {
    local name="$1"
    local camera_code="$2"
    local script="/tmp/walker1_gif_${name}.py"

    echo ""
    echo "=== Rendering variant: ${name} ==="

    python3 -c "
import sys

with open('${BASE_SCRIPT}') as f:
    script = f.read()

# Resolution 150x150
script = script.replace('scene.render.resolution_x = 1280', 'scene.render.resolution_x = 150')
script = script.replace('scene.render.resolution_y = 720', 'scene.render.resolution_y = 150')

# PNG frames with transparency
script = script.replace(
    \"scene.render.filepath = '/tmp/simulation_'\nscene.render.image_settings.file_format = 'FFMPEG'\nscene.render.ffmpeg.format = 'MPEG4'\nscene.render.ffmpeg.codec = 'H264'\",
    \"scene.render.filepath = '${FRAMES_DIR}/frame_'\nscene.render.image_settings.file_format = 'PNG'\nscene.render.image_settings.color_mode = 'RGBA'\nscene.render.film_transparent = True\"
)

# 128 Cycles samples
script = script.replace('scene.cycles.samples = 32', 'scene.cycles.samples = 128')

# Camera replacement
old_camera = '''# ── Camera ─────────────────────────────────────────────────
bpy.ops.object.camera_add(location=(0.450153, -0.609804, 0.388199))
_cam = bpy.context.active_object
scene.camera = _cam
_cam.data.clip_start = 0.001
_cam.data.clip_end = 100.0
_target = mathutils.Vector((-0.007200, 0.000000, 0.007072))
_direction = _target - _cam.location
_cam.rotation_euler = _direction.to_track_quat(\"-Z\", \"Y\").to_euler()'''

new_camera = '''$camera_code'''

script = script.replace(old_camera, new_camera)

# Ensure output dir
script = script.replace(
    \"print(f'Rendering {scene.frame_end} frames...')\",
    \"import os\nos.makedirs('${FRAMES_DIR}', exist_ok=True)\nprint(f'Rendering {scene.frame_end} frames...')\"
)

with open('${script}', 'w') as f:
    f.write(script)
"

    rm -rf "${FRAMES_DIR}"
    mkdir -p "${FRAMES_DIR}"
    "${BLENDER_PATH}" --background --python "${script}" 2>&1 | grep -E "Rendering|Simulation|Error" | tail -5

    echo "  Converting to GIF..."
    python3 -c "
from PIL import Image
import glob, os

paths = sorted(glob.glob('${FRAMES_DIR}/frame_*.png'))
frames = []
for i, p in enumerate(paths):
    if i % 2 == 0:  # 30fps
        img = Image.open(p).convert('RGBA')
        bg = Image.new('RGBA', img.size, (255, 255, 255, 255))
        composite = Image.alpha_composite(bg, img)
        frames.append(composite.convert('P', palette=Image.ADAPTIVE, colors=192))

loop_frames = frames[:-1]  # skip last for seamless loop
loop_frames[0].save(
    'walker1_cycle_${name}.gif',
    save_all=True,
    append_images=loop_frames[1:],
    duration=33,
    loop=0,
    optimize=True
)
print(f'  Raw: {os.path.getsize(\"walker1_cycle_${name}.gif\")//1024} KB, {len(loop_frames)} frames')
"

    if command -v gifsicle &>/dev/null; then
        gifsicle -O3 --colors 192 --lossy=40 "walker1_cycle_${name}.gif" \
            -o "walker1_cycle_${name}.gif"
    fi
    echo "  Final: $(du -h "walker1_cycle_${name}.gif" | cut -f1)"
}

# --- Variant 1: Orthographic side view ---
render_variant "side" '# ── Camera (orthographic side-view) ────────────────────────
bpy.ops.object.camera_add(location=(-0.007, -0.5, 0.000))
_cam = bpy.context.active_object
scene.camera = _cam
_cam.data.type = '"'"'ORTHO'"'"'
_cam.data.ortho_scale = 0.17
_cam.data.clip_start = 0.001
_cam.data.clip_end = 100.0
_cam.rotation_euler = (1.5708, 0, 0)'

# --- Variant 2: Perspective side view ---
render_variant "persp" '# ── Camera (perspective side-view) ─────────────────────────
bpy.ops.object.camera_add(location=(-0.007, -0.35, 0.000))
_cam = bpy.context.active_object
scene.camera = _cam
_cam.data.type = '"'"'PERSP'"'"'
_cam.data.lens = 85
_cam.data.clip_start = 0.001
_cam.data.clip_end = 100.0
_cam.rotation_euler = (1.5708, 0, 0)'

# --- Variant 3: Front-above perspective ---
render_variant "front" '# ── Camera (front-above perspective) ───────────────────────
bpy.ops.object.camera_add(location=(0.10, -0.20, 0.06))
_cam = bpy.context.active_object
scene.camera = _cam
_cam.data.type = '"'"'PERSP'"'"'
_cam.data.lens = 65
_cam.data.clip_start = 0.001
_cam.data.clip_end = 100.0
_target = mathutils.Vector((-0.007, 0.0, -0.005))
_direction = _target - _cam.location
_cam.rotation_euler = _direction.to_track_quat('"'"'-Z'"'"', '"'"'Y'"'"').to_euler()'

echo ""
echo "=== All done ==="
ls -lh walker1_cycle_*.gif
