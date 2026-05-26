#!/usr/bin/env bash
# Generate three looping animations of Walker1's leg cycle.
#
# Outputs (per variant: side, persp, front):
#   walker1_cycle_<variant>.gif  – GIF (256 colors, universal markdown support)
#   walker1_cycle_<variant>.webm – WebM VP9 (much smaller, modern browsers)
#   walker1_cycle_<variant>.mp4  – MP4 H.264 (small, universal browser playback)
#
# Format selection (via FORMAT env var or --format flag):
#   FORMAT=all   – output GIF + WebM + MP4 (default)
#   FORMAT=gif   – GIF only (for GitHub README, universal but large)
#   FORMAT=webm  – WebM only (best size/quality, not in GitHub markdown)
#   FORMAT=mp4   – MP4 only (good size/quality, not in GitHub markdown)
#
# Trade-offs:
#   GIF:  ~300-400 KB, 256 colors, loops natively in <img>, GitHub markdown ✓
#   WebM: ~20-40 KB, full color, needs <video> tag, GitHub markdown ✗
#   MP4:  ~30-50 KB, full color, needs <video> tag, GitHub markdown ✗
#
# Requirements:
#   - pip install -e . (this project)
#   - pip install Pillow
#   - LDraw library at /opt/ldraw/ldraw (or set LDRAW_LIBRARY env var)
#   - Blender at BLENDER_PATH
#   - gifsicle (apt install gifsicle) for GIF optimisation
#   - ffmpeg for WebM/MP4 output

set -euo pipefail

# Parse --format flag if provided
for arg in "$@"; do
    case "$arg" in
        --format=*) FORMAT="${arg#*=}" ;;
    esac
done

FORMAT="${FORMAT:-all}"
LDRAW_LIBRARY="${LDRAW_LIBRARY:-/opt/ldraw/ldraw}"
BLENDER_PATH="${BLENDER_PATH:-/home/codespace/apps/blender/blender-4.1.0-linux-x64/blender}"
MODEL="${MODEL:-sample_models/Walker1/Walker1.ldr}"
FRAMES_DIR="/tmp/walker_gif_frames"
BASE_SCRIPT="/tmp/walker1_gif_base.py"

# One full crank rotation at 60fps ≈ 96 frames (motor 23.04 rad/s, ratio 0.171)
SIM_FRAMES=96

echo "Output format: ${FORMAT}"

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
    "${BLENDER_PATH}" --background --python "${script}" 2>&1 | grep -E "Rendering|Simulation|Error|Framing" | tail -10

    echo "  Converting frames to output format(s)..."

    # --- GIF output ---
    if [[ "${FORMAT}" == "all" || "${FORMAT}" == "gif" ]]; then
        python3 -c "
from PIL import Image
import glob, os

paths = sorted(glob.glob('${FRAMES_DIR}/frame_*.png'))
frames = []
# Build global palette from sampled frames
sample_indices = list(range(0, len(paths), max(1, len(paths) // 8)))
composite = Image.new('RGB', Image.open(paths[0]).size)
for si in sample_indices:
    img = Image.open(paths[si]).convert('RGBA')
    bg = Image.new('RGBA', img.size, (255, 255, 255, 255))
    composite.paste(Image.alpha_composite(bg, img))
global_palette = composite.quantize(colors=192, method=Image.Quantize.MEDIANCUT)

for i, p in enumerate(paths):
    if i % 2 == 0:  # 30fps
        img = Image.open(p).convert('RGBA')
        bg = Image.new('RGBA', img.size, (255, 255, 255, 255))
        rgb = Image.alpha_composite(bg, img).convert('RGB')
        frames.append(rgb.quantize(palette=global_palette))

loop_frames = frames[:-1]  # skip last for seamless loop
loop_frames[0].save(
    'walker1_cycle_${name}.gif',
    save_all=True,
    append_images=loop_frames[1:],
    duration=33,
    loop=0,
    optimize=True
)
print(f'  GIF raw: {os.path.getsize(\"walker1_cycle_${name}.gif\")//1024} KB, {len(loop_frames)} frames')
"
        if command -v gifsicle &>/dev/null; then
            gifsicle -O3 --colors 192 --lossy=40 "walker1_cycle_${name}.gif" \
                -o "walker1_cycle_${name}.gif"
        fi
        echo "  GIF: $(du -h "walker1_cycle_${name}.gif" | cut -f1)"
    fi

    # --- WebM output (VP9, very small, looping, supports alpha) ---
    if [[ "${FORMAT}" == "all" || "${FORMAT}" == "webm" ]]; then
        ffmpeg -y -framerate 60 -start_number 1 -i "${FRAMES_DIR}/frame_%04d.png" \
            -vf "fps=30" -frames:v 47 \
            -c:v libvpx-vp9 -pix_fmt yuva420p -crf 30 -b:v 0 \
            -auto-alt-ref 0 -an \
            "walker1_cycle_${name}.webm" 2>/dev/null
        echo "  WebM: $(du -h "walker1_cycle_${name}.webm" | cut -f1)"
    fi

    # --- MP4 output (H.264, widely compatible, white background) ---
    if [[ "${FORMAT}" == "all" || "${FORMAT}" == "mp4" ]]; then
        ffmpeg -y -framerate 60 -start_number 1 -i "${FRAMES_DIR}/frame_%04d.png" \
            -vf "fps=30,format=yuva420p,colorchannelmixer=aa=1,pad=iw:ih:0:0:white,format=yuv420p" \
            -frames:v 47 \
            -c:v libx264 -crf 23 -preset veryslow -an \
            -movflags +faststart \
            "walker1_cycle_${name}.mp4" 2>/dev/null
        echo "  MP4: $(du -h "walker1_cycle_${name}.mp4" | cut -f1)"
    fi
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
ls -lh walker1_cycle_*.{gif,webm,mp4} 2>/dev/null || true
