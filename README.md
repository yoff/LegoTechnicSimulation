# LegoTechnicSimulation

Physical simulation of Lego Technic builds.

## What this repository does

This project parses LDraw (`.ldr`) models of Lego Technic builds, analyses
their mechanical structure, and generates Blender Python scripts for
visualisation and rigid-body physics simulation.

The pipeline:

1. **Parse** – reads an `.ldr` file and resolves all sub-part references from a
   local LDraw parts library.
2. **Build rigid units** – groups parts connected by friction pins, axles in
   axle holes, and other rigid connectors into single rigid bodies.
3. **Detect joints** – identifies revolute (hinge) joints where frictionless
   pins or axles in round holes connect two units.
4. **Detect motors** – recognises Technic motors (e.g. 58121.dat) and marks
   their output shafts as driven revolute joints.
5. **Detect gear meshes** – finds parallel and bevel gear pairs at correct
   centre distances and computes gear ratios.
6. **Build drive train** – traces the gear chain from the motor outward via BFS.
7. **Generate Blender script** – outputs a self-contained Python script that
   recreates the model in Blender with rigid-body physics, motor constraints,
   ground plane, camera, and lighting.

### Output modes

| Flag | Description |
|------|-------------|
| *(default)* | Static scene with rigid bodies and constraints |
| `--assembly` | Units appear one by one in an animated assembly sequence |
| `--drivetrain` | Gears spin in sequence from motor outward |
| `--simulate` | Full rigid-body physics simulation with gravity |

## Prerequisites

- Python 3.10+
- An LDraw parts library on disk (auto-detected or specified via `--ldraw-library`)
- [Blender](https://www.blender.org/) 4.1+ for running the generated scripts

## Quick start

```bash
git clone https://github.com/yoff/LegoTechnicSimulation.git
cd LegoTechnicSimulation
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt && pip install -e .
```

Download the LDraw library and (optionally) Blender:

```bash
python setup_env.py --ldraw-dir /opt/ldraw
python setup_env.py --blender-dir ~/apps/blender   # optional
```

Download a sample model:

```bash
mkdir -p sample_models/Walker1
curl -L https://yoff.github.io/lego-walker/Walker1/Walker1.ldr \
     -o sample_models/Walker1/Walker1.ldr
```

## Usage

### Basic analysis (static scene)

```bash
lego-technic-sim sample_models/Walker1/Walker1.ldr \
                 /tmp/walker1.py \
                 --ldraw-library /opt/ldraw/ldraw
```

Output:

```
Parsed 42 parts
Built 49 rigid units
Detected 63 joints
Detected 5 gear meshes
Detected 1 motors
Blender script written to /tmp/walker1.py
```

### Physics simulation

```bash
lego-technic-sim sample_models/Walker1/Walker1.ldr \
                 /tmp/walker1_sim.py \
                 --simulate --sim-frames 120
```

Add `--anchor-motor` to fix the motor in space (useful for drive-train
testing), `--follow-motor` to track the camera on the motor unit, or
`--follow-unit 3` to follow a specific unit.

### Assembly animation

```bash
lego-technic-sim sample_models/Walker1/Walker1.ldr \
                 /tmp/walker1_assembly.py \
                 --assembly --frames-per-unit 10
```

### Drive train animation

```bash
lego-technic-sim sample_models/Walker1/Walker1.ldr \
                 /tmp/walker1_drivetrain.py \
                 --drivetrain
```

### Fast rendering

Add `--fast` to any mode for quick preview renders (480×270, 4 Cycles
samples):

```bash
lego-technic-sim model.ldr /tmp/out.py --simulate --fast
```

### LDraw library auto-detection

If `--ldraw-library` is not provided, the tool searches these locations in
order:

1. `LDRAW_LIBRARY` environment variable
2. `/opt/ldraw/ldraw`
3. `/opt/ldraw`
4. `~/ldraw`
5. `~/LDraw`

### Running in Blender

```bash
blender --background --python /tmp/walker1_sim.py
```

Or open the script in Blender's **Scripting** workspace and run it
interactively.

On minimal Linux installations, Blender may need extra system libraries:

```bash
sudo apt-get install -y libxxf86vm1 libxfixes3 libxi6 libxrender1 \
    libxkbcommon0 libsm6 libgl1 libepoxy0
```

## CLI reference

| Argument | Description |
|----------|-------------|
| `input_model` | Path to the `.ldr` model file |
| `output_script` | Destination for the Blender Python script |
| `--ldraw-library PATH` | LDraw parts library root (auto-detected if omitted) |
| `--assembly` | Generate assembly animation |
| `--frames-per-unit N` | Frames between units in assembly mode (default: 10) |
| `--drivetrain` | Generate drive train animation |
| `--simulate` | Generate physics simulation |
| `--sim-frames N` | Simulation length in frames (default: 120) |
| `--follow-unit IDX` | Camera follows the specified unit |
| `--follow-motor [IDX]` | Camera follows a motor's unit (default: first motor) |
| `--anchor-motor` | Fix motor units in space (passive rigid body) |
| `--fast` | Low-resolution fast preview (480×270, 4 samples) |

## Setup script

`setup_env.py` downloads an LDraw parts library and/or a Blender build using
only standard-library modules.

```bash
python setup_env.py --ldraw-dir /opt/ldraw                    # LDraw only
python setup_env.py --blender-dir ~/apps/blender              # Blender only
python setup_env.py --ldraw-dir /opt/ldraw --blender-dir ~/apps/blender  # both
python setup_env.py --blender-dir ~/apps/blender --blender-version 4.2.0
```

> **Note:** The LDraw zip extracts into a subdirectory, so with `--ldraw-dir
> /opt/ldraw` the library root ends up at `/opt/ldraw/ldraw`.

## Test fixtures

Minimal `.ldr` files in `tests/fixtures/` serve as integration tests for the
physics pipeline.  See [`tests/fixtures/README.md`](tests/fixtures/README.md)
for descriptions and rendered thumbnails.

Run the test suite:

```bash
python -m pytest tests/ -v
```

## Project structure

```
lego_technic_sim/
  ldraw/          # LDraw file parsing and model representation
    parser.py     # .ldr parser with recursive sub-file resolution
    model.py      # LDrawBuild, LDrawPart, Triangle dataclasses
  physics/        # Mechanical analysis
    unit_builder.py    # Rigid unit grouping and joint detection
    connection_ports.py # Port extraction from LDraw primitives
    connectors.py      # Pin/axle classification
    gears.py           # Gear mesh detection and ratio computation
    drive_train.py     # BFS drive tree from motor through gears
    motor_detection.py # Motor and crank identification
    mesh_properties.py # Mass, volume, centre-of-mass computation
    model.py           # PhysicsScene, Unit, Joint, Motor dataclasses
  blender/        # Blender script generation
    exporter.py             # Physics simulation script generator
    assembly_animation.py   # Assembly animation generator
    drivetrain_animation.py # Drive train animation generator
  cli.py          # Command-line interface
tests/            # Test suite (pytest)
  fixtures/       # Minimal .ldr test models with renders
sample_models/    # Example Lego Technic models
```

## Notes and limitations

- Referenced LDraw part files must exist locally; the `.ldr` file alone is not
  sufficient.
- Port-based connection detection covers standard Technic pins, axles, and
  motor shafts.  Some exotic connectors may not be recognised.
- Gear mesh constraints are detected but not yet enforced as physics
  constraints in the Blender simulation (gear ratios are computed but not
  applied as coupled constraints).
- Rendering uses Cycles (CPU) for headless compatibility; EEVEE requires a GPU
  display.
