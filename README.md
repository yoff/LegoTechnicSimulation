# LegoTechnicSimulation

Physical simulation of Lego Technic builds.

## What this repository does

This project parses LDraw models, groups connected parts into rigid units, detects likely joints between those units, and generates a Blender Python script for rigid-body simulation.

The target model file for the example below is:

- `https://yoff.github.io/lego-walker/Walker1/Walker1.ldr`

## Prerequisites

- Python 3.10+
- `git`
- Blender, if you want to run the generated simulation script
- An LDraw parts library on disk

This repository can parse an `.ldr` build file directly, but referenced parts must be available locally because the parser resolves sub-files from the model directory and an optional local LDraw library path.

## Setup

Clone the repository:

```bash
git clone https://github.com/yoff/LegoTechnicSimulation.git
cd LegoTechnicSimulation
```

Create and activate a virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate
```

Install dependencies:

```bash
pip install -r requirements.txt
pip install -e .
```

## Download the model file

Create a working directory and download the `.ldr` file:

```bash
mkdir -p sample_models/Walker1
curl -L https://yoff.github.io/lego-walker/Walker1/Walker1.ldr -o sample_models/Walker1/Walker1.ldr
```

## Get an LDraw parts library

You also need a local LDraw library containing `parts/`, `p/`, and related files.

For example, place it somewhere like:

```text
/path/to/ldraw
```

The parser searches the model directory first, then these directories under the configured library path:

- `/path/to/ldraw`
- `/path/to/ldraw/parts`
- `/path/to/ldraw/p`
- `/path/to/ldraw/parts/s`

## Run the tool on `Walker1.ldr`

There is not yet a checked-in CLI entrypoint in this repository, so the easiest way to run it is with a short Python script.

Create a file named `run_walker1.py`:

```python
from pathlib import Path

from lego_technic_sim.blender.exporter import generate_blender_script
from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.physics.unit_builder import build_units_and_joints

model_path = Path("sample_models/Walker1/Walker1.ldr")
ldraw_library = Path("/path/to/ldraw")
output_script = Path("sample_models/Walker1/simulation.py")

parser = LDrawParser(parts_dir=ldraw_library)
build = parser.parse_build(model_path)
scene = build_units_and_joints(build)
script = generate_blender_script(scene, output_path=output_script)

print(f"Parsed {len(build.parts)} parts")
print(f"Built {len(scene.units)} rigid units")
print(f"Detected {len(scene.joints)} joints")
print(f"Blender script written to {output_script}")
```

Run it:

```bash
python run_walker1.py
```

## Open the generated simulation in Blender

After running the script above, you should have:

- `sample_models/Walker1/simulation.py`

You can run that in Blender either interactively or from the command line.

### Option 1: inside Blender

Open Blender, go to the **Scripting** workspace, load `sample_models/Walker1/simulation.py`, and run it.

### Option 2: from the command line

```bash
blender --background --python sample_models/Walker1/simulation.py
```

## Notes and limitations

- The repository currently provides library functions, not a packaged command-line tool.
- The `.ldr` file alone is not enough; referenced LDraw part files must also exist locally.
- Joint detection is heuristic-based, so some Technic connections may need manual adjustment.
- The generated Blender script creates proxy rigid bodies and constraints for simulation setup.
