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

You can download it with the included setup script (see [Setup script](#setup-script) below), or
place one you already have somewhere like:

```text
/path/to/ldraw
```

> **Note:** When using the setup script with `--ldraw-dir /opt/ldraw`, the zip
> extracts into a subdirectory, so the actual library root ends up at
> `/opt/ldraw/ldraw`.  Use that full path when passing `--ldraw-library` to
> `lego-technic-sim`.

The parser searches the model directory first, then these directories under the configured library path:

- `/path/to/ldraw`
- `/path/to/ldraw/parts`
- `/path/to/ldraw/p`
- `/path/to/ldraw/parts/s`

## Setup script

`setup_env.py` downloads an LDraw parts library and/or a Blender build using
only standard-library modules (no extra dependencies required).

Download only the LDraw library:

```bash
python setup_env.py --ldraw-dir /opt/ldraw
```

Download only Blender (extracted into a local directory):

```bash
python setup_env.py --blender-dir ~/apps/blender
```

Download both at once:

```bash
python setup_env.py --ldraw-dir /opt/ldraw --blender-dir ~/apps/blender
```

Specify a different Blender version (default is 4.1.0):

```bash
python setup_env.py --blender-dir ~/apps/blender --blender-version 4.2.0
```

## Run the tool on `Walker1.ldr`

After installing the package (`pip install -e .`), a `lego-technic-sim` command
is available:

```bash
lego-technic-sim sample_models/Walker1/Walker1.ldr \
                 sample_models/Walker1/simulation.py \
                 --ldraw-library /opt/ldraw/ldraw
```

(Adjust the `--ldraw-library` path to wherever your LDraw library root is.)

The command prints a short summary on completion:

```
Parsed 42 parts
Built 12 rigid units
Detected 8 joints
Blender script written to sample_models/Walker1/simulation.py
```

## Open the generated simulation in Blender

After running the script above, you should have:

- `sample_models/Walker1/simulation.py`

You can run that in Blender either interactively or from the command line.

### Linux system dependencies

On a minimal Linux installation (e.g. a CI container or codespace), Blender
requires several shared libraries that may not be present by default:

```bash
sudo apt-get install -y libxxf86vm1 libxfixes3 libxi6 libxrender1 \
    libxkbcommon0 libsm6 libgl1 libepoxy0
```

### Option 1: inside Blender

Open Blender, go to the **Scripting** workspace, load `sample_models/Walker1/simulation.py`, and run it.

### Option 2: from the command line

```bash
blender --background --python sample_models/Walker1/simulation.py
```

## Notes and limitations

- The `.ldr` file alone is not enough; referenced LDraw part files must also exist locally.
- Joint detection is heuristic-based, so some Technic connections may need manual adjustment.
- The generated Blender script creates proxy rigid bodies and constraints for simulation setup.
