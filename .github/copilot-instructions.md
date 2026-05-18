# Copilot Instructions for LegoTechnicSimulation

## Project Overview

This project parses LDraw Technic models, detects rigid units and articulated
joints, builds gear mesh constraints, and generates Blender Python scripts for
assembly animations and physics simulations.

Key pipeline stages:
1. **Parse** LDraw `.ldr` / `.mpd` files into parts with geometry and ports
2. **Classify** parts as connectors (pins, axles) or structural (beams, gears, motors)
3. **Extract ports** from structural parts (recursive LDraw primitive matching)
4. **Match** connector shafts to structural ports → rigid or revolute connections
5. **Build units** (rigid groups via union-find) and **joints** (revolute links)
6. **Detect** gear meshes, motors, and drive trains
7. **Export** to Blender Python scripts (simulation or assembly animation)

## Keeping the README Up-to-Date

- When adding new CLI flags, update the usage examples in README.md.
- When changing the physics pipeline (new connection types, new port primitives),
  update the "Algorithm Overview" docstring in `unit_builder.py` and add a note
  to the README.
- When unit/joint/gear counts change for Walker1, update any "example output"
  sections so they match the current test expectations.

## Communicating via Animations

Animations are the primary debugging tool for this project.  The user cannot
run Blender locally — all visual feedback comes from rendered `.mp4` files. Animations annotated with unit/joint/part numbers, gear ratios etc. make it easy for the user to give precise feedback. Generated animations are copied into the workspace, so the use can view them in a codespace context.

### Assembly Animation (`--assembly`)

- Shows units appearing one-by-one with colour-coded geometry.
- Each unit (except 0) gets a **solo frame** showing it alone, then an
  **assembly frame** where it joins the growing build.
- Connectors (pins, axles) are included in their unit's geometry.
- A text stamp overlay shows the current unit number on each frame.
- Use `--fast --frames-per-unit 1` for quick debugging renders (480×270, 4 samples).

### Brick Inventory Animation

- One frame per part from the `.ldr` file with the part index displayed.
- Useful for identifying specific bricks by number when the user reports
  missing connections.

### Simulation Animation (`--simulate`)

- Physics simulation with rigid body constraints, motor, and gear coupling.
- Driven gears are set to kinematic; their rotation is keyframed after the
  physics bake.
- Use `--fast --sim-frames 48` for quick debugging renders.

### Best Practices

- Always use `--fast` when iterating.  Full renders are expensive.
- After structural changes (unit count, joint count), **always re-render the
  assembly animation** so the user can visually confirm correctness.
- When the user reports a problem by frame number or part number, cross-reference
  with the brick inventory or assembly animation.
- Keep rendered `.mp4` files in the repo root (gitignored) for easy access from
  the IDE.

## Creating and Maintaining Tests

### Test Architecture

There are three tiers of tests:

1. **Unit tests** (synthetic parts): `test_connector_joints.py`, `test_unit_builder.py`
   - Use `_make_part()` / `_make_connector()` helpers to construct artificial builds
   - Fast (<1s), no LDraw library needed
   - Good for testing individual rules (e.g. "friction pin → rigid")

2. **Port extraction tests** (real parts): `test_connection_ports.py`
   - Parse actual `.dat` files from `/opt/ldraw/ldraw`
   - Verify correct port types and counts for key parts
   - Catches: missing primitives, wrong primitive classification, subpart
     recursion failures
   - Skipped if LDraw library is not available

3. **Integration tests** (real models): `test_walker1_integration.py`, `test_fixtures.py`
   - Parse full `.ldr` models and assert scene-level invariants
   - `test_fixtures.py`: minimal 2–4 part models in `tests/fixtures/`
   - `test_walker1_integration.py`: the full Walker1 model
   - Catches: any regression that changes unit/joint/gear/motor counts

### When to Add Tests

- **New connection type or rule change**: Add a unit test in `test_connector_joints.py`
  with synthetic parts, AND verify Walker1 counts still hold.
- **New primitive recognition**: Add a port extraction test for a real part that
  uses that primitive.
- **Bug fix**: Add a test that would have caught the bug.  Prefer the lowest
  tier that reproduces it (unit test > port test > integration test).
- **New fixture model**: Add to `tests/fixtures/` with a corresponding test class
  in `test_fixtures.py`.  Keep fixtures minimal (2–5 parts).

### Test Conventions

- Integration tests that require the LDraw library use:
  ```python
  @pytest.mark.skipif(not LDRAW_LIB.exists(), reason="LDraw library not available")
  ```
- Walker1 integration tests assert exact counts (not ranges).  Update them when
  the model or algorithm intentionally changes.
- Run the full suite with `python -m pytest tests/ -v` (requires `pip install -e .`
  and LDraw library at `/opt/ldraw/ldraw`).

## Key Technical Decisions

### Connection Type Rules

| Connector        | + ROUND_HOLE       | + AXLE_HOLE              |
|------------------|--------------------|--------------------------|
| Friction pin     | rigid              | rigid                    |
| Frictionless pin | revolute           | revolute                 |
| Axle             | revolute           | rigid                    |
| Axle-pin         | rigid (friction)   | rigid (non-motor) / revolute (motor) |

**Motor exception**: On motor parts, AXLE_HOLE ports AND output-axis ROUND_HOLE
ports create revolute joints.  Output-axis ROUND_HOLEs are identified by their
orientation matching the motor's AXLE_HOLE orientation (cos > 0.9).

### Physics Constants

- 1 LDU = 0.4 mm = 0.0004 m
- Standard Technic hole spacing: 20 LDU
- ABS plastic density: 1050 kg/m³
- Axial margin for port matching: 4 LDU
- Motor max_impulse formula: `torque / fps` (impulse per frame)

### Stud / Anti-Stud Classification

- **Studs** (top): `stud.dat`, `stud2.dat`, `stud2a.dat`
- **Anti-studs** (bottom tubes): `stud3.dat`, `stud3a.dat`, `stud4.dat`, `stud4a.dat`
- Anti-stud tubes sit *between* studs laterally — matching uses axial/lateral
  decomposition (axial < 10 LDU, lateral < 14 LDU, opposing orientations).

## Environment

- LDraw library: `/opt/ldraw/ldraw`
- Blender: `~/apps/blender/blender-4.1.0-linux-x64/blender` (v4.1, CPU Cycles)
- Run tests: `python -m pytest tests/ -v`
- Generate simulation: `python -m lego_technic_sim.cli MODEL OUTPUT --ldraw-library /opt/ldraw/ldraw --simulate`
- Generate assembly: `python -m lego_technic_sim.cli MODEL OUTPUT --ldraw-library /opt/ldraw/ldraw --assembly`
