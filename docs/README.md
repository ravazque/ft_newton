# ft_newton

## Description

ft_newton is a **3D rigid-body physics engine** written from scratch in C11, in the spirit of PhysX, Havok or Bullet, demonstrated through an Angry-Birds-style game: a trebuchet launches apples against unstable composed structures, and every reaction — translation, rotation, elastic collisions, friction, gravity — is computed by the engine itself.

Rendering is done on the GPU with **OpenGL 3.3 Core** (GLFW for the window and input, GLAD as the function loader), using hand-written shaders and an explicit render pipeline. All the mathematics (vectors, matrices, quaternions, inertia tensors) and all the physics (integration, collision detection and response) are implemented manually — no external calculation or collision library is used.

Gameplay happens on a 2D plane, but the physics and the renderer are fully 3D, and the camera flies freely through the scene.

## Features

- Three primitive colliders — **sphere, box and plane** — with a sweep-and-prune broad-phase and an exact narrow-phase (sphere/sphere, sphere/plane, sphere/box, box/plane and box/box through the separating axis theorem with face clipping).
- Rigid-body dynamics with a fixed 120 Hz semi-implicit Euler integrator, quaternion orientations and a world-space inertia tensor.
- Impulse-based contact solver: sequential impulses with accumulated clamping, warm starting, elasticity, Coulomb friction and a rotation-aware positional correction.
- Island-based sleeping: a pile that has come to rest switches off as a whole, so the scene always returns to a stable state.
- Every object type (apple, block, ground, trebuchet) is described by a small CSV file in `assets/`, strictly validated: a malformed or out-of-range value is reported with its line instead of being silently replaced.
- Live controls over time scale, gravity, projectile speed, direction and mass, from the keyboard or from the menu; on-demand spawning of walls, pyramids, towers and a 150-block stress wall.
- An in-engine pause menu (`ESC`) with mouse-clickable `[-]` / `[+]` buttons for every tunable value, the controls reference, and apply / reload / save / reset actions.
- FPS and object counters (plus the current gravity, time scale and launch settings) in the window title, a free-flying camera and a toggleable debug display that draws every collider as a wireframe.

## Requirements

- A C11 compiler (`cc`), GNU Make and `pkg-config`-free system libraries: **GLFW 3** and **OpenGL** (Mesa or a vendor driver).
- GLAD is vendored in `glad/` (generated for OpenGL 3.3 Core, no extensions).

On Arch-based systems: `sudo pacman -S --needed glfw mesa`. On Debian/Ubuntu: `sudo apt install libglfw3-dev libgl1-mesa-dev`.

## Build and run

```bash
make          # builds ./newton
make run      # builds and runs (optionally: make run ARGS="1280 720")
make clean    # removes the object files
make fclean   # removes the object files and the binaries
make re       # rebuilds from scratch
```

`./newton [width height]` opens a window of the given size (clamped to 720×480 … 2560×1440). Run it from the repository root so the `shaders/` and `assets/` folders resolve.

The main loop is capped at 30 frames per second (`FPS_CAP` in `includes/defines.h`): vertical sync is off and each frame sleeps off whatever time it has left, so the rate does not depend on the monitor. The physics is independent of it and always advances in fixed 1/120 s steps.

## Controls

The keys act on the running simulation immediately.

| Key | Action |
|---|---|
| `ESC` | open / close the menu (opening it pauses the simulation) |
| `SPACE` | fire an apple |
| `1` / `2` / `3` | spawn a wall (6×6) / a pyramid (7 rows) / a tower (8 blocks) |
| `4` | spawn a large wall (15×10 = 150 blocks) — broad-phase stress load |
| `P` | pause / resume the simulation |
| `F1` | debug wireframe display on/off |
| `H` | title readout (FPS, object counter and live values) on/off |
| `I` / `K` | launch angle + / − (−70° … 90°) |
| `J` / `L` | launch speed − / + (1 … 60 m/s) |
| `Q` / `E` | apple mass − / + |
| `G` / `B` | gravity + / − |
| `T` / `Y` | time scale − / + (0 … 4; the physics step never changes, only how many steps a frame runs) |

The camera is free-flying: it can be walked anywhere in the 3D scene, even though the gameplay itself stays on the XY plane.

| Key | Action |
|---|---|
| `W` / `S` | fly forward / backward along the view |
| `A` / `D` | strafe left / right |
| `R` / `F` | rise / descend |
| arrow keys | turn the view (left / right yaw, up / down pitch) |
| `+` / `-` | fly faster / slower (2 … 60 m/s) |

The yellow bar at the tip of the throwing arm shows the current launch direction; its length grows with the launch speed. The window title reads `FPS | objects | g | time | speed | angle | mass` so every live value is visible while playing; `H` hides it for a clean view.

The launch angle stops at −70°: below that the firing line would pass through the machine's own frame, so there is no clear place for an apple to start.

## Menu

`ESC` opens a menu that pauses the simulation and gathers everything in one place. It is drawn by the engine itself — an 8×8 bitmap font in `srcs/render/font.c` and a batched 2D layer in `srcs/render/ui.c` — so it needs no interface library. The panel never covers more than 80% of the window, so the scene stays visible around it.

**The menu edits a draft, not the simulation.** Moving a value changes nothing on screen: the header switches to `APPLY to use the edits` and the numbers only reach the world when *Apply to simulation* is pressed. That way a whole set of values can be dialled in at once instead of the scene reacting to every intermediate step. Opening the menu always refreshes the draft from what is currently running, so it never shows stale numbers.

- **Values** (left column): gravity, time scale, launch speed and angle, and the mass, friction and elasticity of the apple, the blocks, the ground and the trebuchet. Click `[-]` / `[+]`, or hold them to repeat.
- **Controls** (right column): the same key list as above, on screen.
- **Actions**:
  - *Apply to simulation* — the draft becomes the running world, and the state *Reset* returns to.
  - *Reload from files* — re-reads `assets/*.csv` into both the simulation and the draft.
  - *Save to files* — writes the draft into `assets/*.csv`, overwriting them. It does not touch the running simulation.
  - *Reset simulation* — restores gravity, time scale and launch settings and rebuilds the starting scene.
  - *Resume* / *Quit*.

Mouse and keyboard both work: the cursor highlights and clicks rows, while ↑ / ↓ move the selection, ← / → change the selected value and `ENTER` presses the selected button.

Each value is clamped to a sensible range (gravity −40 … 20 m/s², time scale 0 … 4, launch speed 1 … 60 m/s, launch angle −70° … 90°, mass 0.1 … 50 kg, friction 0 … 20, elasticity 0 … 1) and steps land on round numbers, so a value cannot be dragged into a state the engine would not accept. The bounds live in `includes/defines.h`.

## Object files

Each kind of object reads its properties from `assets/<name>.csv`. A file is a header line followed by one row per property:

```
# property,value1,value2,value3
shape,sphere,,
radius,0.5,,
mass,2.0,,
friction,0.4,,
elasticity,0.45,,
color,0.85,0.25,0.25
```

The header must be exactly `# property,value1,value2,value3`, leading `# ` included. A row holds a property name and then one number, three numbers (a vector or an RGB color) or one word. Trailing empty fields may be left out (`mass,2.0` is the same as `mass,2.0,,`), blank lines and CRLF endings are tolerated.

| File | Properties | Ranges |
|---|---|---|
| `apples.csv` | `shape,sphere` · `radius` · `mass` · `friction` · `elasticity` · `color,r,g,b` | radius > 0, mass > 0 |
| `block.csv` | `shape,box` · `size,x,y,z` (full extents) · `mass` · `friction` · `elasticity` · `color,r,g,b` | every extent > 0, mass > 0 |
| `ground.csv` | `shape,plane` · `normal,x,y,z` · `offset` · `extent` (drawn size) · `friction` · `elasticity` · `color,r,g,b` | normal ≠ 0 (normalized on load), extent > 0; always static, so no `mass` |
| `trebuchet.csv` | `position,x,y,z` · `base_size,x,y,z` · `frame_height` · `arm_length` · `arm_thickness` · `arm_angle` · `counterweight_size,x,y,z` · `launch_speed` · `launch_angle` · `friction` · `elasticity` · `color,r,g,b` | sizes > 0, frame_height above the sill, arm_angle 0 … 90, launch_speed 1 … 60, launch_angle −70 … 90; always static, so no `mass` |

For every file: `friction` ≥ 0, `elasticity` and each `color` component within 0 … 1.

`elasticity` is the bounciness of the material: 0 means an impact dies on contact, 1 means a perfectly elastic rebound. It is the `e` of [F20] in [PHYSICS.md](PHYSICS.md).

Loading is strict, so a mistake is never silently turned into a default:

- the header must be exactly `# property,value1,value2,value3`;
- every property of the shape must be present, and nothing else may appear (a misspelt name such as `masss` is an error, as is a `radius` in a box file);
- an unknown shape word, a duplicated property, a value with the wrong count (two numbers where three are expected, four where three are, a word where a number is expected), a malformed or overflowing number, `nan`, `inf` or hexadecimal are all rejected;
- a value outside its range (negative radius, mass or friction, a color or elasticity above 1, a zero normal…) is rejected.

Every problem is reported on stderr as `file:line: message`, all at once for a given file, and a missing file says so with the reason the system gave. The four files are required: if any of them cannot be read, the program names all four and stops instead of starting with half a scene. Reloading a bad file from the menu keeps the previous definitions and prints why.

### The trebuchet

The machine is five static boxes: the sill lying on the ground (`base_size`), the two legs of the A-frame that carry the pivot at `frame_height`, the throwing arm turning on that pivot (`arm_length`, `arm_thickness`, `arm_angle`) and the counterweight hanging off its short side (`counterweight_size`). What the file does not spell out — how far apart the legs stand, how thick they are and where the pivot cuts the arm — follows fixed proportions in `includes/defines.h`.

Apples are released at the tip of the throwing arm. The spawn point is pushed along the firing line past every one of the five boxes, so whatever the launch angle an apple never starts inside a collider.

The shot itself is not simulated from the counterweight: the launch velocity is exactly `speed × (cos angle, sin angle, 0)`, which is what keeps speed and direction a direct control. From that instant the engine owns the trajectory — the parabola comes out of gravity alone, nothing is scripted.

### Editing the files

The files can be edited while the game runs. *Reload from files* in the menu reads them again and updates what can change on the fly: every existing apple, block, ground and trebuchet body takes the new `mass` (its inertia is recomputed), `friction`, `elasticity` and `color`, the launch settings return to the file values, and everything is woken up so a lighter or slipperier pile reacts immediately. Geometry — `shape`, `size`, `radius`, the plane's `normal` and `offset`, the trebuchet layout — applies to objects spawned from then on, or to the whole scene after *Reset simulation*. *Save to files* does the opposite: it writes what the menu currently holds back into the four files, overwriting them.

## Graphics library and shaders

Rendering uses **OpenGL 3.3 Core Profile** through two thin layers, both free of any physics or collision code:

- **GLFW 3** creates the window and the OpenGL context and reports keyboard and mouse input (`srcs/render/window.c`, `srcs/game/game.c`).
- **GLAD** (vendored in `glad/`, generated for GL 3.3 Core with no extensions) loads the OpenGL function pointers at run time, so no driver-specific linking is needed.

Everything drawn goes through a single programmable pipeline made of two GLSL 3.30 shaders in `shaders/`, compiled and linked at start-up by `srcs/render/shader.c` (a compile or link error is printed with the driver's log and the program exits):

| Shader | Stage | What it does |
|---|---|---|
| `basic.vert` | vertex | Receives each vertex position and normal (attributes 0 and 1 of the mesh's VAO) and the three matrices `uModel`, `uView`, `uProjection`. Computes `gl_Position = P · V · M · position`, which is how a 3D point ends up on the 2D screen, and rotates the normal into world space for the fragment stage. |
| `basic.frag` | fragment | Colors every pixel of a triangle, in one of three modes chosen by `uLit`. `1`: a fixed directional light (ambient 0.25 + diffuse from the world-space normal) applied to the body's `uColor`, so shapes read as solid. `0`: `uColor` flat, which the debug wireframes and the aim bar use. `2`: the color carried by the vertex itself, which is how the menu mixes panels, buttons and text in a single draw call. |

Each frame (`srcs/render/renderer.c`) clears the color and depth buffers, uploads the camera's view and projection matrices once, and then draws every body with its own model matrix built from the body's position and orientation quaternion (`mat4_transform`) and the mesh of its collider: a unit cube scaled to the box extents, a UV sphere scaled to the diameter, or a square scaled to the ground's `extent`. Meshes (`srcs/render/mesh.c`) are uploaded once to the GPU as a VAO + VBO + EBO and drawn with `glDrawElements`; depth testing keeps nearer surfaces in front.

The debug display (`F1`, `srcs/render/debugdraw.c`) uses the same shaders: it switches the rasterizer to `glPolygonMode(GL_LINE)`, draws every collider slightly inflated with flat shading, and switches back.

The menu is drawn with those same two shaders, so the project needs no interface or font library either. `srcs/render/font.c` holds an 8×8 pixel bitmap for every printable ASCII character, one byte per row; `srcs/render/ui.c` turns panels, buttons and glyph pixels into colored quads, all of them in one vertex buffer, and sends them in a single `glDrawArrays` with depth testing off, an orthographic projection in window pixels and `uLit = 2`. Because a glyph is drawn in whole pixels, the menu picks the largest whole font scale that fits instead of stretching.

The readout of FPS and object count stays in the window title, so the scene itself is clean while playing (`H` hides it entirely).

## Project layout

```
Makefile              single `make` builds everything
includes/newton.h     every type and prototype (the only header included by the sources)
includes/defines.h    constants: window, physics, solver, controls, asset paths
docs/PHYSICS.md       every physics formula, tagged [F1]..[F25]
srcs/main.c           entry point
srcs/math/            vec3, mat3, mat4, quat
srcs/physics/         rigidbody, integrator, world (step, sleeping, culling)
srcs/collision/       collider, broadphase, narrowphase, narrowphase_box, resolver
srcs/render/          window, shader, mesh, camera, renderer, debugdraw, font, ui
srcs/game/            game loop and input, trebuchet, projectile, structure, hud, objectdef, menu
srcs/utils/           command-line checks, strict CSV object file reader
shaders/              basic.vert / basic.frag (GLSL 3.30 core)
assets/               the object definition files (CSV)
glad/                 vendored OpenGL loader
```

## Where the physics is written down

[docs/PHYSICS.md](PHYSICS.md) lists every formula the engine implements — Newton's second law, semi-implicit Euler, quaternion integration, the inertia tensors and their rotation into world space, the AABB and interval tests of the broad-phase, each narrow-phase contact test including the separating axis theorem and the face clipping, the effective mass, elasticity, impulse and friction of the contact solver, the positional correction, the sleep test and the launch velocity — each with a tag from `[F1]` to `[F25]`.

The line of code that implements a formula carries the same tag, so either direction works:

```bash
grep -rn "\[F21\]" srcs/      # where is the normal impulse computed?
```

```c
lambda = c->massNormal * (c->bias - vec3_dot(relative_velocity(a, b, c), c->normal));   /* [F21] */
```

## How a step works

The game loop accumulates the real frame time multiplied by the time scale and runs as many fixed steps of 1/120 s as fit (up to a cap per frame, after which leftover time is dropped so a heavy scene slows down instead of stalling). The step itself never changes: at 4× the world simply takes four times as many steps, so a fast projectile cannot skip through a block, and at 0.25× the slow motion is exact.

Each fixed step (`world_step`):

1. **Integrate** every awake body: forces and gravity update the velocity, the velocity moves the position, torque updates the angular velocity, which integrates the orientation quaternion; the world inverse inertia tensor is refreshed as `R · I⁻¹ · Rᵀ`.
2. **Broad-phase**: axis-aligned bounding boxes are sorted along X (sweep and prune); only overlapping pairs with at least one awake dynamic body go further.
3. **Narrow-phase**: the exact test for each shape pair produces contacts (normal, point, penetration), up to four per pair, kept well spread so resting boxes stay steady.
4. **Resolve**: contacts are solved by Gauss-Seidel iterations of sequential impulses. Each contact accumulates a normal impulse (clamped to push only) and two friction impulses (clamped by `μ · normal`), starting from the previous step's values. Impacts faster than a threshold bounce according to elasticity; slower ones aim at zero relative speed. Remaining overlap is then removed by a few positional passes that translate and rotate the bodies through the same effective masses.
5. **Sleep**: bodies below both speed thresholds accumulate rest time; a contact island goes to sleep only when all of its bodies have rested long enough. Bodies flying beyond the world bounds are removed.
