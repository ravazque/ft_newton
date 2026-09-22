# ft_newton

## Description

ft_newton is a **3D rigid-body physics engine** written from scratch in C11, in the spirit of PhysX, Havok or Bullet, demonstrated through an Angry-Birds-style game: a catapult launches birds against unstable composed structures, and every reaction — translation, rotation, elastic collisions, friction, gravity — is computed by the engine itself.

Rendering is done on the GPU with **OpenGL 3.3 Core** (GLFW for the window and input, GLAD as the function loader), using hand-written shaders and an explicit render pipeline. All the mathematics (vectors, matrices, quaternions, inertia tensors) and all the physics (integration, collision detection and response) are implemented manually — no external calculation or collision library is used.

Gameplay happens on a 2D plane, but the physics and the renderer are fully 3D.

## Features

- Three primitive colliders — **sphere, box and plane** — with a sweep-and-prune broad-phase and an exact narrow-phase (sphere/sphere, sphere/plane, sphere/box, box/plane and box/box through the separating axis theorem with face clipping).
- Rigid-body dynamics with a fixed 120 Hz semi-implicit Euler integrator, quaternion orientations and a world-space inertia tensor.
- Impulse-based contact solver: sequential impulses with accumulated clamping, warm starting, restitution, Coulomb friction and a rotation-aware positional correction.
- Island-based sleeping: a pile that has come to rest switches off as a whole, so the scene always returns to a stable state.
- Every object type (bird, block, ground, catapult) is described by a small CSV file in `assets/`, strictly validated: a malformed or out-of-range value is reported with its line instead of being silently replaced.
- Live controls over time scale, gravity, projectile speed, direction and mass, from the keyboard or from the menu; on-demand spawning of walls, pyramids, towers and a 150-block stress wall.
- An in-engine pause menu (`ESC`) with mouse-clickable `[-]` / `[+]` buttons for every tunable value, the controls reference, and reload / save / reset / spawn actions. Values edited there reach the objects already in the scene at once.
- FPS and object counters (plus the current gravity, time scale and launch settings) in the window title, an orbiting camera and a toggleable debug display that draws every collider as a wireframe.

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

| Key | Action |
|---|---|
| `ESC` | open / close the menu (opening it pauses the simulation) |
| `SPACE` | fire a bird |
| `1` / `2` / `3` | spawn a wall (6×6) / a pyramid (7 rows) / a tower (8 blocks) |
| `4` | spawn a large wall (15×10 = 150 blocks) — broad-phase stress test |
| `P` | pause / resume the simulation |
| `F1` | debug wireframe display on/off |
| `H` | title readout (FPS, object counter and live values) on/off |
| `W` / `S` | launch speed + / − (1 … 60 m/s) |
| `A` / `D` | launch angle + / − (−70° … 90°) |
| `Q` / `E` | bird mass − / + |
| `G` / `B` | gravity + / − |
| `T` / `Y` | time scale − / + (0 … 4; the physics step never changes, only how many steps a frame runs) |
| arrow keys | orbit the camera |
| `+` / `-` | zoom the camera in / out |

The yellow bar at the tip of the catapult arm shows the current launch direction; its length grows with the launch speed. The window title reads `FPS | objects | g | time | speed | angle | mass` so every live value is visible while playing; `H` hides it for a clean view.

The launch angle stops at −70°: below that the firing line would pass through the catapult's own base, so there is no clear place for a bird to start.

## Menu

`ESC` opens a menu that pauses the simulation and gathers everything in one place. It is drawn by the engine itself — an 8×8 bitmap font in `srcs/render/font.c` and a batched 2D layer in `srcs/render/ui.c` — so it needs no interface library.

- **Values** (left column): gravity, time scale, launch speed and angle, and the mass, friction and restitution of the bird, the blocks, the ground and the catapult. Click `[-]` / `[+]`, or hold them to repeat. Every change reaches the objects already in the scene at once, so a pile that has come to rest becomes lighter or slipperier immediately.
- **Controls** (right column): the same key list as above, on screen.
- **Actions**: *Reload from files* re-reads `assets/*.csv`, *Save to files* writes the current values back to them, *Reset simulation* restores gravity, time scale and launch settings and rebuilds the starting scene, the four *Spawn* buttons add structures, and *Resume* and *Quit* leave the menu and the game.

Mouse and keyboard both work: the cursor highlights and clicks rows, while ↑ / ↓ move the selection, ← / → change the selected value and `ENTER` presses the selected button.

Each value is clamped to a sensible range (gravity −40 … 20 m/s², time scale 0 … 4, launch speed 1 … 60 m/s, launch angle −70° … 90°, mass 0.1 … 50 kg, friction 0 … 20, restitution 0 … 1) and steps land on round numbers, so a value cannot be dragged into a state the engine would not accept. The bounds live in `includes/defines.h`.

The values the menu edits are the same ones the CSV files hold, which is why *Save to files* can write them back and *Reload from files* can bring them in.

## Object files

Each kind of object reads its properties from `assets/<name>.csv`. A file is a header line followed by one row per property:

```
property,value1,value2,value3
shape,sphere,,
radius,0.5,,
mass,2.0,,
friction,0.4,,
restitution,0.45,,
color,0.85,0.25,0.25
```

A row holds a property name and then one number, three numbers (a vector or an RGB color) or one word. Trailing empty fields may be left out (`mass,2.0` is the same as `mass,2.0,,`), blank lines and CRLF endings are tolerated, and there are no comments.

| File | Properties | Ranges |
|---|---|---|
| `bird.csv` | `shape,sphere` · `radius` · `mass` · `friction` · `restitution` · `color,r,g,b` | radius > 0, mass > 0 |
| `block.csv` | `shape,box` · `size,x,y,z` (full extents) · `mass` · `friction` · `restitution` · `color,r,g,b` | every extent > 0, mass > 0 |
| `ground.csv` | `shape,plane` · `normal,x,y,z` · `offset` · `extent` (drawn size) · `friction` · `restitution` · `color,r,g,b` | normal ≠ 0 (normalized on load), extent > 0; always static, so no `mass` |
| `catapult.csv` | `position,x,y,z` · `base_size,x,y,z` · `arm_length` · `arm_thickness` · `arm_angle` · `launch_speed` · `launch_angle` · `friction` · `restitution` · `color,r,g,b` | sizes > 0, arm_angle 0 … 90, launch_speed 1 … 60, launch_angle −70 … 90; always static, so no `mass` |

For every file: `friction` ≥ 0, `restitution` and each `color` component within 0 … 1.

Loading is strict, so a mistake is never silently turned into a default:

- the header must be exactly `property,value1,value2,value3`;
- every property of the shape must be present, and nothing else may appear (a misspelt name such as `masss` is an error, as is a `radius` in a box file);
- an unknown shape word, a duplicated property, a value with the wrong count (two numbers, a word where a number is expected), a malformed or overflowing number, `nan`, `inf` or hexadecimal are all rejected;
- a value outside its range (negative radius, mass or friction, a color or restitution above 1, a zero normal…) is rejected.

Every problem is reported on stderr as `file:line: message`, all at once for a given file. At start-up a bad file stops the program; reloading a bad file from the menu keeps the previous definitions and prints why.

The files can be edited while the game runs. *Reload from files* in the menu reads them again and updates what can change on the fly: every existing bird, block, ground and catapult body takes the new `mass` (its inertia is recomputed), `friction`, `restitution` and `color`, the launch settings return to the file values, and everything is woken up so a lighter or slipperier pile reacts immediately. Geometry — `shape`, `size`, `radius`, the plane's `normal` and `offset`, the catapult layout — applies to objects spawned from then on, or to the whole scene after *Reset simulation*. *Save to files* does the opposite: it writes what the menu currently holds back into the four files, overwriting them.

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

The menu is drawn with those same two shaders, so the project needs no interface or font library either. `srcs/render/font.c` holds an 8×8 pixel bitmap for every printable ASCII character, one byte per row; `srcs/render/ui.c` turns panels, buttons and glyph pixels into colored quads, all of them in one vertex buffer, and sends them in a single `glDrawArrays` with depth testing off, an orthographic projection in window pixels and `uLit = 2`. Because a glyph is drawn in whole pixels, the menu picks the largest whole font scale that fits the window instead of stretching.

The readout of FPS and object count stays in the window title, so the scene itself is clean while playing (`H` hides it entirely).

## Project layout

```
Makefile              single `make` builds everything
includes/newton.h     every type and prototype (the only header included by the sources)
includes/defines.h    constants: window, physics, solver, controls, asset paths
includes/formulas.h   every physics formula, tagged [F1]..[F25]
srcs/main.c           entry point
srcs/math/            vec3, mat3, mat4, quat
srcs/physics/         rigidbody, integrator, world (step, sleeping, culling)
srcs/collision/       collider, broadphase, narrowphase, narrowphase_box, resolver
srcs/render/          window, shader, mesh, camera, renderer, debugdraw, font, ui
srcs/game/            game loop and input, catapult, projectile, structure, hud, objectdef, menu
srcs/utils/           command-line checks, strict CSV object file reader
shaders/              basic.vert / basic.frag (GLSL 3.30 core)
assets/               the object definition files (CSV)
glad/                 vendored OpenGL loader
```

## Where the physics is written down

`includes/formulas.h` lists every formula the engine implements — Newton's second law, semi-implicit Euler, quaternion integration, the inertia tensors and their rotation into world space, the AABB and interval tests of the broad-phase, each narrow-phase contact test including the separating axis theorem and the face clipping, the effective mass, restitution, impulse and friction of the contact solver, the positional correction, the sleep test and the launch velocity — each with a tag from `[F1]` to `[F25]`.

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
4. **Resolve**: contacts are solved by Gauss-Seidel iterations of sequential impulses. Each contact accumulates a normal impulse (clamped to push only) and two friction impulses (clamped by `μ · normal`), starting from the previous step's values. Impacts faster than a threshold bounce according to restitution; slower ones aim at zero relative speed. Remaining overlap is then removed by a few positional passes that translate and rotate the bodies through the same effective masses.
5. **Sleep**: bodies below both speed thresholds accumulate rest time; a contact island goes to sleep only when all of its bodies have rested long enough. Bodies flying beyond the world bounds are removed.
