# ft_newton

## 📖 About

"ft_newton" is a **3D rigid-body physics engine** written from scratch in C, in
the spirit of PhysX, Havok or Bullet, shown through an Angry-Birds-style game: a
trebuchet throws apples at unstable structures of blocks, and every reaction —
translation, rotation, bounces, friction, rolling, gravity — is computed by the
engine itself.

The scene is drawn with **OpenGL 3.3 Core** (GLFW for the window and the input,
GLAD to load the GL functions), through two hand-written shaders. All the
mathematics (vectors, matrices, quaternions, inertia tensors) and all the
physics (integration, collision detection, contact response) are implemented in
the project: there is no physics or collision library anywhere, and the only
libraries linked are GLFW, OpenGL, `libdl`, `pthread` and `libm`.

The engine does not approximate: boxes collide through the separating axis
theorem and clipped face manifolds, contacts are solved with sequential
impulses, friction is Coulomb, spheres roll and resist rolling, positions are
corrected after the impulses, and a pile that has come to rest is put to sleep
as a whole island. Anything the engine refuses — a body starting inside another
one, a malformed object file — is reported instead of being patched up.

The gameplay happens on the XY plane, but nothing in the engine is 2D: the
integrator, the colliders and the camera are fully three-dimensional, and the
camera flies freely through the scene at any time. The simulation runs on a
fixed 120 Hz step whatever the display rate is, and the time scale changes how
many steps a frame runs, never their size.

Everything is documented: the source carries a tag such as `[F21]` on the line
that implements each formula, and `docs/info/physics.md` explains the same tag.

| Document | What it explains |
|---|---|
| [docs/info/architecture.md](docs/info/architecture.md) | The source tree, one responsibility per file, the main loop and one physics step |
| [docs/info/physics.md](docs/info/physics.md) | Every formula the engine implements, tagged `[F1]` to `[F26]`, and where it lives |
| [docs/info/collisions.md](docs/info/collisions.md) | Broad-phase, narrow-phase, contact response, sleeping, the bounded ground, spawning without overlaps, the classic collision problems |
| [docs/info/rendering.md](docs/info/rendering.md) | The OpenGL pipeline: window, meshes, camera, draw calls, debug wireframes, the menu overlay |
| [docs/info/shaders.md](docs/info/shaders.md) | The vertex and fragment shaders, line by line |
| [docs/info/object-files.md](docs/info/object-files.md) | The CSV files that define apples, blocks, ground and trebuchet, and how they are checked |

## 🎯 Objectives

- Writing the maths from scratch: vectors, matrices, quaternions, inertia
  tensors, and the world-space form of a tensor that turns with its body
- Implementing **semi-implicit Euler** integration over a fixed step, with
  quaternion orientation integration and renormalisation
- Building **collision detection** in three stages: AABBs and sweep and prune,
  then the exact test of every shape pair, then contact generation
- Implementing the **separating axis theorem** on 15 axes, face clipping
  (Sutherland-Hodgman) and the reduction of a contact patch to four spread
  points
- Solving contacts with **sequential impulses**: effective mass, accumulated
  clamping, warm starting, Coulomb friction and rolling resistance
- Removing the leftover overlap with **positional correction**, on positions
  and not on velocities, so no energy is added
- Making a scene come to rest: island sleeping, a bounce threshold, friction,
  angular damping and culling of what left the world
- Placing new bodies without ever creating an overlap, and refusing the ones
  that would
- Loading the object definitions strictly: no silent defaults, every mistake
  reported with its file and line
- Rendering an interactive 3D scene, a free camera, a debug wireframe view and
  a 2D menu drawn by the engine's own overlay
- Freeing everything on every path, and checking it with valgrind

## 📋 Function Overview

<details>
<summary><strong>ft_newton — modules breakdown</strong></summary>

<br>

| Module | Feature | Description |
|--------|---------|-------------|
| **math** | Vectors | `vec3`: add, subtract, scale, dot, cross, length, normalise |
| **math** | Matrices | `mat3` for inertia tensors (product, transpose, inverse, from a quaternion); `mat4` for rendering (translation, scale, rotation, perspective, look-at) |
| **math** | Quaternions | Rotation of a vector, composition, shortest arc between two vectors, integration of an angular velocity, renormalisation |
| **physics** | Bodies | Mass, `invMass`, `invInertiaLocal`, forces, torque, impulses at a point, accumulators cleared every step |
| **physics** | Inertia | Solid sphere `I = (2/5) m R²` [F5], solid box from its half extents [F6], conjugated into world space [F7] |
| **physics** | Integration | Semi-implicit Euler [F1]-[F2]: velocity first, then the position moves with the new velocity; angular damping [F4] |
| **physics** | World | The body array (add, swap-with-last removal, clear, wake all), the growing scratch arrays, and `world_step` |
| **physics** | Sleeping | Union-find over the contacts: an island sleeps only when **all** its bodies stayed still for 0.5 s [F24] |
| **physics** | Culling | Bodies 200 m away, or 20 m under a plane after falling past its edge, leave the world and the counters |
| **collision** | Colliders | Sphere, oriented box, and a **bounded plane**: solid down to 2 m under its surface, but only over the square that is drawn |
| **collision** | Broad-phase | Sweep and prune: AABBs sorted by minimum X, neighbours tested on Y and Z, static/static and sleeping/sleeping pairs skipped |
| **collision** | Dispatch | Each candidate pair goes to its test; the normal is always rewritten to point from `a` to `b` |
| **collision** | Sphere tests | Sphere/sphere, sphere/plane and sphere/box, the last through the closest point clamped in the box's own frame [F12]-[F14] |
| **collision** | Box tests | Box/plane corner by corner; box/box through SAT [F15], then clipping [F16] or an edge contact |
| **collision** | SAT | 15 candidate axes (6 faces, 9 edge cross products), least overlap wins, with a bias that prefers face normals |
| **collision** | Manifolds | Sutherland-Hodgman clipping of the incident face, then the deepest point, the farthest one and the two widest kept |
| **collision** | Queries | Do two bodies touch or overlap, is a spawn point free, is a body over the ground square |
| **response** | Solver | Wake, prepare, warm start, 16 Gauss-Seidel passes, position correction, remember for the next step |
| **response** | Impulses | Normal impulse with accumulated clamping and elasticity [F20]-[F21], Coulomb friction on two tangents [F22], equal and opposite at the contact point [F9] |
| **response** | Rolling | Rolling resistance for spheres: an opposing spin impulse bounded by `c_rr R jn`, about both tangents and the normal [F26] |
| **response** | Correction | The overlap left beyond 5 mm removed in 3 passes, by moving **and** turning the bodies through the same effective masses [F23] |
| **render** | Window | GLFW window and OpenGL 3.3 Core context, GLAD loader, framebuffer resize, cursor, mouse and keyboard queries |
| **render** | Shaders | Compiling and linking `shaders/basic.vert` / `basic.frag`, uniforms for the MVP matrices, the colour and the shading mode |
| **render** | Meshes | One unit cube, one sphere and one plane in GPU memory; every body is drawn by scaling one of them with its model matrix |
| **render** | Camera | Free-flying eye (position, yaw, pitch) with a 60° perspective, a clamped pitch and a fly speed of 2 to 60 m/s |
| **render** | Draw | Depth-tested draw calls, a flat model for the debug wireframes and the aim bar, and the body-to-mesh mapping |
| **render** | Debug view | `F1` draws every collider as a wireframe, slightly inflated (the plane lifted) so it sits on the solid |
| **render** | Overlay | A batched 2D layer: one vertex buffer with a colour per vertex, and the whole menu in a single draw call |
| **render** | Font | An 8×8 bitmap font for ASCII 32 to 126, uploaded once, with a fixed advance per glyph |
| **menu** | Rows | Sections, value rows (pointing into the draft), action buttons and hint lines, with a value stepper per row |
| **menu** | Layout | The 75% panel and the rectangle of every row, recomputed each frame so the text scales with the window |
| **menu** | Input | Mouse hover, click and hold-to-repeat on `[-]` / `[+]`; arrows to move and change, ENTER to press a button |
| **menu** | Drawing | The panel, the title line and every row, with the hovered and held states, drawn into the overlay |
| **data** | CSV reader | Header, then `property,value1,value2,value3` rows; plain decimal numbers only, trailing empty fields optional, blank lines tolerated |
| **data** | Getters | Typed access that counts errors instead of defaulting, an unknown-property check, and range checks that name the expected bound |
| **data** | Object definitions | Apples, blocks and ground loaded into definitions, turned into bodies, saved back to the same files |
| **data** | Trebuchet file | The machine's own file, with the projectile mass taken from the apple file and checked against its minimum |
| **game** | Loop | Poll, read the input, drain the accumulated time in fixed steps, update the counters, draw, swap, sleep to the frame cap |
| **game** | Input | Every key of the game (fire, spawn, pause, toggles, live tuning, camera), with held keys changing a value per second |
| **game** | HUD | FPS and object counter with the live values in the window title, rewritten five times a second |
| **game** | Scene | Loading the four files, refusing a starting scene that would overlap or stand off the ground, rebuilding it on reset |
| **game** | Structures | Walls, pyramids and towers spawned on demand, lifted onto whatever occupies their spot |
| **game** | Trebuchet | Five static boxes built from the file; the launch point at the tip of the arm, the shot, and the refusal when the point is blocked |
| **game** | Clearance | A ray along the firing line against every box grown by the apple's radius, at every allowed angle, gives one spawn distance |
| **game** | Projectile | An apple with its launch velocity from the file's speed and angle, and the mass of the next apples |
| **game** | Actions | What the menu buttons do: sync the draft, apply, reload, save, reset, resume, quit |
| **utils** | Arguments | The optional `[width height]`, accepted only when both are numbers inside the window bounds |

<br>

</details>

<details>
<summary><strong>Usage Example & Testing</strong></summary>

<br>

### Build and run

```bash
make                                    # builds ./newton (and the vendored GLAD)
make run                                # builds and runs with the default 1280x720
make run ARGS="1920 1080"               # ... or at another size
make clean && make re                   # object files, then a full rebuild
make fclean                             # removes objects/ and the binary
```

Run it **from the repository root**: `shaders/` and `assets/` are opened by
relative path, and a missing one is fatal.

```bash
./newton                # window of 1280x720
./newton 2560 1440      # the largest accepted size
./newton 100 100        # out of range: warns and falls back to 1280x720
./newton 800            # wrong argument count: prints the usage and exits 1
```

### What the window shows

The starting scene is a 160 m square of ground, the five boxes of the trebuchet
at x = -12 and a five-row pyramid of blocks at x = 8. The camera starts on the
+Z side, framing the machine on the left and the pyramid on the right.

The title is rewritten every 0.2 s from the live state (press `H` to hide it),
in the shape below — the values are the ones a fresh run reports:

```
ft_newton  |  FPS: nn  |  objects: n  |  g: -9.81  |  time: x1.00  |  speed: 11.0 m/s  |  angle: 25 deg  |  mass: 2.00 kg  |  PAUSED
```

Fire an apple with `SPACE` and watch it: the parabola is not scripted, the apple
is an ordinary rigid body from its first step. Spawn a wall with `1` and hit it;
the blocks tip, slide, pile up and finally switch off as one sleeping island.
`F1` shows the colliders that produce all of it, `T` slows the time down to see a
single impact step by step, `P` freezes a contact to look at it from any angle.

### Refusals and error handling

The engine reports what it will not do instead of quietly fixing it. All of
these are real messages from the program:

```bash
$ ./newton 800
Usage: ./newton [width height]
$ ./newton 100 100
Width '100' is out of range [720..2560]; using default 1280
Height '100' is out of range [480..1440]; using default 720
$ mv assets/apples.csv /tmp/
assets/apples.csv, assets/block.csv, assets/ground.csv and assets/trebuchet.csv are all required and must be valid
```

A broken object file stops the load with its line and the reason, and nothing is
half-loaded:

```
assets/block.csv:1: first line must be the header '# property,value1,value2,value3'
assets/block.csv:4: 'mass' must be > 0
assets/block.csv:9: unknown property 'density'
assets/ground.csv: missing property 'normal'
assets/apples.csv:7: duplicated property 'mass'
assets/apples.csv:2: unknown shape 'disc' (expected sphere, box or plane)
```

Starting scenes are checked as well: a pyramid that would overlap the trebuchet,
sink into the ground or stand outside the ground square is refused with the
files involved. At runtime, `SPACE` prints `Launch point blocked: an apple would
start inside another body` when something already occupies the spawn point.

### Checking the work

```bash
# The engine's own allocations: leaks reported inside the GL / GTK / Mesa
# driver stack are suppressed, so what is left is the program's.
valgrind --leak-check=full --show-leak-kinds=all \
         --suppressions=docs/valgrind.supp ./newton
```

`docs/valgrind.supp` names the driver and toolkit stacks (fontconfig, pango,
GTK/GDK, GLib, glycin, Mesa gallium, and the rest) that keep global state until
process exit; nothing of `ft_newton` itself is suppressed.

There is no test suite: the program *is* the test bench. The loop that matters
when a change lands is: build, `F1` to see the colliders, `T` to slow the time,
`P` to freeze, and — after touching the solver, the spawn rules or the loaders —
the valgrind run above. The project sources are built with `-Wall -Wextra
-Werror`, so a warning is a build failure and cannot be left behind; only the
vendored GLAD is compiled with the relaxed flag set.

<br>

</details>

## 🎮 Controls

The keys act on the running simulation immediately; the mouse and the keyboard
both drive the menu when it is open.

| Key | Action |
|---|---|
| `ESC` | open / close the menu (opening it pauses the simulation) |
| `SPACE` | fire an apple (refused, with a message, while the spawn point is blocked) |
| `1` / `2` / `3` | spawn a 6x6 wall / a 7-row pyramid / an 8-block tower at x = 8 m |
| `4` | spawn a 15x10 wall (150 blocks) at x = 20 m |
| `P` | pause / resume |
| `F1` | collider wireframes on / off |
| `H` | window title readout on / off |
| `I` / `K` | launch angle up / down (-70 to 90 degrees) |
| `J` / `L` | launch speed down / up (1 to 60 m/s) |
| `Q` / `E` | mass of the next apples down / up (0.1 to 50 kg) |
| `G` / `B` | gravity up / down (-40 to 20 m/s²) |
| `T` / `Y` | time scale down / up (0 to 4) |
| `W` `A` `S` `D` | fly the camera forward, left, backward, right |
| `R` / `F` | camera up / down |
| arrow keys | turn the camera |
| `+` / `-` | camera fly speed up / down (2 to 60 m/s) |

Held keys change a value at a fixed rate per second (10 m/s of launch speed,
60° of angle, 1 kg of mass, 5 m/s² of gravity, 1 of time scale); the tuned
values are the ones the title shows. Changing gravity wakes every body, so a
resting pile reacts again instead of standing on stale supports.

The yellow bar at the tip of the throwing arm shows the launch direction, and
the thin line past it grows with the launch speed (0.12 m per m/s). A new apple
appears past that bar, at the same distance for every allowed angle: the
distance is computed once, by testing the firing line against the machine's
boxes grown by the apple's radius.

Structures always appear at the same spot. When that spot is taken, the whole
structure is lifted until no block would sink into what is there and it lands on
top of it, so no body ever starts inside another one.

## 🚀 Installation & Structure

<details>
<summary><strong>📥 Setup & Usage</strong></summary>

<br>

### Host requirements

| Tool | Needed for | Notes |
|------|------------|-------|
| `cc` (gcc or clang) | Compiling | `-Wall -Wextra -Werror -g3 -O3` |
| `make` | Driving the build | One `make` builds the sources and the vendored GLAD |
| **GLFW 3** | Window and input | Development headers, `-lglfw` |
| **OpenGL** | Rendering | A 3.3 Core capable driver (Mesa or a vendor one), `-lGL` |
| Any Linux / Unix | Running | Also links `-ldl -lpthread -lm` |

GLAD is **not** an external dependency: the generated loader for OpenGL 3.3
Core (no extensions) is committed in `glad/` and compiled with the project.

```bash
# Arch
sudo pacman -S --needed glfw mesa
# Debian / Ubuntu
sudo apt install libglfw3-dev libgl1-mesa-dev
```

### Make targets

```bash
make            # build ./newton (project sources, then glad/src/gl.c)
make run        # build and run; ARGS="1280 720" is passed to the binary
make clean      # remove the objects/ directory
make fclean     # clean + remove the binary
make re         # fclean + build
```

Project sources are compiled with the strict warning set; the vendored GLAD is
compiled with the same optimisation but without `-Wall -Wextra -Werror`, so a
warning from generated code cannot stop the build. Dependencies are tracked
(`-MMD -MP`), so touching a header rebuilds what includes it.

### Running

```bash
./newton                    # 1280x720
./newton 1920 1080          # any size in [720..2560] x [480..1440]
make run ARGS="2560 1440"
```

The program takes either no argument or exactly two: a width and a height, both
decimal, inside the bounds above. Anything else prints the usage and exits with
status 1; a single bad value is replaced by the default and reported on stderr.

### Object file format

Each object is one small CSV file in `assets/`, named by its first row:

```
# property,value1,value2,value3
shape,sphere,,
radius,0.5,,
mass,2,,
rolling_resistance,0.3,,
friction,0.4,,
elasticity,0.45,,
color,0.85,0.25,0.25
```

| File | Property rows |
|---|---|
| `assets/apples.csv` | `shape` (sphere), `radius`, `mass`, `rolling_resistance`, `friction`, `elasticity`, `color` |
| `assets/block.csv` | `shape` (box), `size`, `mass`, `friction`, `elasticity`, `color` |
| `assets/ground.csv` | `shape` (plane), `normal`, `offset`, `extent`, `friction`, `elasticity`, `color` |
| `assets/trebuchet.csv` | `position`, `base_size`, `frame_height`, `arm_length`, `arm_thickness`, `arm_angle`, `counterweight_size`, `launch_speed`, `launch_angle`, `friction`, `elasticity`, `color` |

The format is deliberately small and strict: `#` comments and the header line,
then one property per line holding one number, three numbers, or one word.
Numbers are plain decimals (no `nan`, `inf` or hex), trailing empty fields may be
left out, blank lines and CRLF are tolerated, and **nothing has a fallback
value** — a misspelt property, a missing one, a wrong number of fields, a
duplicate row or a value outside its range stops the load with
`file:line: reason`. The ground file must describe a plane, the apple file a
sphere and the block file a box.

`Save to files` in the menu writes the draft back in exactly this format, in a
fixed property order, so the files stay diff-friendly.

<br>

</details>

<details>
<summary><strong>📁 Project Structure</strong></summary>

<br>

```
ft_newton/
│
├── README.md                             # Main project documentation
├── Makefile                              # Build rules: ./newton, run, clean, fclean, re
├── .gitignore                            # objects/, the binary, editor files
│
├── includes/
│   ├── newton.h                          # The only header: every type, then every prototype
│   └── defines.h                         # Every constant, with its unit, and nothing hidden in a .c
│
├── docs/
│   ├── README.md                         # Condensed project documentation
│   ├── valgrind.supp                     # Suppressions for the driver / toolkit stacks
│   └── info/
│       ├── architecture.md               # Files, the main loop, one physics step
│       ├── physics.md                    # Every formula, tagged [F1] to [F26]
│       ├── collisions.md                 # Broad-phase, narrow-phase, response, classic problems
│       ├── rendering.md                  # Window, meshes, camera, draw calls, overlay
│       ├── shaders.md                    # The two shaders, line by line
│       └── object-files.md               # The CSV format and its checks
│
├── assets/                               # The object definitions (CSV)
│   ├── apples.csv                        # Sphere: radius, mass, rolling resistance, material, colour
│   ├── block.csv                         # Box: size, mass, material, colour
│   ├── ground.csv                        # Plane: normal, offset, extent, material, colour
│   └── trebuchet.csv                     # Position, base, frame, arm, counterweight, launch settings
│
├── shaders/
│   ├── basic.vert                        # GLSL 3.30 core: the MVP chain, the world normal
│   └── basic.frag                        # Three modes: lit, flat, and one colour per vertex
│
├── glad/                                 # OpenGL 3.3 Core loader, generated, committed
│   ├── include/glad/gl.h                 # The GL function pointers
│   ├── include/KHR/khrplatform.h         # The types those pointers use
│   └── src/gl.c                          # The loader implementation
│
└── srcs/
    ├── main.c                            # game_init -> game_run -> game_shutdown
    │
    ├── math/
    │   ├── vec3.c                        # Vectors: add, dot, cross, length, normalise
    │   ├── scalar.c                      # clampf, shared by collision, solver and menu
    │   ├── mat3.c                        # Inertia tensors: inverse, transpose, from a quaternion
    │   ├── mat4.c                        # Rendering matrices: transform, perspective, look-at
    │   └── quat.c                        # Quaternions: rotate, compose, shortest arc, integrate
    │
    ├── physics/
    │   ├── rigidbody.c                   # Mass, forces, torque, impulses at a point
    │   ├── inertia.c                     # Sphere and box tensors, and their world-space form
    │   ├── integrator.c                  # One semi-implicit Euler step, damping, orientation
    │   ├── world.c                       # The body array, the scratch arrays, world_step
    │   ├── sleep.c                       # Contact islands and the decision to sleep
    │   └── cull.c                        # Removal of bodies that flew away or fell off the edge
    │
    ├── collision/
    │   ├── collider.c                    # The three shapes, the bounded plane, every AABB
    │   ├── broadphase.c                  # Sweep and prune over the sorted AABBs
    │   ├── narrowphase.c                 # Pair dispatch, normal orientation, contact collection
    │   ├── contact_sphere.c              # Sphere against sphere, plane and box
    │   ├── contact_box.c                 # Box against plane, and box against box
    │   ├── obb.c                         # Oriented-box helpers: axes, radius, corners, support edge
    │   ├── sat.c                         # The separating axis test on 15 axes
    │   ├── manifold.c                    # Face clipping and the reduction to four spread points
    │   └── query.c                       # Touching? overlapping? is this spot free?
    │
    ├── response/
    │   ├── resolver.c                    # Wake, prepare, warm start, iterate, correct, remember
    │   ├── impulse.c                     # Normal, friction and rolling impulses of one contact
    │   └── correction.c                  # Removal of the overlap the impulses left
    │
    ├── render/
    │   ├── window.c                      # GLFW window, OpenGL 3.3 Core context, input queries
    │   ├── shader.c                      # Compiling, linking and the uniforms
    │   ├── mesh.c                        # The unit cube, sphere and plane on the GPU
    │   ├── camera.c                      # Free-flying eye, view and projection
    │   ├── renderer.c                    # Per-frame setup, draw calls, body model and mesh
    │   ├── debugdraw.c                   # The collider wireframes (F1)
    │   ├── font.c                        # The 8x8 bitmap font for ASCII 32..126
    │   └── ui.c                          # The batched 2D overlay: rectangles and text
    │
    ├── menu/
    │   ├── menu.c                        # Rows, sections, buttons and stepping a value
    │   ├── menu_layout.c                 # The 75% panel and the rectangle of every row
    │   ├── menu_input.c                  # Hover, click, hold-to-repeat and the keyboard
    │   └── menu_draw.c                   # Panel, title line and rows, into the overlay
    │
    ├── data/
    │   ├── csvfile.c                     # The strict reader of the object files
    │   ├── csv_values.c                  # Typed getters, unknown-property and range checks
    │   ├── objectdef.c                   # Definition <-> body, and saving the file back
    │   └── trebuchet_file.c              # The trebuchet file: load, check, save
    │
    ├── game/
    │   ├── game.c                        # Start-up, the main loop, the frame cap, shut-down
    │   ├── draw.c                        # Bodies, aim bar, wireframes, menu and title
    │   ├── input.c                       # Every key the game reads
    │   ├── hud.c                         # FPS, counters and live values in the title
    │   ├── scene.c                       # Loading the four files, the checked starting scene
    │   ├── actions.c                     # Apply, reload, save, reset, resume, quit
    │   ├── menu_rows.c                   # What the menu contains
    │   ├── trebuchet.c                   # The five boxes of the machine and the shot
    │   ├── trebuchet_clearance.c         # How far past the arm tip an apple must appear
    │   ├── projectile.c                  # An apple with its launch velocity and mass
    │   └── structure.c                   # Walls, pyramids and towers, lifted onto what is there
    │
    └── utils/
        └── start_check.c                 # The optional [width height] arguments
```

`includes/newton.h` is the only header: every type, then every prototype grouped
by the file that defines it, in pipeline order (math, physics, collision,
response, render, menu, data, game). Every constant lives in
`includes/defines.h` with its unit in a comment, so tuning values never hide
inside a `.c` file, and `srcs/main.c` is twelve lines long.

<br>

</details>

<details>
<summary><strong>🧱 Algorithm Overview</strong></summary>

<br>

### Pipeline

```
assets/*.csv ──> ObjectDef / Trebuchet ──> RigidBody ──> World
                                                            │
                                       world_step (1/120 s) │
                                                            v
        input ──> frame loop ──> draw_frame (OpenGL) <── bodies, colliders
```

### One frame, one step

`game_run` repeats until the window closes or **Quit** is pressed:

1. poll the window events and measure the real frame time, capped at 0.25 s;
2. read the input — the menu while it is open, the game keys otherwise;
3. add `frame_time * time_scale` to an accumulator and drain it in fixed steps
   of 1/120 s, at most 32 per frame (a scene too heavy to keep up drops time
   instead of piling it up);
4. update the FPS average and draw the frame;
5. swap the buffers and sleep off what is left of the 1/30 s frame.

The time scale never changes the step size, only how many steps a frame runs.
At 4x a fast apple is still checked every 1/120 s of simulated time; at 0.25x
the slow motion is exact.

### One physics step

```
integrate -> broad-phase -> narrow-phase -> response -> sleep -> cull
```

| Stage | What it does |
|---|---|
| **Integrate** | Every awake body: `a = F/m + g`, `v += a dt`, `x += v dt` with the **new** velocity, `alpha = I^-1 T`, `w += alpha dt`, damping, quaternion integration, world inertia refreshed |
| **Broad-phase** | AABBs sorted by minimum X; the neighbours that overlap on X are tested on Y and Z; a pair with no awake dynamic body is dropped |
| **Narrow-phase** | The exact test of each candidate, up to four contacts per pair, every normal pointing from `a` to `b` |
| **Response** | Wake what an awake body touches, prepare, warm start, 16 Gauss-Seidel passes (rolling, then friction, then the normal impulse), 3 positional passes, remember |
| **Sleep** | Union-find over the contacts; an island whose bodies all stayed under 5 cm/s and 0.05 rad/s for 0.5 s is switched off |
| **Cull** | Bodies 200 m from the origin, or 20 m under a plane, leave the world and the object counter |

### Collision detection in three questions

```
broad-phase        narrow-phase              response
"who might touch?" "where, how deep?"       "what impulses fix it?"
   AABBs, sweep       SAT, clipping,           sequential impulses,
   and prune          closest point            friction, correction
```

| Pair | Test |
|---|---|
| sphere / sphere | distance between centres against the sum of the radii |
| sphere / plane | signed distance of the centre, and the centre over the square |
| sphere / box | closest point of the box, clamped in the box's own frame |
| box / plane | each of the 8 corners below the surface and over the square |
| box / box | SAT on 15 axes, then face clipping (up to 4 spread points) or an edge contact |

A box resting flat can produce eight candidate points; they are reduced to the
deepest one, the farthest from it and the two spanning the largest area. Four
spread points hold a box flat — four points bunched on one side make it rock.

### Which way the normal points, and why it matters

```
a ─────────────n─────────────> b        normal from the FIRST body toward the SECOND
   ▲                                    pairs are stored with the lower index first,
   └── the dispatcher flips tests that were called with the arguments swapped
```

Every test writes its normal in the same convention, the pairs keep their
orientation from one step to the next, and the solver can therefore reuse last
step's impulses at the same contact point — that is what makes a resting stack
converge instead of being re-solved from zero every step.

### New bodies are never born inside old ones

```
1 2 3 -> x = 8 m        4 -> x = 20 m (150 blocks)
   │                            │
   └── if the spot is taken, the whole structure is lifted
       until no block sinks into what is there, and lands on top
```

A body created with a deep overlap would be pushed apart violently by the
positional correction, so the engine never creates one: structures are lifted
onto what occupies their spot, an apple is refused (with a message) when the
launch point is blocked, and the starting scene is checked when the files are
loaded. "Overlap" means deeper than 1 mm — bodies that merely touch, like blocks
stacked with their gap, are fine.

### Why the scene always comes back to rest

| Problem | How it is handled |
|---|---|
| **Tunneling** | A fixed 1/120 s step whatever the time scale, a launch speed capped at 60 m/s (0.5 m per step, well inside a 1 m block), and a plane solid 2 m deep, so something that sank in is pushed back out |
| **Jitter** | Accumulated impulses with warm starting, contacts kept in the same order, face normals preferred over edge normals unless clearly better, four spread manifold points |
| **Sinking stacks** | Three positional passes per step, so a push travels up a stack within the step |
| **Energy gain** | Correction works on positions, capped at 20 cm per contact and pass; elasticity is read before any impulse of the step; nothing starts overlapping |
| **Endless bouncing and rolling** | A 1 m/s bounce threshold, Coulomb friction, rolling resistance for spheres (which also kills the spin in place that friction cannot see), angular damping, island sleeping |
| **Missed pairs** | A conservative broad-phase (AABBs contain their shapes) that counts touching boxes as overlapping |
| **Stale contact memory** | Removing a body forgets the remembered contacts of its index **and** of the body moved into that slot |
| **Numerical drift** | The quaternion is renormalised every step and the integrator is semi-implicit |

<br>

</details>

<details>
<summary><strong>🕹️ The game layer</strong></summary>

<br>

The engine is the subject; the game is the way to exercise it with something
that can be looked at. All of it goes through the same `World` the engine
exposes to nothing else.

### The trebuchet

Five **static** boxes read from `assets/trebuchet.csv`: a sill, two A-frame legs,
the throwing arm and its counterweight. The pivot sits at `frame_height`, the
throwing side is 72% of the arm, and the launch point is the tip of that side.
`launch_speed` and `launch_angle` come from the file and are the values the keys
and the menu change at runtime.

The spawn distance is not a magic number: `trebuchet_clearance.c` casts a ray
along the firing line against every box of the machine, **grown by the apple's
radius**, at every launch angle the controls allow (1° apart, from -70° to 90°),
keeps the worst exit distance and adds 10 cm. One distance therefore serves
every angle, the aim bar and the spawn point always agree, and an apple can
never start inside its own machine.

### The structures

| Key | Structure | Blocks | Where |
|---|---|---|---|
| `1` | wall | 6 x 6 = 36 | x = 8 m |
| `2` | pyramid | 7 rows, 7+6+5+4+3+2+1 = 28 | x = 8 m |
| `3` | tower | 8 | x = 8 m |
| `4` | large wall | 15 x 10 = 150 | x = 20 m |

Blocks of one structure are placed with a 0.2% gap between them (they are
stacked, not glued), then the whole structure is lifted as one until none of its
blocks sinks into anything already there.

### The menu

`ESC` pauses and opens a panel covering 75% of the window in both directions,
whose text scales with the window. The menu edits a **draft** — a copy of the
definitions and the settings — never the running simulation: moving a value
changes nothing on screen, the title line switches to `APPLY to use the edits`,
and the values reach the world only when **Apply** is pressed. Apply also fixes
the definitions the rebuilt starting scene uses; **Reset simulation** puts
gravity and the time scale back to their defaults.

| Values | Section |
|---|---|
| gravity, time scale | SIMULATION |
| speed, angle | LAUNCH |
| mass, friction, elasticity, rolling resistance | APPLE |
| mass, friction, elasticity | BLOCK |
| friction, elasticity | GROUND |
| friction, elasticity | TREBUCHET |

| Action | What it does |
|---|---|
| **Apply** | Commits the draft: mass and material reach the existing bodies at once, then everything is woken (geometry only reaches bodies created afterwards, or all of them after a reset); prints `Menu values applied to the simulation` |
| **Reload from files** | Re-reads `assets/*.csv` into the simulation and the draft; on failure prints `Reload aborted: fix the object files and try again` and changes nothing |
| **Save to files** | Writes the draft to the four files without applying it; prints `Object files written (the running simulation is unchanged until Apply)` |
| **Reset simulation** | Default gravity and time scale, and the starting scene rebuilt; prints `Simulation reset` |
| **Resume** | Closes the menu and unpauses |
| **Quit** | Leaves the loop and frees everything |

Click `[-]` / `[+]` and hold to repeat (after 0.35 s, every 0.06 s); the mouse
highlights rows and buttons, the arrows move the selection, the left and right
arrows change the selected value, and `ENTER` presses the selected button. The
whole panel — panel, text, buttons and states — is drawn by the engine's own 2D
overlay, which packs one colour per vertex so the entire menu is a single draw
call.

### Reading the scene

`F1` draws every collider as a flat yellow wireframe over the solid render,
1% larger (the plane lifted 1 cm above the ground) so it is never hidden by the
body it describes. It is the view that shows why a pile holds: the ground's
wireframe is exactly the square that supports bodies, and past it they fall and
are eventually removed from the world and the counter.

<br>

</details>

## 💡 Key Learning Outcomes

- **Rigid-body dynamics**: mass and inertia as `1/m` and `I^-1`, torque from a
  force applied off-centre, impulses that change both the velocity and the spin
  — and the realisation that rotation is not a special case bolted on, but the
  same laws acting at a point away from the centre
- **Quaternions**: integrating an angular velocity, renormalising every step
  and rotating the inertia tensor into world space instead of storing it in
  fifteen numbers
- **Collision detection**: why a broad-phase is not optional (n²/2 pairs become
  `n log n` plus what actually overlaps), what the 15 axes of a box-box test
  are, and how clipping turns a face contact into a usable manifold
- **Contact manifolds**: one contact per box-box pair is not enough to hold a
  box flat, and eight are not needed — the four spread points are
- **Sequential impulses**: effective mass along a direction, clamping the
  *accumulated* impulse rather than each increment, warm starting, and reading
  elasticity before the step's impulses so warm starts do not look like impacts
- **Friction and rolling**: Coulomb friction on two tangents, and the two blind
  spots of a rigid sphere — rolling without slipping and spinning in place —
  which is why a rolling-resistance term exists at all
- **Making a simulation settle**: bounce threshold, positional correction on
  positions instead of velocities, angular damping, and islands that sleep as a
  whole because sleeping one box of a pile removes its neighbours' support
- **Numerical stability**: fixed timestep, semi-implicit Euler, capped
  corrections, and the arithmetic of anti-tunneling (60 m/s ÷ 120 Hz = 0.5 m,
  which is half of the thinnest body)
- **Defensive loading**: a data format with no fallback values, errors named by
  file and line, and a starting scene refused instead of repaired
- **Immediate-mode UI on a 3D renderer**: a bitmap font, a batched vertex
  buffer, and a hit-test that runs on the same rectangles the layout produced
- **Checking your own work**: wireframes, pause, slow motion and valgrind with a
  suppression file that names exactly what the driver stack is allowed to leak

## ⚙️ Technical Specifications

- **Language**: C, compiled with `cc -Wall -Wextra -Werror -g3 -O3`, no `-std`
  flag beyond the compiler's default; GLSL 3.30 core for the two shaders
- **Size**: ~5 000 lines of C in `srcs/` (52 files), 916 lines of headers
  (`newton.h` 756, `defines.h` 160) and 53 lines of shaders
- **Dependencies**: GLFW 3, OpenGL 3.3, `libdl`, `pthread`, `libm`. GLAD is
  vendored in `glad/`. **No physics, collision or matrix library is used.**
- **Rendering**: OpenGL 3.3 Core through GLFW + GLAD, one shader program, three
  meshes (unit cube, sphere with 32 segments, plane), depth test on, no vsync
  (the frame cap governs), clear colour `0.10, 0.11, 0.13`
- **Shading**: a directional light with 0.25 ambient for the scene, a flat mode
  for the wireframes and the aim bar, and a per-vertex-colour mode for the 2D
  overlay — one fragment shader serving all three
- **Camera**: free-flying, 60° vertical field of view, 0.1 to 500 m, fly speed
  2 to 60 m/s, pitch clamped to ±89°, `WASD` / `RF` / arrows
- **Simulation**: fixed 1/120 s semi-implicit Euler, at most 32 steps per frame,
  gravity -9.81 m/s² by default
- **Solver**: 16 Gauss-Seidel passes per step, 3 positional passes, at most 4
  contacts per pair, warm starting within 5 cm, elasticity below 1 m/s thrown
  away, friction `sqrt(mu_a mu_b)`, correction slop 5 mm at 20% per pass capped
  at 20 cm
- **Sleeping**: 0.05 m/s and 0.05 rad/s for 0.5 s, per contact island, computed
  with union-find over the contacts
- **World bounds**: bodies are removed 200 m from the origin, or 20 m under a
  plane; a plane is solid 2 m beneath its surface
- **Broad-phase**: sweep and prune, AABBs sorted by minimum X, `O(n log n)` plus
  the pairs that actually overlap
- **Narrow-phase**: 15-axis SAT for boxes with a face-normal bias (5% and 5 mm),
  Sutherland-Hodgman clipping, edge contact in the box-box case
- **Data**: four strict CSV files, plain decimals only, every error reported as
  `file:line: reason`, and no default value anywhere
- **Window**: 1280x720 by default, accepted sizes 720x480 to 2560x1440, frame
  cap 30 FPS with the simulation independent of it, title readout every 0.2 s
- **Overlay**: one vertex buffer of 196 608 floats (6 per vertex, position +
  colour), enough for the whole menu in one draw call
- **Memory**: nothing per body on the GPU — three meshes serve the whole scene —
  and the CPU containers (bodies, pairs, contacts, warm-start memory and the
  per-step scratch: island parents, island timers, start positions, rotation
  deltas) only grow with the body count. The single allocation of a physics step
  is the broad-phase sweep array, freed before the solver runs; every path
  releases through `game_shutdown`
- **Verification**: `docs/valgrind.supp` suppresses only the GL / GTK / Mesa
  driver stacks, so what valgrind reports is the program's own memory

## 🔧 Requirements

- A C compiler (`cc`, `gcc` or `clang`) and GNU `make`
- **GLFW 3** development files and an OpenGL 3.3 Core driver: `pacman -S glfw
  mesa` on Arch, `apt install libglfw3-dev libgl1-mesa-dev` on Debian/Ubuntu
- A desktop session: the program opens a window and repeatedly draws into it
- Nothing else to install: GLAD is committed in `glad/`, and the shaders and the
  object files are read from the repository, so run the binary from its root

---

> [!NOTE]
> ft_newton is a small engine with a large amount of physics behind it: the fun
> is not that apples fly in an arc, it is that the arc, the bounce, the slide,
> the roll and the pile that finally stops moving are all the same forty lines
> of maths run 120 times a second, checked by a collision pipeline that has to
> be right about every one of them.
