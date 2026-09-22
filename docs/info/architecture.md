# Architecture

How the program is organised, what every file is responsible for, and how a frame and a physics step run.

## Principles

- **One header.** `includes/newton.h` holds every type and every prototype, grouped by the file that defines them, in pipeline order. `includes/defines.h` holds every constant with its unit, so no tuning value is hidden in a `.c` file.
- **One responsibility per file.** Each `.c` does one job and its name says which. Folders follow the data flow: maths, then physics, collision, response, then rendering, menu, data files and the game.
- **Small types by value, stateful objects by pointer.** `Vec3`, `Quat`, `Mat3` and `Mat4` travel by value; `World`, `RigidBody`, `Menu` and `Game` travel by pointer.
- **Every formula is tagged.** A physics line carries a tag such as `[F21]`, and [physics.md](physics.md) explains the same tag, so `grep -rn "\[F21\]" srcs/` finds the code of a formula.

## Source map

### `srcs/math/`: the numbers

| File | Responsibility |
|---|---|
| `vec3.c` | 3D vectors: add, subtract, scale, dot, cross, length, normalize |
| `scalar.c` | `clampf`, shared by collision, solver and menu |
| `mat3.c` | 3x3 matrices for inertia tensors: product, transpose, inverse, rotation from a quaternion |
| `mat4.c` | 4x4 matrices for rendering: translation, scale, rotation, perspective, look-at |
| `quat.c` | Quaternions: rotation of vectors, composition, shortest arc, integration of an angular velocity |

### `srcs/physics/`: bodies that move

| File | Responsibility |
|---|---|
| `rigidbody.c` | A body's mass, and how forces, impulses and torque impulses change its motion |
| `inertia.c` | Inertia tensors of spheres and boxes, and their rotation into world space |
| `integrator.c` | One semi-implicit Euler step: forces to velocity to position, torque to spin to orientation |
| `world.c` | The body container (add, remove, clear, wake) and `world_step`, which runs the whole pipeline |
| `sleep.c` | Contact islands and the decision to put a resting island to sleep |
| `cull.c` | Removal of bodies that flew too far away or fell off the edge of the ground |

### `srcs/collision/`: who touches whom

| File | Responsibility |
|---|---|
| `collider.c` | Collider shapes, the bounded plane (its frame, distance, extent) and every body's AABB |
| `broadphase.c` | Sweep and prune: the candidate pairs whose AABBs overlap |
| `narrowphase.c` | Dispatches each pair to the right contact test and collects the contacts |
| `contact_sphere.c` | Sphere against sphere, plane and box |
| `contact_box.c` | Box against plane and box against box (face clipping or edge contact) |
| `obb.c` | Oriented-box helpers: axes, projected radius, corners, support edge |
| `sat.c` | The separating axis test between two boxes |
| `manifold.c` | Polygon clipping and the reduction of many contact points to four well spread ones |
| `query.c` | Questions asked outside the step: do two bodies touch or overlap, is a spot free |

### `srcs/response/`: what contacts do

| File | Responsibility |
|---|---|
| `resolver.c` | One step of contact response: wake, prepare, warm start, iterate, correct, remember |
| `impulse.c` | The impulses of one contact: normal (with elasticity), friction, rolling resistance |
| `correction.c` | Removal of the overlap the impulses left, by moving and turning the bodies |

### `srcs/render/`: pixels

| File | Responsibility |
|---|---|
| `window.c` | GLFW window and OpenGL context, framebuffer size, cursor, mouse and keys |
| `shader.c` | Loads, compiles and links `shaders/`, sets uniforms |
| `mesh.c` | The unit cube, sphere and square uploaded to the GPU |
| `camera.c` | The free-flying camera and its view and projection matrices |
| `renderer.c` | Per-frame setup, draw calls, and the model matrix and mesh of a body |
| `debugdraw.c` | The collider wireframes (`F1`) |
| `font.c` | The 8x8 bitmap font |
| `ui.c` | The batched 2D overlay: rectangles and text in one draw call |

### `srcs/menu/`: the pause menu as a widget

| File | Responsibility |
|---|---|
| `menu.c` | Rows (sections, values, buttons, hints) and stepping a value |
| `menu_layout.c` | The 75% panel and the position of every row, recomputed every frame |
| `menu_input.c` | Mouse hover, click and hold-to-repeat; keyboard navigation |
| `menu_draw.c` | Panel, title line and rows, drawn into the overlay |

### `srcs/data/`: files on disk

| File | Responsibility |
|---|---|
| `csvfile.c` | Reads and parses an object file, row by row |
| `csv_values.c` | Typed getters and range checks, each problem reported with its line |
| `objectdef.c` | apples / block / ground files to object definitions and bodies; saving them back |
| `trebuchet_file.c` | The trebuchet file: load, validate, save |

### `srcs/game/`: the application

| File | Responsibility |
|---|---|
| `game.c` | Start-up, the main loop with its fixed step and frame cap, shut-down |
| `draw.c` | What one frame shows: bodies, aim bar, wireframes, menu, title |
| `input.c` | Every key the game reads |
| `hud.c` | FPS and object counters and live values in the window title |
| `scene.c` | Loading the four files, checking and building the starting scene, re-stamping materials |
| `actions.c` | What the menu buttons do: sync the draft, apply, reload, save, reset, resume, quit |
| `menu_rows.c` | The content of the menu: which values, which keys, which buttons |
| `trebuchet.c` | The five boxes of the machine and the shot |
| `trebuchet_clearance.c` | How far past the arm tip an apple must appear to clear the machine |
| `projectile.c` | An apple with its launch velocity and mass |
| `structure.c` | Walls, pyramids and towers, lifted onto whatever occupies their spot |

## Data flow

```
assets/*.csv --csvfile/csv_values--> ObjectDef / Trebuchet --objectdef_make_body--> RigidBody --> World
                                          ^                                                        |
                          menu Draft --Apply-+                                     world_step (physics)
                                                                                                   |
                                                    window <-- draw_frame (render) <----------------+
```

1. At start-up `scene_load_assets` reads the four files. Each file becomes a definition, and the starting scene built from them is checked before anything is accepted.
2. `scene_build` turns the definitions into bodies: the ground, five static trebuchet boxes and a 5-row pyramid.
3. Every frame the keys may change the live values (`input.c`), the world advances by fixed steps, and `draw_frame` renders it.
4. The menu works on a `Draft`, a copy of the definitions and settings. Only **Apply** copies it back, and `scene_apply_definitions` re-stamps mass and material onto existing bodies of each kind (`RigidBody.kind`).

## The main loop

`game_run` repeats until the window closes or **Quit** is pressed:

1. poll the window events and measure the real frame time (capped at 0.25 s);
2. read the input: the menu while it is open, the game keys otherwise;
3. advance the simulation: the frame time times the time scale feeds an accumulator, drained in fixed steps of 1/120 s (at most 32 per frame; beyond that the scene slows down instead of stalling);
4. update the FPS average and draw the frame;
5. swap buffers and sleep off what is left of the 1/30 s frame.

The time scale never changes the step size, only how many steps a frame runs. At 4x a fast apple is still checked every 1/120 s of simulated time; at 0.25x the slow motion is exact.

## One physics step

`world_step(w, dt)`:

1. **Integrate** every awake body: gravity and forces to velocity to position, torque to angular velocity to orientation, then refresh the world inertia tensor.
2. **Broad-phase**: AABBs sorted along X; overlapping pairs with at least one awake dynamic body become candidates.
3. **Narrow-phase**: the exact test of each candidate gives contact points (normal, point, depth), at most four per pair.
4. **Response**: sequential impulses over all contacts (16 passes), then positional correction (3 passes).
5. **Sleep**: islands of bodies at rest for 0.5 s are switched off.
6. **Cull**: bodies 200 m away, or 20 m under the ground after falling off its edge, are removed.

## Where to find things

| Looking for | Go to |
|---|---|
| The FPS and object counters | `srcs/game/hud.c` |
| Gravity, the step size, solver passes, thresholds | `includes/defines.h` |
| A physics formula | [physics.md](physics.md), then `grep -rn "\[Fn\]" srcs/` |
| Why a body bounces, slides or rolls the way it does | `srcs/response/impulse.c` |
| Why something touches something | `srcs/collision/` and [collisions.md](collisions.md) |
| The keys | `srcs/game/input.c` (the menu's list of them is `srcs/game/menu_rows.c`) |
| The menu's look and size | `srcs/menu/menu_layout.c`, `srcs/menu/menu_draw.c` |
| The shaders and how they are used | `shaders/`, `srcs/render/`, [shaders.md](shaders.md) |
| The file format | `srcs/data/`, [object-files.md](object-files.md) |
