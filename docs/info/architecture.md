# Architecture

How the program is organised, what every file is responsible for, and how a frame and a physics step run.

## Principles

- **Two headers.** `includes/newton.h` holds every type and every prototype, grouped by source folder in pipeline order. `includes/defines.h` holds every constant, each group titled and each value with its unit, so no tuning value is hidden in a `.c` file.
- **One responsibility per file.** Each `.c` does one job and its name says which. Folders follow the data flow: maths, then physics, collision, response, then rendering, menu, data files and the game.
- **Small types by value, stateful objects by pointer.** `Vec3`, `Quat`, `Mat3` and `Mat4` travel by value; `World`, `RigidBody`, `Menu` and `Game` travel by pointer.
- **Every formula is tagged.** A physics line carries a tag such as `[F21]`, and [physics.md](physics.md) explains the same tag, so `grep -rn "\[F21\]" srcs/` finds the code of a formula.

## Source map

### `srcs/math/`: the numbers

| File | Responsibility |
|---|---|
| `vec3.c` | 3D vectors: add, subtract, scale, dot, cross, length, normalize, component-wise min and max |
| `scalar.c` | `clampf`, shared by collision, solver and menu |
| `mat3.c` | 3x3 matrices for inertia tensors and joints: product, sum, scale, transpose, inverse, cross-product matrix, rotation from a quaternion |
| `mat4.c` | 4x4 matrices for rendering: translation, scale, rotation, perspective, look-at |
| `quat.c` | Quaternions: rotation of vectors, composition, shortest arc, integration of an angular velocity |

### `srcs/physics/`: bodies that move

| File | Responsibility |
|---|---|
| `rigidbody.c` | A body's mass, and how forces, impulses and torque impulses change its motion |
| `inertia.c` | Inertia tensors of spheres, boxes and meshes, and their rotation into world space |
| `mesh_inertia.c` | Volume, centre of mass and inertia tensor of a closed triangle surface |
| `integrator.c` | One semi-implicit Euler step: forces to velocity to position, torque to spin to orientation |
| `world.c` | The body container (add, remove, clear, wake) and `world_step`, which runs the whole pipeline |
| `joint_list.c` | The joint container: a joint made from the current pose, renumbered when bodies are removed, and the jointed pairs that do not collide |
| `sleep.c` | Islands (bodies linked by contacts or joints) and the decision to put a resting island to sleep |
| `cull.c` | Removal of bodies that flew too far away or fell off the edge of the ground |

### `srcs/collision/`: who touches whom

| File | Responsibility |
|---|---|
| `collider.c` | Collider shapes, the bounded plane (its frame, distance, extent) and every body's AABB |
| `broadphase.c` | Sweep and prune: the candidate pairs whose AABBs overlap, minus the jointed ones |
| `narrowphase.c` | Dispatches each pair to the right contact test and collects the contacts |
| `contact_sphere.c` | Sphere against sphere, plane and box |
| `contact_box.c` | Box against plane and box against box (face clipping or edge contact) |
| `contact_mesh.c` | Mesh against sphere, plane and mesh (the vertices of each inside the other, and the edges of each crossing the other's surface) |
| `contact_box_mesh.c` | Box against mesh: the 13-axis test of each nearby triangle, then clipping or an edge contact |
| `mesh_shape.c` | A model turned into a collider: loaded, scaled, repaired, centred on its centre of mass, indexed; rescaled in place when the size changes |
| `mesh_repair.c` | Making a surface closed and outward: welding split vertices, dropping repeated faces, orienting each piece, closing holes |
| `mesh_topology.c` | Closed and consistently wound surfaces: neighbours of every triangle, face and vertex normals |
| `bvh.c` | The bounding volume tree of a mesh: build and query |
| `closest.c` | Closest points on a triangle and between two segments, and where a segment crosses a triangle |
| `mesh_query.c` | The surface point closest to a point, and whether that point is inside |
| `obb.c` | Oriented-box helpers: axes, projected radius, corners, support edge, incident face |
| `sat.c` | The separating axis test between two boxes |
| `manifold.c` | Polygon clipping, the candidate set of a mesh pair and the reduction of many contact points to four well spread ones |
| `overlap.c` | Questions asked outside the step: does a body sink into another one, which body occupies a spot |

### `srcs/response/`: what contacts do

| File | Responsibility |
|---|---|
| `resolver.c` | One step of response: wake, prepare joints and contacts, warm start, iterate, correct, project the joints, remember |
| `impulse.c` | The impulses of one contact: normal (with elasticity), friction, rolling resistance; the mass ratio limit of a light body holding up a heavy one |
| `joint_point.c` | The shared point of a joint: its 3x3 effective mass, impulse, positional correction and projection, and the joint's friction |
| `joint_limit.c` | The angles of a joint: swing, twist and hinge angle, locks and limits as angular rows, and their projection |
| `correction.c` | Removal of the overlap and the joint drift the impulses left, by moving and turning the bodies |

### `srcs/render/`: pixels

| File | Responsibility |
|---|---|
| `window.c` | GLFW window and OpenGL context, framebuffer size, cursor, mouse and keys |
| `shader.c` | Loads, compiles and links `shaders/`, sets uniforms |
| `mesh.c` | The unit cube, sphere and square, and the triangles of each model, uploaded to the GPU |
| `camera.c` | The free-flying camera and its view and projection matrices |
| `renderer.c` | Per-frame setup, draw calls, and the model matrix and mesh of a body |
| `debugdraw.c` | The collider wireframes and the contact points with their normals (`F1`) |
| `font.c` | The 8x8 bitmap font |
| `ui.c` | The batched 2D overlay: rectangles and text in one draw call |

### `srcs/menu/`: the pause menu as a widget

| File | Responsibility |
|---|---|
| `menu.c` | Rows (sections, values, buttons, model buttons, hints) and stepping a value |
| `menu_layout.c` | The 75% panel, the columns in use and the position of every row, recomputed every frame |
| `menu_input.c` | Mouse hover, click and hold-to-repeat; keyboard navigation |
| `menu_draw.c` | Panel, title line and rows, drawn into the overlay |

### `srcs/data/`: files on disk

| File | Responsibility |
|---|---|
| `csv_file.c` | Reads and parses an object file, row by row |
| `csv_values.c` | Typed getters and range checks, each problem reported with its line |
| `object_def.c` | apples / block / ground / models files to object definitions, the primitive bodies built from them, and saving them back |
| `trebuchet_file.c` | The trebuchet file: load, validate, save |
| `ragdoll_file.c` | The ragdoll file: load, validate, save |
| `obj_load.c` | The `.obj` reader: vertices first, then the faces |
| `obj_face.c` | One face line: its corners resolved to vertex indices |
| `obj_triangulate.c` | Polygons to triangles by ear clipping |

### `srcs/game/`: the application

| File | Responsibility |
|---|---|
| `game.c` | Start-up, the main loop with its fixed step and frame cap, shut-down |
| `draw.c` | What one frame shows: bodies, aim bar, wireframes, menu, title |
| `input.c` | Every key the game reads |
| `hud.c` | FPS and object counters and live values in the window title |
| `scene.c` | Loading the object files, building the starting scene, re-stamping mass and materials |
| `scene_check.c` | The starting scene built aside before the window opens: trebuchet and pyramid over the ground and not sunk into it, no overlap, a free apple spawn point, a camera above the ground and outside every body |
| `model_library.c` | Every `.obj` of `assets/models/`, found and loaded once at start-up in name order, and the selected one |
| `actions.c` | What the menu buttons do: sync the draft, apply, reload, save, reset, resume, quit, open the models page, place a model |
| `menu_rows.c` | The two menu pages: the values, keys and buttons of the main page, and the list of models |
| `trebuchet.c` | The five boxes of the machine and the shot |
| `trebuchet_clearance.c` | How far past the arm tip an apple must appear to clear the machine |
| `projectile.c` | An apple with its launch velocity and mass |
| `placement.c` | New bodies lifted as one group onto whatever occupies their spot, then added |
| `camera_collision.c` | The camera as a 0.3 m sphere: moved in 10 cm steps, pushed out of every body, out of a model that swallows it through the closest surface point, to the nearest free spot when pinned |
| `structure.c` | Walls, pyramids and towers of blocks |
| `ragdoll.c` | The eleven parts and ten joints of the ragdoll |
| `model.c` | A model body: its surface, its mass from the shared density, its material; every placed model resized at once |
| `rain.c` | The test rain that shows where a collider is |

### `srcs/utils/`: helpers

| File | Responsibility |
|---|---|
| `arguments.c` | The optional `[width height]` arguments |
| `array.c` | Growable arrays for the loaders |
| `file.c` | A whole file in memory: shaders and models |

## Data flow

```
assets/*.csv --csv_file/csv_values--> ObjectDef / Trebuchet --object_def_make_body--> RigidBody --> World
assets/models/*.obj --model_library--> MeshShape --model_spawn----------------------------^          |
                                           ^                                                        |
                           menu Draft --Apply-+                                     world_step (physics)
                                                                                                    |
                                                     window <-- draw_frame (render) <----------------+
```

1. At start-up `scene_load_assets` reads the object files. Each file becomes a definition and the starting scene built from them is checked (`scene_check.c`) before anything is accepted: every problem is reported with its files and position, and the program exits with status 1. Then `model_library_load` reads every `.obj` of `assets/models/` at the size of `assets/models.csv`; a model that cannot be used is reported and skipped.
2. `scene_build` turns the definitions into bodies: the ground, five static trebuchet boxes and a 5-row pyramid.
3. Every frame the keys may change the live values (`input.c`), the world advances by fixed steps, the camera is pushed out of whatever reached it (`camera_collision.c`), and `draw_frame` renders it.
4. The menu works on a `Draft`, a copy of the definitions and settings. Only **Apply** copies it back, and `scene_apply_definitions` re-stamps mass and material onto existing bodies of each kind (`RigidBody.kind`; a model takes the shared density times its own volume, a ragdoll part its share of the ragdoll's mass).

## The main loop

`game_run` repeats until the window closes or **Quit** is pressed:

1. poll the window events and measure the real frame time (capped at 0.25 s);
2. read the input: the menu while it is open, the game keys otherwise (the camera flies in steps of 10 cm and cannot enter a body);
3. advance the simulation: the frame time times the time scale feeds an accumulator, drained in fixed steps of 1/120 s (at most 32 per frame; beyond that the scene slows down instead of stalling);
4. push the camera out of any body that moved, fell or appeared where it stands;
5. update the FPS average and draw the frame;
6. swap buffers and sleep off what is left of the 1/30 s frame.

The time scale never changes the step size, only how many steps a frame runs. At 4x a fast apple is still checked every 1/120 s of simulated time; at 0.25x the slow motion is exact.

## One physics step

`world_step(w, dt)`:

1. **Integrate** every awake body: gravity and forces to velocity to position, torque to angular velocity to orientation, then refresh the world inertia tensor.
2. **Broad-phase**: AABBs sorted along X; overlapping pairs with at least one awake dynamic body become candidates, except two ragdoll parts their joint keeps apart.
3. **Narrow-phase**: the exact test of each candidate gives contact points (normal, point, depth), at most four per pair; a mesh is tested triangle by triangle through its tree.
4. **Response**: sequential impulses over all joints and contacts (16 passes), then positional correction of overlaps and joint drift (3 passes), then joint projection (what is left beyond 1 cm or 5°).
5. **Sleep**: islands of bodies (linked by contacts or joints) at rest for 0.5 s are switched off.
6. **Cull**: bodies 200 m away, or 20 m under the ground after falling off its edge, are removed.

## Where to find things

| Looking for | Where |
|---|---|
| The FPS and object counters | `srcs/game/hud.c` |
| Gravity, the step size, solver passes, thresholds | `includes/defines.h` |
| A physics formula | [physics.md](physics.md), then `grep -rn "\[Fn\]" srcs/` |
| The ragdoll's layout and joint limits | `srcs/game/ragdoll.c`, `includes/defines.h` (`RAGDOLL_*`) |
| How a model becomes a collider | `srcs/collision/mesh_shape.c` and [collisions.md](collisions.md#mesh-colliders) |
| Why a body bounces, slides or rolls the way it does | `srcs/response/impulse.c` |
| Why something touches something | `srcs/collision/` and [collisions.md](collisions.md) |
| The keys | `srcs/game/input.c` (the menu's list of them is `srcs/game/menu_rows.c`) |
| Why the camera stops at a body | `srcs/game/camera_collision.c` |
| Why a starting scene is refused | `srcs/game/scene_check.c` |
| The models and the selected one | `srcs/game/model_library.c`, `assets/models/`, `assets/models.csv` |
| The menu's look, size and pages | `srcs/menu/menu_layout.c`, `srcs/menu/menu_draw.c`, `srcs/game/menu_rows.c` |
| The shaders and how they are used | `shaders/`, `srcs/render/`, [shaders.md](shaders.md) |
| The file format | `srcs/data/`, [object-files.md](object-files.md) |
