# ft_newton

ft_newton is a 3D rigid-body physics engine written from scratch in C, in the spirit of PhysX, Havok or Bullet, shown through a small game: a trebuchet throws apples at unstable structures of blocks, and every reaction (translation, rotation, bounces, friction, rolling, gravity) is computed by the engine itself.

The scene is drawn with OpenGL 3.3 Core through two hand-written shaders; GLFW provides the window and the input, and GLAD loads the OpenGL functions. All the mathematics (vectors, matrices, quaternions, inertia tensors) and all the physics (integration, collision detection, contact response, joints) live in the project, with no physics, collision or maths library. The game happens on the XY plane, but the engine and the camera are fully three-dimensional.

## Dependencies

| Dependency | Version | Debian / Ubuntu | Arch |
|---|---|---|---|
| C compiler and GNU Make | gcc or clang | `build-essential` | `base-devel` |
| `pkg-config` | any | `pkg-config` | `pkgconf` |
| GLFW 3 | 3.3 or newer | `libglfw3-dev` | `glfw` |
| OpenGL | 3.3 Core, Mesa or a vendor driver | `libgl1-mesa-dev` | `mesa` |
| `valgrind` | optional, for `make valgrind` | `valgrind` | `valgrind` |

GLAD, the OpenGL loader, is included in `glad/` and built with the project. `make install` checks the compiler, `pkg-config`, GLFW and OpenGL, and names whatever is missing.

## Build and run

```
make install    # checks the dependencies
make            # builds ./newton with -O3; O=0, 1, 2, s or g sets another level
make run        # builds and runs; ARGS="1920 1080" sets the window size
make valgrind   # builds and runs under valgrind with the suppression files
make clean      # removes the objects
make fclean     # removes the objects and the binary
make re         # fclean, then build
make help       # this list
```

The project sources are compiled with `-Wall -Wextra -Werror -g3`; the objects are rebuilt whenever the compile flags change (`objects/.flags`), so switching `O` never mixes levels. The binary runs from the repository root, where `shaders/` and `assets/` are found:

```
./newton                # 1280x720
./newton 1920 1080      # any size from 720x480 to 2560x1440
```

A size outside those bounds, or not made of digits, is reported and replaced by the default; any other number of arguments prints the usage and exits with status 1. The main loop is capped at 30 frames per second, and the physics runs on its own fixed 120 Hz step.

| Key | Action |
|---|---|
| `ESC` | Menu (pauses the simulation); from the models page, back to the main page |
| `SPACE` | Fire an apple (refused while the spawn point is occupied) |
| `1` `2` `3` | Wall (6x6), pyramid (7 rows), tower (8 blocks) at x = 8 m |
| `4` | Large wall, 150 blocks, at x = 20 m |
| `5` | Ragdoll at x = 2 m |
| `6` | The selected model at x = 2 m |
| `7` `8` | Next / previous model (its name is in the window title) |
| `9` | Test rain: 196 small balls on whatever stands at x = 2 m |
| `P` | Pause / resume |
| `F1` | Collider wireframes and contact points |
| `H` | Window title readout |
| `I` `K`, `J` `L`, `Q` `E` | Launch angle, launch speed, mass of the next apples |
| `G` `B`, `T` `Y` | Gravity, time scale |
| `0` | Gravity off, then back to its previous value |
| `W` `A` `S` `D`, `R` `F`, arrows, `+` `-` | Camera: fly, up / down, turn, fly speed |

Every new body appears at a fixed spot and is lifted onto whatever already stands there, so no body ever starts inside another one. The camera is a 0.3 m sphere that collides with the scene: it slides along the bodies it flies into, stops at a gap narrower than itself, and is pushed out by any body that moves, falls or appears where it stands.

## How it works

- **Colliders**: sphere, box, plane and mesh. The plane is limited to the drawn ground square, and a body that falls past its edge is removed. A mesh is the exact triangle surface of an `.obj` model, with its mass and inertia taken from the volume it encloses.
- **Integration**: semi-implicit Euler on a fixed 1/120 s step, quaternion orientations and a world-space inertia tensor. The time scale changes how many steps a frame runs, never their size.
- **Collision detection**: a sweep-and-prune broad-phase, then an exact test for every shape pair. Box against box uses the separating axis theorem on 15 axes and face clipping; a mesh is tested triangle by triangle through a bounding volume tree, and two meshes by the vertices of each inside the other and the edges of each crossing the other's surface.
- **Response**: a sequential-impulse solver with accumulated clamping, warm starting, elasticity, Coulomb friction and rolling resistance, followed by a positional correction that removes the leftover overlap without adding energy. Where a light body holds up one more than ten times heavier, the light body counts as only ten times lighter (and never more than ten times its own mass), so a heavy model rests on a ragdoll instead of sinking into it.
- **Joints**: a ball and socket with cone and twist limits, and a hinge with an angle range, solved with the contacts. Whatever a step still leaves beyond 1 cm at an anchor or 5° past a limit is projected back, so a violent impact or a heavy load never pulls a ragdoll apart.
- **Sleeping**: a pile, or a ragdoll, that has come to rest switches off as a whole island, so every impact ends in a stable state; a body shaking in place without going anywhere, or an island creeping very slowly under a heavy load, is put to sleep too.
- **Object files**: apples, blocks, ground, trebuchet, ragdoll and models are described by CSV files in `assets/`, loaded strictly. A wrong value or an unknown property is reported with its file and line, and the program does not start.
- **Starting scene**: built aside from the files before the window opens. A trebuchet or a pyramid that would sink into the ground, overlap each other or stand off the ground square, an apple spawn point taken by another body, and a camera that would start under the ground or inside a body are each reported with the files involved and the position, and the program exits with status 1. **Reload from files** refuses the same files and keeps the running scene.
- **Camera**: moved in steps of 10 cm at most, a third of its radius, so it never crosses a thin body; after every step and every frame it is pushed out along the deepest contact with each body, a centre swallowed by a model leaves through the closest point of its surface [F30], and a camera pinned between bodies moves to the nearest free spot (up or sideways, never down).

Every formula carries a tag such as `[F15]` in the source, and the topic documents explain each one:

| Document | Contents |
|---|---|
| `docs/info/architecture.md` | Source tree, main loop, one physics step |
| `docs/info/physics.md` | Every formula, tagged `[F1]` to `[F39]`, and where it lives |
| `docs/info/collisions.md` | Broad-phase, narrow-phase, mesh colliders, response, sleeping, spawning without overlaps, the precision of a mesh collider |
| `docs/info/rendering.md` | Window, meshes, camera, draw calls, debug wireframes, menu overlay |
| `docs/info/shaders.md` | The two shaders, line by line |
| `docs/info/object-files.md` | The CSV format, the `.obj` models, their repair and their checks |

## Main formulas

| Tag | Formula | Use |
|---|---|---|
| [F1] | `a = F / m + g`, `alpha = I_world^-1 T` | Newton's second law, linear and angular |
| [F2] | `v += a dt`, then `x += v dt`; `w += alpha dt` | Semi-implicit Euler on a fixed 1/120 s step |
| [F3] | `q = normalize(q + 0.5 (0, w) q dt)` | Orientation as a quaternion |
| [F5] [F6] | sphere `I = 2/5 m R²`; box `Ixx = m/3 (hy² + hz²)` and the same on the other axes | Inertia of the primitives |
| [F7] | `I_world^-1 = R I_local^-1 R^T` | Inertia turned with the body |
| [F9] | `v += J / m`, `w += I_world^-1 (r x J)` | An impulse applied at a point |
| [F11] | `min_a <= max_b` and `min_b <= max_a` on X, Y and Z | Broad-phase overlap of two bounding boxes |
| [F15] | 15 axes (3 + 3 face normals, 9 edge cross products); the least overlap gives the normal and the depth | Box against box (separating axis theorem) |
| [F19] | `1/m_n = 1/m_a + 1/m_b + n . ((I_a^-1 (r_a x n)) x r_a + (I_b^-1 (r_b x n)) x r_b)` | Effective mass of a contact |
| [F20] | `bias = -e v_rel_n` when approaching faster than 1 m/s, `e = max(e_a, e_b)` | Elasticity (Newton's impact law) |
| [F21] | `j_n = max(j_n + m_n (bias - v_rel_n), 0)` | Normal impulse, accumulated and clamped |
| [F22] | `abs(j_t) <= mu j_n`, `mu = sqrt(mu_a mu_b)` | Coulomb friction, static and kinetic |
| [F23] | `c = min(max(depth - slop, 0), max_correction) percent m_n`, bodies moved by `c n / m` | Positional correction, no energy added |
| [F25] | `v0 = speed (cos angle, sin angle, 0)` | Launch velocity; gravity then draws the parabola |
| [F27] | `V = sum of d_t.x (x0 + x1 + x2) / 6` over the triangles, and likewise the centre of mass and the full inertia tensor (divergence theorem) | Mass properties of a mesh |
| [F30] | a point is inside when it lies behind the angle-weighted pseudo-normal of its closest feature | Inside test of a mesh collider |

## Test scenes

The launch values are set with `I` `K` (angle) and `J` `L` (speed), or in the menu. At x = 2 m (ragdoll, models, test rain), an apple hits at -10° and 11 m/s, -25° and 20 m/s, or -30° and 40 to 60 m/s; the default shot (25°, 11 m/s) hits the starting pyramid.

| Scene | Keys and values | Result |
|---|---|---|
| Throw | `SPACE` | The apple follows a parabola and knocks the pyramid at x = 8 m; every block comes back to rest and sleeps |
| Gravity | `0`, then `SPACE`; `G` `B` | Without gravity the apple flies in a straight line and nothing falls; `0` again restores the previous value (-9.81 m/s² at start); any value from -40 to 20 m/s² applies at once and wakes every body |
| Slow motion | `T` down to 0.1, `P` | Time runs ten times slower, the step stays 1/120 s; pause freezes it |
| Masses | `E` up to 50 kg, then `Q` down to 0.1 kg, with the default shot | The 50 kg apple scatters the starting pyramid (blocks thrown up to 13 m); the 0.1 kg apple bounces back at about 3 m/s and moves no block |
| Fast projectile | 60 m/s at -30° against the ragdoll or a model, at -15° against the starting pyramid | No tunnelling: the apple never passes through what it hits; its radius is at least 0.25 m, so even at 60 m/s it moves no more than its diameter per step |
| Many objects | `4` several times, `9` | 150 blocks per wall, 196 balls per rain, the frame rate and the object count in the title; measured on a desktop processor, about 900 bodies keep the average step under the 8.3 ms of real time |
| Wireframes | `F1` | Every collider in yellow and every contact in red with its normal in cyan |
| Ragdoll | `5`, then `SPACE` at -10° and 11 m/s | The ragdoll folds at the joints within their limits, never comes apart, and sleeps as one island |
| Mesh colliders | `7` `8` to choose, `6` to place, `9` for the rain, `F1` | The balls rest on the exact surface: between the humanoid's legs, through the hole of the torus, in the openings of the logo |
| Camera | `W` `A` `S` `D` `R` `F` into any body | The camera stops at the surface or slides along it; a body dropped on it pushes it out |
| Starting scene | `assets/trebuchet.csv` with `position,0,6,33.7` | `assets/trebuchet.csv: the camera would start inside the trebuchet at (0, 8.75, 33.7)` and exit status 1 |
| Object file | `assets/block.csv` with `mass,-1,,` | `assets/block.csv:4: 'mass' must be positive` and exit status 1 |
| Reload | the same edits, then **Reload from files** in the menu | The same messages, then `Reload aborted: the object files above need fixing`; the running scene is kept |

## Bonus

- **Ragdoll** (`5`): eleven bodies, boxes and a sphere for the head, held by ten joints. Neck, waist, shoulders and hips are ball and socket joints whose limb swings inside a cone leaning where the body bends more and twists within a range; elbows and knees are hinges that fold one way only. Jointed parts do not collide with each other, except at the shoulders, where the chest stops the arm. It appears standing, folds up under gravity or an apple, and comes to rest as one sleeping island; `assets/ragdoll.csv` gives its height, mass and material.
- **Mesh colliders** (`6`, `7`, `8`): every `.obj` file in `assets/models/` is loaded at start-up and becomes a dynamic body whose collider is its own triangle surface, with the volume, centre of mass and full inertia tensor computed from it. Files exported for display are repaired on load: split vertices are welded, repeated faces dropped, faces turned to one winding and holes closed, each repair reported on standard output; a surface that cannot be closed is reported and skipped. Three models ship (a humanoid mannequin, the logo model scop displays, a torus), and scop's own models (the Utah teapot, a crown, a star prism) load as they are. `assets/models.csv` sets their size (0.2 to 6 m), density (10 to 1 000 kg/m³) and material: a model's mass is the density times its volume, and a new size, from the file with **Reload** or from the menu with **Apply**, rescales every model already placed.
- **Precision test** (`9`): 196 small balls dropped on whatever stands at the spot; with `F1` every contact is drawn on the surface with its normal. The balls fall between the humanoid's legs, through the hole of the torus and into the open parts of the logo, where a box or a convex hull would hold them in the air.
- **GUI** (`ESC`): every tunable value (gravity, time scale, launch, the material and mass of apples, blocks, ground, trebuchet and ragdoll, and the size, density and material of the models) in a panel drawn by the engine's own 2D overlay, with the mouse or the keyboard. It edits a draft: values reach the simulation on **Apply**, and **Save to files** / **Reload from files** write or read `assets/*.csv`. **Choose a model** opens a second page that lists every loaded model with its triangle count; a click places it and resumes.

The collider wireframes (`F1`) show what the collision code tests: every collider drawn as a yellow wireframe over the scene, slightly inflated, and every contact of the last step as a red point with its normal in cyan.

## Valgrind

The window system, the toolkit that draws the window decorations and the OpenGL driver keep global state until the process exits. Two suppression files silence only that code, matched by library and never by a project function, so a leak or an invalid access in `srcs/` is still reported:

| File | Valgrind | Contents |
|---|---|---|
| `docs/valgrind.supp` | every version | GLFW, Wayland, X11, GTK / libdecor, GLib, D-Bus, fontconfig, Mesa, NVIDIA, libstdc++ and the glibc loader |
| `docs/valgrind_recent.supp` | 3.22 or newer | Two NVIDIA start-up errors whose kinds (`ReallocZero`, `BadSize`) older versions do not know |

`make valgrind` always loads `docs/valgrind.supp` and adds `docs/valgrind_recent.supp` when the installed valgrind accepts it. `--keep-debuginfo=yes` is required: the Mesa driver is unloaded before the leak check, and without it its frames lose their library name. The program frees everything it owns in `game_shutdown`; a run closed through the menu or the window's close button ends with every lost and reachable counter at 0 and `ERROR SUMMARY: 0 errors`. `Ctrl+C` skips the cleanup.

## Shared with scop

scop, made after ft_newton, reuses parts of it. The two projects share:

- **The `.obj` reader** (`srcs/data/obj_load.c`, `obj_face.c`, `obj_triangulate.c`): two passes so a face may name a later vertex, corners as `v`, `v/vt`, `v//vn` or `v/vt/vn`, negative indices, ear clipping in the plane of the Newell normal, degenerate faces dropped, `file:line` errors. ft_newton then repairs the surface into a closed one, because a collider needs a volume; scop only draws it.
- **A model**: `assets/models/logo.obj` is the logo model scop displays, turned to face the camera.
- **Small helpers**: reading a whole file (`srcs/utils/file.c`) and growing an array by doubling (`srcs/utils/array.c`).
- **The rendering basics**: column-major 4x4 matrices uploaded without a transpose, a look-at view and a perspective projection, interleaved position and normal buffers behind a VAO, a flat normal per triangle corner, GLSL shaders loaded from `shaders/`, and a GLFW window whose title shows the frame rate (`H` in both).
- **The way they are built and checked**: `make install` and `make help`, `make O=…` with a flags stamp that rebuilds the objects when the flags change, `make valgrind` with the same two suppression files and `--keep-debuginfo=yes`, and strict loading that refuses a bad file with its line before any window opens.

They differ in what each one needs: ft_newton draws with OpenGL 3.3 Core loaded by GLAD and caps itself at 30 frames per second with the physics on a fixed 120 Hz step; scop uses OpenGL 4.1 with its own function loader and follows the monitor's refresh.

## Layout

```
ft_newton/
├── Makefile          # build (O=0..3, s, g), run, valgrind, install check, help
├── includes/         # newton.h (every type and prototype), defines.h (every constant, with units)
├── srcs/math/        # vectors, matrices, quaternions
├── srcs/physics/     # rigid bodies, inertia (meshes too), integration, world, joints, sleeping, removal
├── srcs/collision/   # colliders, broad-phase, narrow-phase, contact tests, mesh colliders and their repair, overlaps
├── srcs/response/    # impulses, friction, rolling resistance, joints, positional correction
├── srcs/render/      # window, shaders, meshes, camera, draw calls, wireframes and contacts, font, 2D overlay
├── srcs/menu/        # the pause menu: rows, layout, input, drawing
├── srcs/data/        # the strict CSV reader, the object definitions and the .obj reader
├── srcs/game/        # loop, keys, title readout, scene and its start check, camera collision, menu pages, trebuchet, structures, ragdoll, models, rain
├── srcs/utils/       # arguments, growable arrays, whole files
├── shaders/          # basic.vert, basic.frag (GLSL 3.30 core)
├── assets/           # object files (CSV) and models/ (the .obj models)
├── glad/             # OpenGL function loader
└── docs/             # info/ (topic documents) and the valgrind suppression files
```
