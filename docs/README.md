# ft_newton

ft_newton is a 3D rigid-body physics engine written from scratch in C11, in
the spirit of PhysX, Havok or Bullet, shown through a small game: a trebuchet
throws apples at unstable structures of blocks, and every reaction
(translation, rotation, bounces, friction, rolling, gravity) is computed by the
engine itself.

The scene is drawn with OpenGL 3.3 Core through two hand-written shaders; GLFW
provides the window and the input, and GLAD loads the OpenGL functions. All the
mathematics (vectors, matrices, quaternions, inertia tensors) and all the
physics (integration, collision detection, contact response) live in the
project, with no physics or collision library. The game happens on the XY
plane, but the engine and the camera are fully three-dimensional.

## How it works

- Three colliders: sphere, box and plane. The plane is limited to the drawn
  ground square; bodies that fall past its edge are removed.
- Fixed 120 Hz semi-implicit Euler integration, quaternion orientations and a
  world-space inertia tensor; the time scale changes how many steps a frame
  runs, never their size.
- Sweep-and-prune broad-phase, then an exact test for every shape pair; box
  against box uses the separating axis theorem on 15 axes and face clipping.
- Sequential-impulse contact solver with accumulated clamping, warm starting,
  elasticity, Coulomb friction and rolling resistance, followed by a positional
  correction that removes the leftover overlap without adding energy.
- Island sleeping: a pile that has come to rest switches off as a whole, so
  every impact ends in a stable state.
- Apples, blocks, ground and trebuchet are described by CSV files in
  `assets/`, loaded strictly: a wrong value or a starting scene where bodies
  would overlap is reported with its file and line instead of being fixed
  silently.

Every formula carries a tag such as `[F15]` in the source, and the documents
of `info/` explain each one:

| Document | Contents |
|---|---|
| [architecture.md](info/architecture.md) | Source tree, main loop, one physics step |
| [physics.md](info/physics.md) | Every formula, tagged `[F1]` to `[F26]`, and where it lives |
| [collisions.md](info/collisions.md) | Broad-phase, narrow-phase, response, sleeping, spawning without overlaps |
| [rendering.md](info/rendering.md) | Window, meshes, camera, draw calls, debug wireframes, menu overlay |
| [shaders.md](info/shaders.md) | The two shaders, line by line |
| [object-files.md](info/object-files.md) | The CSV format and its checks |

## Dependencies

| Dependency | Version | Debian / Ubuntu | Arch |
|---|---|---|---|
| C compiler and GNU Make | C11 | `build-essential` | `base-devel` |
| `pkg-config` | any | `pkg-config` | `pkgconf` |
| GLFW 3 | 3.3 or newer | `libglfw3-dev` | `glfw` |
| OpenGL | 3.3 core, Mesa or a vendor driver | `libgl1-mesa-dev` | `mesa` |
| `valgrind` | optional, for `make valgrind` | `valgrind` | `valgrind` |

GLAD is included in `glad/`, so there is no loader to install. `make install`
checks the compiler, `pkg-config`, GLFW and OpenGL, and names whatever is
missing.

## Build and run

```
make install    # checks the dependencies
make            # builds ./newton
make run        # builds and runs; ARGS="1280 720" sets the window size
make valgrind   # builds and runs under valgrind with the suppression files
make clean      # objects
make fclean     # objects and the binary
make help       # the list above
```

`./newton [width height]` opens a window from 720x480 to 2560x1440 (1280x720
by default) and runs from the repository root, where `shaders/` and `assets/`
are found. The main loop is capped at 30 frames per second; the physics does
not depend on it.

| Key | Action |
|---|---|
| `ESC` | Menu (pauses the simulation) |
| `SPACE` | Fire an apple |
| `1` `2` `3` | Wall (6x6), pyramid (7 rows), tower (8 blocks) |
| `4` | Large wall, 150 blocks |
| `P` | Pause / resume |
| `F1` | Collider wireframes |
| `H` | Window title readout |
| `I` `K`, `J` `L`, `Q` `E` | Launch angle, launch speed, mass of the next apples |
| `G` `B`, `T` `Y` | Gravity, time scale |
| `W` `A` `S` `D`, `R` `F`, arrows, `+` `-` | Camera: fly, up / down, turn, speed |

New structures appear at a fixed spot and are lifted onto whatever already
stands there, so no body ever starts inside another one.

## Valgrind

The window system, the toolkit that draws the window decorations and the
OpenGL driver keep global state until the process exits. The two suppression
files in `docs/` silence only that code, matched by library and never by a
project function, so a leak or an invalid access in `srcs/` is still reported.

| File | Valgrind | Contents |
|---|---|---|
| [valgrind.supp](valgrind.supp) | every version | GLFW, Wayland, X11, GTK / libdecor, GLib, fontconfig, Mesa, NVIDIA, libstdc++ and the glibc loader |
| [valgrind_recent.supp](valgrind_recent.supp) | 3.22 or newer | Two NVIDIA start-up errors whose kinds (`ReallocZero`, `BadSize`) older versions do not know |

`make valgrind` always loads `docs/valgrind.supp` and adds
`docs/valgrind_recent.supp` when the installed valgrind accepts it.
`--keep-debuginfo=yes` is required: the Mesa driver is unloaded before the leak
check, and without it its frames lose their library name. The program frees
everything it owns in `game_shutdown`; a run closed through the menu or the
window's close button ends with every lost and reachable counter at 0 and
`ERROR SUMMARY: 0 errors`. `Ctrl+C` skips the cleanup.

## Bonus

The menu (`ESC`) is the bonus, a GUI for the engine: every tunable value
(gravity, time scale, launch, and the materials of apples, blocks, ground and
trebuchet) in a panel drawn by the engine's own 2D overlay. It edits a draft:
values reach the simulation on **Apply**, and **Save to files** / **Reload from
files** write or read `assets/*.csv`.

The collider wireframes (`F1`) are the debug display the project requires:
every collider drawn as a yellow wireframe over the scene, slightly inflated,
showing what the collision code tests; it turns on and off at any time.

## Layout

```
ft_newton/
├── Makefile          # build, run, valgrind, install check
├── includes/         # newton.h (every type and prototype), defines.h (every constant, with units)
├── srcs/math/        # vectors, matrices, quaternions
├── srcs/physics/     # rigid bodies, inertia, integration, world, sleeping, removal
├── srcs/collision/   # colliders, broad-phase, narrow-phase, contact tests, placement queries
├── srcs/response/    # impulses, friction, rolling resistance, positional correction
├── srcs/render/      # window, shaders, meshes, camera, draw calls, wireframes, font, 2D overlay
├── srcs/menu/        # the pause menu
├── srcs/data/        # the strict CSV reader and the object definitions
├── srcs/game/        # loop, keys, title readout, scene, trebuchet, structures
├── shaders/          # basic.vert, basic.frag (GLSL 3.30 core)
├── assets/           # object definition files (CSV)
├── glad/             # OpenGL function loader
└── docs/             # info/ and the valgrind suppression files
```
