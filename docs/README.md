# ft_newton

## Description

ft_newton is a **3D rigid-body physics engine** written from scratch in C11, in the spirit of PhysX, Havok or Bullet, shown through an Angry-Birds-style game: a trebuchet throws apples at unstable structures of blocks, and every reaction (translation, rotation, bounces, friction, rolling, gravity) is computed by the engine itself.

Rendering runs on the GPU with **OpenGL 3.3 Core** (GLFW for the window and input, GLAD to load the OpenGL functions), through two hand-written shaders. All the mathematics (vectors, matrices, quaternions, inertia tensors) and all the physics (integration, collision detection, contact response) are implemented in the project; no calculation or collision library is used.

The gameplay happens on the XY plane, but the physics and the renderer are fully 3D and the camera flies freely through the scene.

## Documentation

| Document | What it explains |
|---|---|
| [architecture.md](info/architecture.md) | How the source tree is organised, what each file does, the main loop and one physics step |
| [physics.md](info/physics.md) | Every formula the engine implements, tagged `[F1]` to `[F26]`, and where it lives in the code |
| [collisions.md](info/collisions.md) | Broad-phase, narrow-phase, contact response, sleeping, the bounded ground, spawning without overlaps and the classic collision problems |
| [rendering.md](info/rendering.md) | The OpenGL pipeline: window, meshes, camera, draw calls, debug wireframes, the menu overlay |
| [shaders.md](info/shaders.md) | The vertex and fragment shaders, line by line |
| [object-files.md](info/object-files.md) | The CSV files that define apples, blocks, ground and trebuchet, and how they are validated |

## Features

- Three primitive colliders: **sphere, box and plane**. The plane is limited to the ground square that is drawn; past its edge bodies fall and are removed.
- Sweep-and-prune broad-phase and an exact narrow-phase for every shape pair (box against box through the separating axis theorem with face clipping).
- Fixed 120 Hz semi-implicit Euler integration, quaternion orientations and a world-space inertia tensor.
- Sequential-impulse contact solver with accumulated clamping, warm starting, elasticity, Coulomb friction, rolling resistance for spheres and a rotation-aware positional correction.
- Island-based sleeping: a pile that has come to rest switches off as a whole, so every impact ends in a stable state.
- Object types (apple, block, ground, trebuchet) described by small CSV files in `assets/`, loaded strictly: a wrong or out-of-range value, or a starting scene where bodies would overlap, is reported instead of being silently fixed.
- Direct keyboard control over time scale, gravity, projectile speed, direction and mass; walls, pyramids, towers and a 150-block wall spawned on demand, stacked on whatever already stands where they appear.
- A pause menu drawn by the engine (own bitmap font and 2D layer) that covers 75% of the window at any size, with mouse and keyboard control of every tunable value.
- FPS and object counters in the window title, a free-flying camera and a collider wireframe view that can be toggled at any time.

## Requirements

- A C11 compiler (`cc`), GNU Make, **GLFW 3** and **OpenGL** (Mesa or a vendor driver).
- GLAD is included in `glad/` (generated for OpenGL 3.3 Core, no extensions).

Arch-based systems: `sudo pacman -S --needed glfw mesa`. Debian/Ubuntu: `sudo apt install libglfw3-dev libgl1-mesa-dev`.

## Build and run

```bash
make          # builds ./newton
make run      # builds and runs (optionally: make run ARGS="1280 720")
make valgrind # builds and runs under valgrind with the suppression files (accepts ARGS too)
make clean    # removes the object files
make fclean   # removes the object files and the binary
make re       # rebuilds from scratch
```

`./newton [width height]` opens a window of that size (720x480 to 2560x1440). Run it from the repository root so `shaders/` and `assets/` are found.

The main loop is capped at 30 frames per second (`FPS_CAP`); the physics does not depend on it and always advances in fixed steps of 1/120 s.

## Valgrind

The window, the input and the OpenGL driver come from system libraries that allocate global state, keep it for the whole process and never free it before exit. Run bare, valgrind buries any real problem under thousands of their blocks. Two suppression files in `docs/` silence **only code outside the project**; a leak or an invalid access whose stack goes through `srcs/` is still reported.

| File | Valgrind | Contents |
|---|---|---|
| [valgrind.supp](valgrind.supp) | every version | Every library listed below, using only the error kinds all versions know (`Leak`, `Addr8`, `Addr16`, `Addr32`) |
| [valgrind_recent.supp](valgrind_recent.supp) | 3.22 or newer | The two NVIDIA start-up errors whose kinds (`ReallocZero`, `BadSize`) only exist since 3.21 / 3.22 |

They are separate because valgrind refuses a whole suppression file that names an unknown error kind (`unknown tool suppression type`): a single file with the new kinds would not load on an older valgrind.

### Running

```bash
make valgrind                  # builds, picks the right files and runs
make valgrind ARGS="1280 720"  # same, with a window size

# by hand, any valgrind version
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes \
         --keep-debuginfo=yes --suppressions=docs/valgrind.supp ./newton

# by hand, valgrind 3.22 or newer
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes \
         --keep-debuginfo=yes --suppressions=docs/valgrind.supp \
         --suppressions=docs/valgrind_recent.supp ./newton
```

`make valgrind` always loads `valgrind.supp` and adds `valgrind_recent.supp` only when the installed valgrind accepts it, so the same command works on every version.

- `--keep-debuginfo=yes` is required. The Mesa driver is loaded with `dlopen` and unloaded before the leak check; without the flag its frames lose their library name and show up as `???`, and no suppression can match them.
- Close the program normally (`ESC` and **Quit**, or the window's close button). `Ctrl+C` skips the cleanup and every block of the engine shows up as lost.
- A clean run ends with `definitely lost: 0`, `indirectly lost: 0`, `possibly lost: 0`, `still reachable: 0` and `ERROR SUMMARY: 0 errors`; everything else is counted as `suppressed`.
- `-s` lists, at the end of the report, every suppression used and how many bytes or errors it covered.

### What is suppressed, and why it is not project code

Every entry matches on the library the allocation or the access happens in (`obj:*/lib...so*`) or on a glibc loader function, never on a project function. The program frees everything it allocates in `game_shutdown`, and calls `glfwDestroyWindow` and `glfwTerminate`; what remains belongs to the libraries below.

| Source | Reported as | Why |
|---|---|---|
| GLFW | definitely lost (2 × 192 B) | On Wayland, `glfwCreateWindow` creates two `wl_proxy` objects (viewporter, fractional scale) that `glfwDestroyWindow` never destroys: a known GLFW bug |
| Wayland (client, cursor, egl), X11, xcb | still reachable | The display connection, cursor themes and protocol caches live until the process exits |
| GTK / GDK / libdecor, cairo, pango, harfbuzz, fribidi | still reachable, possibly lost | Window decorations on Wayland are drawn by libdecor through GTK, which initialises a full toolkit and never tears it down |
| GLib / GObject / GIO, gmodule, libffi, dconf | still reachable, invalid read | Type system, settings and D-Bus state created by `gtk_init`; GIO also performs an invalid read while reading its settings at start-up |
| fontconfig | definitely lost | The global font cache GTK loads; fontconfig only releases it in `FcFini`, which nothing calls |
| glycin, gdk-pixbuf | possibly lost, still reachable | GTK's image loader runs detached threads and async executors that are still alive at exit |
| NVIDIA driver (`libnvidia-*`) | definitely lost, still reachable | Driver state kept until exit |
| NVIDIA EGL driver (`libnvidia-eglcore`) | `ReallocZero`, `BadSize` (valgrind 3.22+) | Zero-size `realloc` / `posix_memalign` calls while the driver initialises |
| Mesa: gallium, DRI drivers, LLVM (llvmpipe), EGL, GLX, GLdispatch, glapi, libdrm | definitely lost, still reachable | The shader compiler, the software rasteriser and its LLVM JIT keep their state for the life of the process |
| libstdc++ | still reachable (72,704 B) | The emergency exception pool the C++ runtime allocates when the driver loads it, by design |
| glibc dynamic loader | still reachable, invalid read of size 8 | Buffers `dlopen` keeps for every library the GL stack loads at run time; its word-at-a-time `strncmp` reads past the end of an rpath string while expanding it |

## Controls

The keys act on the running simulation immediately.

| Key | Action |
|---|---|
| `ESC` | open / close the menu (opening it pauses the simulation) |
| `SPACE` | fire an apple (refused, with a message, while something occupies the launch point) |
| `1` / `2` / `3` | spawn a wall (6x6) / a pyramid (7 rows) / a tower (8 blocks) |
| `4` | spawn a large wall (15x10 = 150 blocks) |
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
| `+` / `-` | camera fly speed up / down |

The yellow bar at the tip of the throwing arm shows the launch direction; the thin line past it grows with the launch speed. The window title reads `FPS | objects | g | time | speed | angle | mass`.

Structures always appear at the same spot (walls, pyramids and towers at x = 8, the large wall at x = 20). When that spot is taken, the new structure is lifted just enough to rest on top of what is there, so no block ever starts inside another body.

## Menu

`ESC` pauses and opens a panel with every tunable value, the list of controls and the actions. The panel covers 75% of the window in both directions and its text scales continuously with the window.

The menu edits a **draft**, not the simulation: moving a value changes nothing on screen, the title line switches to `APPLY to use the edits`, and the values reach the world only when **Apply** is pressed.

- **Values**: gravity, time scale, launch speed and angle, the mass, friction and elasticity of apples and blocks, the rolling resistance of apples, and the friction and elasticity of the ground and the trebuchet. Click `[-]` / `[+]` or hold them to repeat.
- **Actions**:
  - **Apply**: the draft becomes the running simulation, and the state **Reset** returns to.
  - **Reload from files**: re-reads `assets/*.csv` into the simulation and the draft.
  - **Save to files**: writes the draft to `assets/*.csv` without applying it.
  - **Reset simulation**: default gravity and time scale, starting scene rebuilt.
  - **Resume** / **Quit**.

The mouse highlights and clicks rows; the up / down arrows move the selection, left / right change the selected value and `ENTER` presses the selected button.

## Project layout

```
Makefile          one `make` builds everything
includes/         newton.h (every type and prototype), defines.h (every constant, with units)
srcs/main.c       entry point
srcs/math/        vectors, matrices, quaternions, scalar helpers
srcs/physics/     rigid bodies, inertia, integration, the world, sleeping, removal of lost bodies
srcs/collision/   colliders, broad-phase, narrow-phase and every contact test, placement queries
srcs/response/    contact solver: impulses, friction, rolling resistance, positional correction
srcs/render/      window, shaders, meshes, camera, draw calls, debug wireframes, font, 2D overlay
srcs/menu/        the pause menu: rows, layout, input, drawing
srcs/data/        the strict CSV reader and the object definitions
srcs/game/        the loop, the frame, the keys, the title readout, the scene, the trebuchet, the structures
srcs/utils/       command-line arguments
shaders/          basic.vert / basic.frag (GLSL 3.30 core)
assets/           the object definition files (CSV)
glad/             OpenGL function loader
docs/             this README, info/ (the detailed documents), the valgrind suppression files
```
