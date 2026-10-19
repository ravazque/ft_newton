# Rendering

How the simulation reaches the screen. The renderer only reads the world (positions, orientations, colliders, colors) and never changes it.

## The layers

| Layer | Role | Where |
|---|---|---|
| **GLFW 3** | creates the window and its OpenGL context, reports keyboard, mouse and window size | `srcs/render/window.c` |
| **GLAD** | loads the OpenGL 3.3 Core function pointers from the driver at start-up | `glad/`, called in `window_init` |
| **OpenGL 3.3 Core** | the GPU API: buffers, shaders, draw calls | `srcs/render/*.c` |
| **Shaders** | two small GLSL programs that run on the GPU | `shaders/`, see [shaders.md](shaders.md) |

Vertical sync is off and the main loop sleeps to hold 30 frames per second, so the frame rate does not depend on the monitor.

## Meshes: geometry on the GPU

`srcs/render/mesh.c`

Three unit meshes serve every primitive body, and each model has one of its own, all built once at start-up:

| Mesh | Shape | Scaled to |
|---|---|---|
| cube | 24 vertices (4 per face, so each face has its own flat normal), 36 indices | the box's full extents |
| sphere | a UV sphere of 32 rings and 32 slices, radius 0.5 | the sphere's diameter |
| square | 4 vertices in the XZ plane, facing +Y | the ground's extent |
| model | the triangles of its collider, three vertices each with the triangle's flat normal | already at its size, centred on its centre of mass |

A model is drawn from the very triangles it collides with, so what is seen is what is tested.

Each mesh lives in GPU memory as three OpenGL objects:

- a **VBO** (vertex buffer): 6 floats per vertex, the position and the normal;
- an **EBO** (element buffer): the indices joining vertices into triangles;
- a **VAO** (vertex array): the recipe that says attribute 0 is the position, attribute 1 the normal, 6 floats apart.

Drawing a mesh is binding its VAO and calling `glDrawElements`.

## From a body to the screen: the MVP chain

Every vertex goes through three matrices (`srcs/math/mat4.c`):

```
clip position = Projection · View · Model · vertex
```

- **Model** (`renderer_body_model` in `srcs/render/renderer.c`): scale the unit mesh to the collider's size (a model's own mesh is not scaled), rotate it by the body's quaternion, move it to the body's position. A plane is placed from its equation, at `normal * offset`, rotated so the square faces its normal, and scaled to its extent. The ground drawn is therefore exactly the ground that collides.
- **View** (`camera_view`): moves the world so the camera sits at the origin looking down -Z; built with look-at from the camera's position and its yaw / pitch direction.
- **Projection** (`camera_projection`): a 60° perspective from 0.1 m to 500 m that makes distant things smaller and maps the visible volume to the cube OpenGL draws.

All matrices are column-major, the layout OpenGL expects, so they are uploaded without a transpose.

## The camera

`srcs/render/camera.c`

A free-flying eye: a position plus a yaw and a pitch. The look direction is `(cos pitch · cos yaw, sin pitch, cos pitch · sin yaw)`. Moving forward follows that direction, strafing follows the horizontal right vector (so it stays level), and rising follows world up. The pitch stops at ±89° so the view never becomes degenerate.

The camera collides with the scene (`srcs/game/camera_collision.c`). It is a sphere of 0.3 m tested against every body with the same contact tests the bodies use, and:

- a move is cut into steps of at most 10 cm, a third of the radius, so its centre never crosses even a thin body at 60 m/s;
- after each step it is pushed back along the deepest contact with each body, in 3 passes, so it slides along walls and into corners;
- a step that leaves it deeper inside something than it started (a gap narrower than the camera) is undone, so it stops there;
- after every physics step it is pushed out again, so a body that moves, falls, grows or appears where it stands never contains it;
- a centre swallowed by a model, too deep to touch any triangle, leaves through the closest point of the surface [F30];
- a camera pinned between bodies moves to the nearest free spot, tried in 10 cm steps up, along Z and along X, never down.

The starting position (0, 8.75, 33.7) is checked when the object files are loaded: a starting scene that puts the camera under the ground or inside a body is refused.

## A frame

`draw_frame` in `srcs/game/draw.c`:

1. **Begin** (`renderer_begin_frame`): clear the color and depth buffers, upload the view and projection matrices once.
2. **Bodies**: for each body, pick its mesh, build its model matrix and draw it lit in its color. The depth test keeps the nearest surface in front.
3. **Aim bar**: two thin boxes along the launch direction, drawn unlit in wireframe. The thick one goes from the arm tip to where apples appear; the thin one grows with the launch speed.
4. **Wireframes and contacts** (`F1`, `srcs/render/debugdraw.c`): the same meshes with `glPolygonMode(GL_LINE)`, unlit yellow, 1% larger (the plane 1 cm higher) so the lines sit on top of the solids instead of fighting with them in the depth buffer. Then every contact of the last step, with the depth test off so none is hidden: a small red cube at the contact point and a thin cyan stick along its normal.
5. **Menu** (when open): the 2D overlay described below.
6. **Title**: the FPS and object counters are written in the window title a few times per second (`srcs/game/hud.c`), so the scene itself stays free of debug text.

Then the buffers are swapped.

## The 2D overlay and the menu

`srcs/render/font.c`, `srcs/render/ui.c`, `srcs/menu/`

The menu needs no interface or font library.

- **Font**: `font.c` stores an 8x8 bitmap for each printable ASCII character, one byte per row.
- **Batching**: `ui.c` turns every panel, button and lit glyph pixel into a colored rectangle (two triangles) in one CPU-side buffer. The whole overlay is uploaded once and drawn with a single `glDrawArrays`.
- **Same shaders**: the overlay reuses the 3D program. An orthographic projection maps window pixels to the screen (y downward, like a page), the depth test is off so the overlay is always in front, and `uLit = 2` tells the fragment shader to take each vertex's own color, which travels in the normal slot. That is how panels, buttons and text of different colors share one draw call.
- **Size**: `menu_layout.c` makes the panel cover 75% of the window's width and height. It measures the content once in font pixels and picks the largest scale that fits; the scale is continuous, so the text grows smoothly with the window. Rows may stretch up to 1.5 times the text height to fill the panel and start right under the title line, and the columns in use share the spare width: three on the main page (values; ragdoll, models and actions; controls), two on the models page (the list of models; how placing works, the files and Back). Each font pixel is snapped to whole screen pixels, so text stays sharp at any scale.
- **Input on the same rectangles**: the layout is recomputed every frame, and the mouse is tested against exactly the rectangles that are drawn (`menu_input.c`).
