# Shaders

The engine draws everything, 3D scene and 2D menu, with one GPU program made of two GLSL 3.30 shaders in `shaders/`.

## What a shader is

The GPU does not draw objects; it draws triangles, and runs small programs at two points of the way:

```
vertices (VBO) --> [vertex shader] --> triangles assembled and rasterized --> [fragment shader] --> pixels
                   once per vertex                                            once per pixel covered
```

- The **vertex shader** decides where each vertex lands on the screen.
- The **fragment shader** decides the color of each pixel a triangle covers.

Data reaches them in two ways:

- **attributes**, different for every vertex (position, normal), read from the mesh's buffers;
- **uniforms**, the same for a whole draw call (matrices, color, mode), set from C with `glUniform*`.

## Loading

`srcs/render/shader.c`, at start-up:

1. read `shaders/basic.vert` and `shaders/basic.frag` into memory;
2. `glCreateShader` + `glShaderSource` + `glCompileShader` for each, and print the driver's log on a compile error;
3. `glCreateProgram` + `glAttachShader` + `glLinkProgram`, and check the link;
4. delete the two shader objects (the program keeps what it needs).

If anything fails, the program stops with the driver's message.

## The vertex shader: `basic.vert`

```glsl
layout (location = 0) in vec3 aPos;      // attribute 0: position, in the mesh's own space
layout (location = 1) in vec3 aNormal;   // attribute 1: normal

uniform mat4 uModel;        // mesh space -> world    (the body's size, rotation, position)
uniform mat4 uView;         // world -> camera
uniform mat4 uProjection;   // camera -> clip space   (perspective)

out vec3 vNormalWorld;      // handed to the fragment shader

gl_Position  = uProjection * uView * uModel * vec4(aPos, 1.0);
vNormalWorld = mat3(uModel) * aNormal;
```

- `gl_Position` is the vertex in clip space: the whole Model-View-Projection chain in one line. The GPU then divides by `w` (perspective) and maps the result to window pixels.
- The normal is rotated into world space by the upper 3x3 part of the model matrix, so the lighting follows the body when it turns. The scale is folded in too, which the fragment shader removes by normalizing.
- The `location` numbers match the attribute slots the mesh's VAO declares (`srcs/render/mesh.c`).

## The fragment shader: `basic.frag`

```glsl
in vec3 vNormalWorld;       // interpolated across the triangle
uniform vec3 uColor;        // the body's color
uniform int  uLit;          // how to color the pixel
out vec4 FragColor;
```

`uLit` picks one of three modes:

| `uLit` | Mode | Used for |
|---|---|---|
| `1` | lit: `uColor * (0.25 + 0.85 * max(dot(n, -lightDir), 0))` | every body of the scene |
| `0` | flat: `uColor` as is | collider wireframes, the aim bar |
| `2` | per-vertex color: the value in `vNormalWorld` is the color | the menu overlay |

### The lighting

One fixed directional light, like the sun, coming from `(-0.4, -1, -0.3)`:

- **ambient** 0.25: every surface gets some light, so faces turned away are dark but not black;
- **diffuse** (Lambert): `max(dot(normal, -lightDir), 0)` is 1 for a face turned straight at the light and 0 for one at 90° or more, scaled by 0.85.

Because the cube has a separate normal per face, each face of a block gets a flat, distinct shade, which is what makes blocks read as solid.

### Mode 2: colors through the normal slot

The menu is a single buffer of colored rectangles. Instead of a second shader program, the overlay stores each vertex's color where the 3D meshes store the normal. The vertex shader passes it through unchanged (the overlay's model matrix is the identity), and mode 2 outputs it as the pixel color. One program, one draw call, any number of colors.

## Using the program each frame

`srcs/render/renderer.c` and `srcs/render/ui.c`:

1. `glUseProgram` once per frame, then `uView` and `uProjection` from the camera;
2. per body: `uModel`, `uColor`, `uLit = 1`, then draw its mesh;
3. per wireframe or aim bar: the same with `uLit = 0`, with the rasterizer in line mode;
4. for the menu: `uProjection` = orthographic in pixels, `uView` = `uModel` = identity, `uLit = 2`, depth test off, one `glDrawArrays`.
