# Object files

Every kind of object reads its properties from a CSV file in `assets/`, and every Wavefront `.obj` file in `assets/models/` becomes a model. The CSV files are required: the program loads them at start-up and refuses to run with a broken one. A model file that cannot be used is reported and skipped, and the program runs with the others.

## Format

```
# property,value1,value2,value3
shape,sphere,,
radius,0.5,,
mass,2,,
rolling_resistance,0.15,,
friction,0.4,,
elasticity,0.45,,
color,0.85,0.25,0.25
```

- The first line must be exactly `# property,value1,value2,value3`, leading `# ` included.
- Each other line is a property name followed by **one number**, **three numbers** (a vector or an RGB color) or **one word**.
- Trailing empty fields may be left out (`mass,2` equals `mass,2,,`); blank lines and CRLF line endings are accepted.
- Numbers are plain decimals (`2`, `-0.5`, `.25`, `1e-3`). `nan`, `inf` and hexadecimal are refused.

## The files

| File | Properties |
|---|---|
| `apples.csv` | `shape,sphere` · `radius` · `mass` · `rolling_resistance` · `friction` · `elasticity` · `color,r,g,b` |
| `block.csv` | `shape,box` · `size,x,y,z` (full extents) · `mass` · `friction` · `elasticity` · `color,r,g,b` |
| `ground.csv` | `shape,plane` · `normal,x,y,z` · `offset` · `extent` · `friction` · `elasticity` · `color,r,g,b` |
| `trebuchet.csv` | `position,x,y,z` · `base_size,x,y,z` · `frame_height` · `arm_length` · `arm_thickness` · `arm_angle` · `counterweight_size,x,y,z` · `launch_speed` · `launch_angle` · `friction` · `elasticity` · `color,r,g,b` |
| `ragdoll.csv` | `height` · `mass` · `friction` · `elasticity` · `color,r,g,b` |
| `models.csv` | `shape,mesh` · `size` · `density` · `friction` · `elasticity` · `color,r,g,b`, shared by every model |

### What the properties mean

| Property | Meaning |
|---|---|
| `mass` | kg. Inertia is derived from it and the shape. The ground and the trebuchet are static and have none; a model's mass comes from its density. |
| `friction` | Coulomb coefficient. Two surfaces use `sqrt(mu_a * mu_b)`. 0 is ice; values above 1 are allowed (very grippy). |
| `elasticity` | Bounciness: 0 means an impact dies on contact, 1 a perfectly elastic rebound. The higher of the two bodies is used. |
| `rolling_resistance` | Spheres only, because only they roll: the torque resisting rolling and spinning in place, as a share of normal force times radius. 0 rolls on forever; the shipped 0.15 stops a 5 m/s apple in about 5 s. |
| `color` | RGB, each 0 to 1. |
| `normal`, `offset` | The ground plane `normal · x = offset`. The normal is normalized on load. |
| `extent` | Side of the ground square. It is both what is drawn and what holds bodies up: past it, bodies fall. |
| `position`, `base_size` | Where the trebuchet's sill stands, and its size. |
| `frame_height` | Height of the pivot the arm turns on. |
| `arm_length`, `arm_thickness`, `arm_angle` | The throwing arm; the angle is how high its throwing side points, in degrees. |
| `counterweight_size` | The weight hanging off the arm's short side. |
| `launch_speed`, `launch_angle` | The starting values of the live launch controls. |
| `height` | Ragdoll: the figure's height; its eleven parts follow fixed proportions of it. |
| `mass` (ragdoll) | The whole figure: each part takes a fixed share (the torso about a third, a forearm 2.5%). |
| `size` | Models: the longest side of each model's box, in metres; every model is scaled uniformly to it, also the ones already placed when it changes. |
| `density` | Models: kg/m³. A model's mass is the density times the volume its surface encloses, so a bigger model is heavier; the shipped 100 gives the humanoid 6.6 kg, the logo 42.2 kg and the torus 84.4 kg. |

What the trebuchet and the ragdoll files do not spell out (leg spread and thickness, where the pivot cuts the arm; the layout of the parts and the limits of the joints) follows fixed proportions in `includes/defines.h` and `srcs/game/ragdoll.c`.

### The models

`assets/models/` ships three models: `humanoid.obj` (a mannequin, one closed shell per segment), `logo.obj` (the logo model scop displays, turned to face the camera) and `torus.obj` (a ring). Every file of the directory ending in `.obj` is loaded at start-up, in name order, up to 16; its name without the extension (up to 31 characters) is the name the menu and the window title show. The reader takes any Wavefront `.obj`:

- `v x y z` vertices and `f` faces; `vt`, `vn`, `o`, `g`, `s`, `usemtl`, `mtllib` and unknown keywords are skipped, `#` starts a comment and a line ending in `\` continues on the next;
- a face corner is `v`, `v/vt`, `v//vn` or `v/vt/vn`, and a negative index counts back from the face's own line; a face may name a vertex defined after it;
- a face of any number of corners, concave or slightly non-planar, is cut into triangles by ear clipping in the plane of its Newell normal; faces with no area are dropped, and vertices no face uses are ignored.

A collider must enclose a volume: every edge must be shared by exactly two faces that run it in opposite directions. Files exported for display often are not like that, so the surface is repaired after it is read, and each repair is reported on standard output:

1. vertices closer than a millionth of the model size are welded into one, which joins faces exported with split corners;
2. faces with a repeated corner, or repeating another face, are dropped;
3. each connected piece is oriented from one face outward, turning the faces wound the other way;
4. every hole is closed by a fan of triangles around the centroid of its rim;
5. a piece that encloses a negative volume (wound inward) is turned inside out.

```
assets/models/teapot.obj: repaired (403 vertices welded, 0 faces dropped, 0 faces turned, 6 holes closed, 0 pieces turned outward)
```

A surface that cannot be made closed is refused: an edge shared by three faces or more, a surface whose two sides meet (a Möbius strip), a hole whose rim touches itself at a vertex, or a surface that encloses no volume. All of scop's models (the Utah teapot with its open spout and lid, the crown, the star prism) load this way.

## Validation

Loading is strict, so a mistake is reported instead of silently replaced by a default:

- the header must be exact;
- every property of the shape must be present, and nothing else may appear: a misspelt name (`masss`), a property of another shape (`radius` in the box file) or a `rolling_resistance` on anything but a sphere is an error;
- an unknown shape, a duplicated property, the wrong number of values, or a malformed or overflowing number is an error;
- values must be in range:

| Property | Range |
|---|---|
| `mass`, `extent`, every component of `size` | > 0 |
| apple `radius` | >= 0.25 m: at the top launch speed (60 m/s) an apple moves 0.5 m per step, its diameter, so it never crosses a surface between two steps |
| apple `mass` | >= 0.1 kg (the lightest projectile) |
| `friction` | >= 0 |
| `elasticity`, `rolling_resistance`, each `color` component | 0 to 1 |
| ground `normal` | not the zero vector |
| trebuchet sizes, `frame_height`, `arm_length`, `arm_thickness` | > 0, and the pivot above the sill |
| `arm_angle` | 0 to 90 |
| `launch_speed` | 1 to 60 m/s |
| `launch_angle` | -70 to 90 degrees |
| ragdoll `height` | 0.5 to 3 m |
| ragdoll `mass` | 1 to 200 kg |
| model `size` | 0.2 to 6 m (at 6 m and the highest density the torus weighs 22.8 t; the solver stays stable up to there, and a model placed at x = 2 m stays clear of the starting pyramid) |
| model `density` | 10 to 1000 kg/m³ |

Every problem is printed as `file:line: message`, all of them at once for a file. The models report theirs the same way, followed by the model being skipped:

```
assets/models/logo.obj:93: vertex index out of range '999'
assets/models/logo.obj: model skipped
assets/models/fin.obj: 1 edges are shared by more than two faces
assets/models/strip.obj: the surface cannot be oriented (its two sides meet)
assets/models/bowtie.obj: a hole touches itself at a vertex and cannot be closed
assets/models/flat.obj: the surface encloses no volume
assets/models/: only the first 16 models are loaded, 4 files ignored
```

### The starting scene

Once the files are valid, the starting scene (ground, trebuchet and a 5-row pyramid at x = 8) is built aside and checked. The files are refused, naming the files involved, when:

- the pyramid would overlap the trebuchet (a trebuchet moved onto its spot, or blocks so large the pyramid reaches it);
- the pyramid would sink into the ground (a ground raised through it);
- the pyramid would not stand over the ground square (an extent too small).

## Changing the files while the game runs

The menu's actions work with the files:

- **Reload from files** reads them again. Mass, friction, elasticity, rolling resistance and color reach every existing body of that kind at once (inertia is recomputed; each ragdoll part takes its share of the ragdoll's mass), the launch settings return to the file values, and everything is woken so piles react. Geometry (shape, size, radius, the plane, the trebuchet layout, the ragdoll's height) applies to bodies spawned from then on, or to the whole scene after **Reset simulation**. The model surfaces are loaded once, at start-up, and shared by their bodies. A new `size` (from the file with Reload, or from the menu with Apply) rescales every model at once: its surface, its drawing and every body placed from it, which keeps its lowest point where it was and is lifted onto anything it would otherwise sink into; volume, mass and inertia follow. A new `density` changes the mass of every model at once. A broken or overlapping set of files is refused and the running values are kept.
- **Save to files** writes what the menu holds into the six files, overwriting them, in the same layout. It does not change the running simulation; **Apply** does that.
