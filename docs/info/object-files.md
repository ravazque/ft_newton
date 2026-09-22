# Object files

Every kind of object reads its properties from a CSV file in `assets/`. The four files are required; the program loads them at start-up and refuses to run with a broken one.

## Format

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

### What the properties mean

| Property | Meaning |
|---|---|
| `mass` | kg. Inertia is derived from it and the shape. The ground and the trebuchet are static and have none. |
| `friction` | Coulomb coefficient. Two surfaces use `sqrt(mu_a * mu_b)`. 0 is ice; values above 1 are allowed (very grippy). |
| `elasticity` | Bounciness: 0 means an impact dies on contact, 1 a perfectly elastic rebound. The higher of the two bodies is used. |
| `rolling_resistance` | Spheres only, because only they roll: the torque resisting rolling and spinning in place, as a share of normal force times radius. 0 rolls on forever; 0.3 stops a 5 m/s apple in about 2.5 s. |
| `color` | RGB, each 0 to 1. |
| `normal`, `offset` | The ground plane `normal · x = offset`. The normal is normalized on load. |
| `extent` | Side of the ground square. It is both what is drawn and what holds bodies up: past it, bodies fall. |
| `position`, `base_size` | Where the trebuchet's sill stands, and its size. |
| `frame_height` | Height of the pivot the arm turns on. |
| `arm_length`, `arm_thickness`, `arm_angle` | The throwing arm; the angle is how high its throwing side points, in degrees. |
| `counterweight_size` | The weight hanging off the arm's short side. |
| `launch_speed`, `launch_angle` | The starting values of the live launch controls. |

What the trebuchet file does not spell out (leg spread and thickness, where the pivot cuts the arm) follows fixed proportions in `includes/defines.h`.

## Validation

Loading is strict, so a mistake is reported instead of silently replaced by a default:

- the header must be exact;
- every property of the shape must be present, and nothing else may appear: a misspelt name (`masss`), a property of another shape (`radius` in the box file) or a `rolling_resistance` on anything but a sphere is an error;
- an unknown shape, a duplicated property, the wrong number of values, or a malformed or overflowing number is an error;
- values must be in range:

| Property | Range |
|---|---|
| `radius`, `mass`, `extent`, every component of `size` | > 0 |
| apple `mass` | >= 0.1 kg (the lightest projectile) |
| `friction` | >= 0 |
| `elasticity`, `rolling_resistance`, each `color` component | 0 to 1 |
| ground `normal` | not the zero vector |
| trebuchet sizes, `frame_height`, `arm_length`, `arm_thickness` | > 0, and the pivot above the sill |
| `arm_angle` | 0 to 90 |
| `launch_speed` | 1 to 60 m/s |
| `launch_angle` | -70 to 90 degrees |

Every problem is printed as `file:line: message`, all of them at once for a file.

### The starting scene

Once the four files are valid, the starting scene (ground, trebuchet and a 5-row pyramid at x = 8) is built aside and checked. The files are refused, naming the files involved, when:

- the pyramid would overlap the trebuchet (a trebuchet moved onto its spot, or blocks so large the pyramid reaches it);
- the pyramid would sink into the ground (a ground raised through it);
- the pyramid would not stand over the ground square (an extent too small).

## Changing the files while the game runs

The menu's actions work with the files:

- **Reload from files** reads them again. Mass, friction, elasticity, rolling resistance and color reach every existing body of that kind at once (inertia is recomputed), the launch settings return to the file values, and everything is woken so piles react. Geometry (shape, size, radius, the plane, the trebuchet layout) applies to bodies spawned from then on, or to the whole scene after **Reset simulation**. A broken or overlapping set of files is refused and the running values are kept.
- **Save to files** writes what the menu holds into the four files, overwriting them. It does not change the running simulation; **Apply** does that.
