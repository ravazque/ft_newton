# Physics

Every formula the engine implements, once, with a tag. The line of code that implements a formula carries the same tag in a comment:

```bash
grep -rn "\[F21\]" srcs/
```

## Notation

| Symbol | Meaning | Symbol | Meaning |
|---|---|---|---|
| `x` | position (m) | `v` | linear velocity (m/s) |
| `q` | orientation (unit quaternion) | `w` | angular velocity (rad/s) |
| `m` | mass (kg) | `I` | inertia tensor (kg·m²) |
| `F` | force (N) | `T` | torque (N·m) |
| `J` | impulse (N·s) | `g` | gravity (m/s²) |
| `n` | contact normal, unit, from body `a` toward body `b` | `r` | lever arm: contact point minus center of mass |
| `e` | elasticity (0 = no bounce, 1 = perfectly elastic) | `mu` | Coulomb friction coefficient |
| `c_rr` | rolling resistance coefficient | `dt` | fixed step, 1/120 s |

`.` is the dot product, `x` between vectors the cross product, `^T` the transpose.

## Moving and turning: the same laws twice

Every quantity of translation has a rotational twin, and the engine treats them the same way:

| Translation | Rotation |
|---|---|
| position `x` | orientation `q` |
| velocity `v` | angular velocity `w` |
| mass `m` (resistance to being pushed) | inertia tensor `I` (resistance to being turned, depends on the axis) |
| force `F` makes a body accelerate: `a = F / m` | torque `T = r x F` makes it spin up: `alpha = I^-1 T` |
| impulse `J` changes the velocity: `v += J / m` | angular impulse `r x J` changes the spin: `w += I^-1 (r x J)` |
| momentum `m v` | angular momentum `I w` |

A force through the center of mass only moves a body. The same force applied off-center also produces a torque, so the body moves and turns. That is why a hit on the edge of a block spins it more than a hit on its middle, and why a collision "naturally" creates rotation: contact impulses act at contact points, not at centers.

## Integration

`srcs/physics/integrator.c`, `srcs/math/quat.c`

### [F1] Newton's second law, linear and angular

```
a = F / m + g                  alpha = I_world^-1 . T
```

The engine stores `1/m` and `I^-1`: every use is a division, and a static body is simply `1/m = 0`, `I^-1 = 0`.

### [F2] Semi-implicit Euler over one fixed step

```
v += a * dt                    x += v * dt      (with the NEW v)
w += alpha * dt                q  = integrate(q, w, dt)
```

Updating the velocity first and moving with the new velocity keeps stacks and bounces stable. Explicit Euler, which moves with the old velocity, adds energy and blows them up.

### [F3] Quaternion integration of an angular velocity

```
dq/dt = 0.5 * (0, w) * q       q = normalize(q + dq * dt)
```

The pure quaternion `(0, w)` times `q` is the rate of change of the orientation. Normalizing every step removes the drift of the first-order step.

### [F4] Angular damping

```
w *= ANGULAR_DAMPING^dt        (0.98 per second)
```

Bleeds the rotational energy the solver cannot see (air), so a spinning body can come to rest.

## Mass and inertia

`srcs/physics/inertia.c`, `srcs/physics/rigidbody.c`

### [F5] Solid sphere of radius R

```
I = (2/5) m R²   on every axis
```

### [F6] Solid box of half extents (hx, hy, hz)

```
Ixx = (m/3)(hy² + hz²)    Iyy = (m/3)(hx² + hz²)    Izz = (m/3)(hx² + hy²)
```

This is the usual `(m/12)(h² + d²)` written with half extents.

### [F7] Inertia in world space, refreshed whenever the body turns

```
I_world^-1 = R . I_local^-1 . R^T          R = rotation matrix of q
```

The tensor is constant in the body's own frame, but torques act in world space; this conjugation moves it between the two.

### [F8] Torque of a force applied away from the center of mass

```
T = r x F
```

### [F9] Impulse applied at a point

```
v += J / m                      w += I_world^-1 . (r x J)
```

An impulse is an instant change of momentum. At a point away from the center it changes both velocities.

## Broad-phase

`srcs/collision/collider.c`, `srcs/collision/broadphase.c`

### [F10] World AABB of a collider

```
sphere:  x ± R on each axis
box:     x ± |R| . h     (absolute rotation matrix times the half extents)
plane:   its square, extruded PLANE_THICKNESS below the surface
```

### [F11] Interval overlap

```
a.min <= b.max  AND  b.min <= a.max
```

Sweep and prune sorts the boxes on X and applies this on Y and Z to the neighbours that overlap on X.

## Narrow-phase

`srcs/collision/contact_sphere.c`, `srcs/collision/contact_box.c`, `srcs/collision/obb.c`, `srcs/collision/sat.c`, `srcs/collision/manifold.c`

### [F12] Sphere against sphere

```
d = xb - xa,  contact when |d| <= Ra + Rb
n = d / |d|,  depth = Ra + Rb - |d|
```

### [F13] Against the bounded plane (n . x = offset, limited to a square)

```
distance = n_plane . p - offset
sphere: contact when distance <= R and the center is over the square,   depth = R - distance
box:    one contact per corner with distance < 0 that is over the square, depth = -distance
```

"Over the square" means inside the ground's extent and no deeper than `PLANE_THICKNESS` under it. Past the edge nothing holds a body up.

### [F14] Sphere against box: closest point

```
p = x_box + R_box . clamp(R_box^T (x_sphere - x_box), -h, +h)
contact when |x_sphere - p| <= R
```

Clamping in the box's own frame handles faces, edges and corners with the same formula. A center already inside the box is pushed out through the nearest face.

### [F15] Separating axis theorem for two boxes

For each candidate axis `L` (3 face normals of `a`, 3 of `b`, 9 edge cross products):

```
ra = Σ |h_a[i] (A_i . L)|     rb = Σ |h_b[i] (B_i . L)|
overlap = ra + rb - |(xb - xa) . L|
```

One axis with a negative overlap proves the boxes do not touch. Otherwise the axis of least overlap is the contact normal and the overlap is the depth. A face axis is kept unless another axis is clearly better (5% and 5 mm), which stops a resting box from flickering between candidate normals.

### [F16] Face contact: Sutherland-Hodgman clipping

The face of the other box most opposed to the normal (the incident face) is clipped against the four side planes of the reference face. The clipped points lying below the reference face are the contacts. When more than four remain, the deepest, the one farthest from it and the two spanning the largest area are kept.

## Contact response

`srcs/response/impulse.c`, `srcs/response/correction.c`

### [F17] Velocity of a point of a body

```
v_point = v + w x r
```

### [F18] Relative velocity at a contact

```
v_rel = v_point(b) - v_point(a)          v_rel_n = v_rel . n   (negative: approaching)
```

### [F19] Effective mass along a direction d

```
1/m_d = 1/ma + 1/mb + d . [ (Ia^-1 (ra x d)) x ra + (Ib^-1 (rb x d)) x rb ]
```

How much impulse it takes to change the relative velocity along `d` by 1 m/s, rotation included. A contact far from the center is "lighter" because part of the impulse goes into turning the body.

### [F20] Elasticity target (Newton's impact law)

```
bias = -e * v_rel_n      when v_rel_n < -ELASTICITY_THRESHOLD (1 m/s)
bias = 0                 otherwise
e = max(e_a, e_b)
```

Measured before any impulse of the step. Slow touches do not bounce, which is what ends the endless micro-bouncing of a body at rest.

### [F21] Normal impulse with accumulated clamping

```
lambda = m_n * (bias - v_rel_n)
jn_new = max(jn_old + lambda, 0)          (contacts only push)
apply (jn_new - jn_old) * n               (see [F9])
```

Clamping the accumulated impulse instead of each increment lets the Gauss-Seidel passes converge on a stack instead of fighting.

### [F22] Coulomb friction, two tangent directions

```
mu = sqrt(mu_a * mu_b)
|jt| <= mu * jn
```

The same accumulate-then-clamp scheme, with the bound following the normal impulse. Below the bound the contact sticks (static friction); at the bound it slides (kinetic friction).

### [F23] Positional correction

```
c = min(max(depth - SLOP, 0), MAX_CORRECTION) * PERCENT * m_n
x += ∓ c n / m            q turns by I^-1 (r x c n)
```

The overlap left after the impulses is removed by moving the bodies, through the same effective mass, a fraction per pass. It works on positions, not velocities, so it adds no energy.

### [F26] Rolling resistance (spheres)

```
limit = c_rr * R * jn
|jr| <= limit        (an opposing spin impulse about t1, t2 and n)
1/m_r = u . Ia^-1 u + u . Ib^-1 u        (u = t1, t2 or n)
```

A rigid ball rolling without slipping never slides, so friction alone never slows it down. A real ball and the ground deform slightly where they touch, which creates a torque against the rolling, `T = c_rr N R`. The engine applies that torque as a clamped angular impulse. The rolling ball then decelerates at `a = c_rr g / (1 + 2/5)`, about 2.1 m/s² for the apples (`c_rr = 0.3`).

A ball spinning in place about the contact normal is the same blind spot: its contact point only turns, it never slides, so friction never sees it either. The same patch resists that spin with the same bound, about the normal axis, so the spin drops at `alpha = c_rr g / (2/5 R)`, about 14.7 rad/s² for the apples. Boxes do not roll, so they never get these impulses: when a box tips over an edge its rotation is left untouched.

## Resting

`srcs/physics/sleep.c`

### [F24] Sleep test

```
|v| < SLEEP_LINEAR_EPS  AND  |w| < SLEEP_ANGULAR_EPS  for SLEEP_TIME (0.5 s)
=> the whole contact island sleeps
```

Bodies in contact form islands (static bodies do not link islands). A sleeping body skips integration and has zero velocity, and it wakes when an awake body touches it.

## Ballistics

`srcs/game/trebuchet.c`

### [F25] Launch velocity

```
v0 = speed * (cos(angle), sin(angle), 0)
```

The shot is not scripted: from its first step the apple is an ordinary body, and gravity through [F1] and [F2] draws the parabola.
