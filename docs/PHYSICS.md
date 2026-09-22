# The physics of ft_newton, in one place

Every formula the engine implements is written out here once, with a tag. The
line of code that implements it carries the same tag in a comment, so a formula
can be found from this document and read back from the source:

```bash
grep -rn "\[F12\]" srcs/
```

## Notation

| Symbol | Meaning | Symbol | Meaning |
|---|---|---|---|
| `x` | position (m) | `v` | linear velocity (m/s) |
| `q` | orientation (unit quaternion) | `w` | angular velocity (rad/s) |
| `m` | mass (kg) | `I` | inertia tensor (kg·m²) |
| `F` | force (N) | `T` | torque (N·m) |
| `J` | impulse (N·s = kg·m/s) | `g` | gravity (m/s²) |
| `n` | contact normal (unit, from body `a` toward body `b`) | `r` | lever arm: contact point minus center of mass |
| `e` | elasticity (0 = dead, 1 = perfectly elastic) | `mu` | Coulomb friction coefficient |

`.` is the dot product, `x` the cross product and `^T` the transpose.

## Integration

`srcs/physics/integrator.c`, `srcs/math/quat.c`

### [F1] Newton's second law, linear and angular

```
a = F / m + g                  alpha = I_world^-1 . T
```

The engine stores `1/m` and `I^-1` instead of `m` and `I`: every use is a
division, and a static body is simply `1/m = 0`, `I^-1 = 0`.

### [F2] Semi-implicit (symplectic) Euler, one fixed step dt

```
v += a * dt                    x += v * dt      <- NEW v
w += alpha * dt                q  = integrate(q, w, dt)
```

Updating the velocity first and moving with the new velocity is what keeps
stacks and bounces stable; explicit Euler (`x += old v * dt`) injects energy and
blows them up.

### [F3] Quaternion integration of an angular velocity

```
dq/dt = 0.5 * (0, w) * q       q = normalize(q + dq * dt)
```

The pure quaternion `(0, w)` times `q` is the rotation rate; normalizing every
step removes the drift the first-order step introduces.

### [F4] Exponential angular damping over a variable step

```
w *= ANGULAR_DAMPING^dt
```

Bleeds the energy the contact solver cannot see (rolling resistance, air) so
spinning bodies can come to rest and fall asleep.

## Mass and inertia

`srcs/collision/collider.c`, `srcs/physics/rigidbody.c`

### [F5] Inertia of a solid sphere of radius R about any axis through its center

```
I = (2/5) m R^2                 (I^-1 is a diagonal of 1/I)
```

### [F6] Inertia of a solid box of half extents (hx, hy, hz)

```
Ixx = (m/3) (hy^2 + hz^2)
Iyy = (m/3) (hx^2 + hz^2)
Izz = (m/3) (hx^2 + hy^2)
```

(The usual `(m/12)(w^2 + d^2)` with full extents `w = 2hx`, `d = 2hz`.)

### [F7] Inertia in world space, refreshed whenever the body rotates

```
I_world^-1 = R . I_local^-1 . R^T          R = rotation of q
```

Torque and impulses act in world space, the tensor is constant only in body
space; this conjugation moves it between the two.

### [F8] Torque of a force applied away from the center of mass

```
T = r x F                       (r = application point - x)
```

This is why an off-center hit spins a body instead of just pushing it.

### [F9] Impulse applied at a point: an instant change of both velocities

```
v += J / m                      w += I_world^-1 . (r x J)
```

## Broad-phase

`srcs/collision/broadphase.c`

### [F10] World AABB of a collider

```
sphere:  x +- R  on each axis
box:     x +- (|R| . h), the rotated half extents, i.e. the absolute
         value of the rotation matrix times h
plane:   infinite (never culled)
```

### [F11] Interval overlap on one axis (sweep and prune sorts on X, tests Y/Z)

```
a.min <= b.max  AND  b.min <= a.max
```

## Narrow-phase

`srcs/collision/narrowphase.c`, `srcs/collision/narrowphase_box.c`

### [F12] Sphere against sphere

```
d = xb - xa,  overlap when |d| < Ra + Rb
n = d / |d|,  penetration = Ra + Rb - |d|
contact point = xa + n * Ra
```

### [F13] Sphere against plane (plane: n . x = offset)

```
distance = n_plane . x_sphere - offset
overlap when distance < R,  penetration = R - distance
```

### [F14] Sphere against box: closest point on the box to the sphere center

```
p = x_box + R_box . clamp(R_box^T (x_sphere - x_box), -h, +h)
overlap when |x_sphere - p| < R
```

Clamping in the box's local frame is what makes this exact for faces, edges and
corners alike.

### [F15] Separating axis theorem (SAT) for two oriented boxes

For each candidate axis `L` (3 faces of `a`, 3 faces of `b`, 9 edge cross
products):

```
ra = sum |h_a[i] * (A_i . L)|      (projected radius of a)
rb = sum |h_b[i] * (B_i . L)|
separation = |(xb - xa) . L| - (ra + rb)
```

Any axis with `separation > 0` proves no contact. Otherwise the axis of LEAST
separation is the contact normal and its value is the penetration depth.

### [F16] Face contact manifold: Sutherland-Hodgman clipping

The incident face (the one most anti-parallel to the normal) is clipped against
the side planes of the reference face; the points that stay behind the reference
plane are the contact points.

## Contact response

`srcs/collision/resolver.c`

### [F17] Velocity of a body at a world point (contact points are not centers)

```
v_point = v + w x r
```

### [F18] Relative velocity at a contact, and its normal component

```
v_rel   = v_point(b) - v_point(a)
v_rel_n = v_rel . n
```

Negative means the bodies are approaching along the normal.

### [F19] Effective mass along a direction d

How much impulse that direction costs, linear and angular together:

```
1/mD = 1/ma + 1/mb
       + d . [ (Ia^-1 (ra x d)) x ra + (Ib^-1 (rb x d)) x rb ]
```

The stored `massNormal` / `massT1` / `massT2` are the reciprocal, `mD`.

### [F20] Elasticity target (Newton's impact law)

Computed BEFORE any impulse of the step is applied:

```
bias = e * v_rel_n            when v_rel_n < -ELASTICITY_THRESHOLD
bias = 0                      otherwise (slow touch: no bounce)
```

Measuring it afterwards would turn warm-start transients into fake bounces, and
the threshold is what stops the endless micro-bouncing of a body at rest.

### [F21] Normal impulse of one solver iteration, with accumulated clamping

```
lambda   = -(v_rel_n + bias) * mN
jn_new   = max(jn_old + lambda, 0)        <- contacts only push
lambda   = jn_new - jn_old                <- what to apply now
v_a -= lambda * n / ma ...                 (see [F9])
```

Clamping the ACCUMULATED impulse instead of each increment is what lets
Gauss-Seidel iterations converge on a stack instead of fighting.

### [F22] Coulomb friction, one impulse per tangent direction

```
|jt| <= mu * jn,        mu = sqrt(mu_a * mu_b)
```

Same accumulate-then-clamp scheme as [F21], with the bound following the normal
impulse of the very same iteration.

### [F23] Positional correction (Baumgarte-style, on positions not velocities)

```
c = min(max(penetration - SLOP, 0), MAX_CORRECTION) * PERCENT * mN
x  += -/+ c * n / m            q  rotates by I^-1 (r x c n)
```

Removing the leftover overlap by moving bodies, through the same effective
masses, avoids the energy a velocity-based push would add.

## Resting

`srcs/physics/world.c`

### [F24] Sleep test, accumulated per body but decided per contact island

```
|v| < SLEEP_LINEAR_EPS  AND  |w| < SLEEP_ANGULAR_EPS
for SLEEP_TIME seconds  =>  the whole island sleeps
```

Islands (bodies transitively in contact) sleep together: sending a body to sleep
on its own would drop the support under its neighbours.

## Ballistics

`srcs/game/trebuchet.c`

### [F25] Launch velocity from the aiming controls

```
v0 = speed * (cos(angle), sin(angle), 0)
```

The engine then owns the trajectory: no projectile path is scripted, gravity in
[F1]/[F2] produces the parabola on its own.
