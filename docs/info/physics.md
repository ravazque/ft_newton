# Physics

Every formula the engine implements, once, with a tag, `[F1]` to `[F39]`. The line of code that implements a formula carries the same tag in a comment:

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

A rigid ball rolling without slipping never slides, so friction alone never slows it down. A real ball and the ground deform slightly where they touch, which creates a torque against the rolling, `T = c_rr N R`. The engine applies that torque as a clamped angular impulse. The rolling ball then decelerates at `a = c_rr g / (1 + 2/5)`, about 1.05 m/s² for the apples (`c_rr = 0.15`).

A ball spinning in place about the contact normal is the same blind spot: its contact point only turns, it never slides, so friction never sees it either. The same patch resists that spin with the same bound, about the normal axis, so the spin drops at `alpha = c_rr g / (2/5 R)`, about 7.4 rad/s² for the apples. Boxes do not roll, so they never get these impulses: when a box tips over an edge its rotation is left untouched.

### [F39] A light body holding up a heavy one

```
lighter body below the heavier one (normal within 84° of the vertical), m_light < m_heavy / MASS_RATIO_LIMIT:
s_light = max(MASS_RATIO_LIMIT · (1/m_heavy) / (1/m_light),  MASS_SCALE_FLOOR)        s_heavy = 1
1/m_eff = s_a (1/m_a + n · (I_a^-1 (r_a x n)) x r_a) + s_b (1/m_b + ...)          as [F19], each body scaled
apply:    -P s_a on a, +P s_b on b                             impulses and positional pushes alike
```

Sequential impulses converge slowly when a body hundreds of times heavier rests on a light one: an 844 kg model on a 0.3 kg forearm pushes the forearm into the ground faster than 16 passes can push back, and the light body ends 15 cm deep. In such a contact the lighter body counts as at most `MASS_RATIO_LIMIT` (10) times lighter than the heavy one, so it holds the heavy body up the way a rigid support does; the heavy body keeps its own mass and stops as it should. Only a lighter body *below* a heavier one is scaled (the normal within 84° of the vertical, `SUPPORT_COSINE` 0.1): a light body lying on a heavy one (the test rain on a model) pushes it with its real weight.

The scale never goes under `MASS_SCALE_FLOOR` (0.1): a body counts as at most 10 times its own mass. A 1 kg block under a 22 t model would otherwise count as 2 300 kg in that contact and 1 kg in its contacts with the ground and the next block; that disagreement feeds on itself and once launched blocks at thousands of m/s. With the floor, the heaviest model the files allow (22.8 t) lands on ragdolls, blocks and 50 g balls without losing any of them.

## Resting

`srcs/physics/sleep.c`

### [F24] Sleep test

```
d = net move since the body was last seen moving,  P = path covered since then (sum of |v| dt),  t = time since then
still     : |v| < SLEEP_LINEAR_EPS (5 cm/s),  or  d < SLEEP_LINEAR_EPS · max(t, 0.5 s)  and  P > 3 d     (shaking in place)
            and the same for the rotation, with SLEEP_ANGULAR_EPS (0.05 rad/s)
creeping  : d < 4 · SLEEP_LINEAR_EPS · max(t, 0.5 s)  and the same for the rotation                    (slow on average)
=> an island sleeps when every body has been still for SLEEP_TIME (0.5 s), or creeping for SLEEP_CREEP_TIME (3 s)
```

Bodies in contact or held by a joint form islands (static bodies do not link islands). A sleeping body skips integration and has zero velocity, and it wakes when an awake body touches it.

The plain speed test alone misses two kinds of rest. A body crushed between the ground and a heavy model can shake in place: its speed swings above the limit every few steps while it does not actually go anywhere (a forearm under a 7 t model: spin peaks of 2 rad/s, 0.25° and 3 mm of net change in 2 s). A path three times longer than the net move tells shaking from moving, and only after the first quarter of a second, so a body just starting to move, whose path equals its move, is never mistaken for one. And a pile under a heavy load can creep a few millimetres a second for a long time; whatever creeps at under four times the sleep speeds for three seconds is put to sleep as well. A block balanced on its edge and tipped by half a degree still falls flat, a rolling apple still rolls until it stops, and a ragdoll still folds up before it sleeps.

## Ballistics

`srcs/game/trebuchet.c`

### [F25] Launch velocity

```
v0 = speed * (cos(angle), sin(angle), 0)
```

The shot is not scripted: from its first step the apple is an ordinary body, and gravity through [F1] and [F2] draws the parabola.

## Mesh colliders

`srcs/physics/mesh_inertia.c`, `srcs/collision/mesh_shape.c`, `srcs/collision/mesh_repair.c`, `srcs/collision/bvh.c`, `srcs/collision/closest.c`, `srcs/collision/mesh_query.c`, `srcs/collision/contact_mesh.c`, `srcs/collision/contact_box_mesh.c`

A model is its own collider: the triangles of the `.obj`, scaled, repaired into a closed surface, moved so the centre of mass is the body's origin, and indexed by a tree. Nothing is approximated by a box, a hull or a set of spheres.

### [F27] Mass properties of a closed surface (divergence theorem)

```
∫_V f dV = ∮_S F · n dS        with  div F = f
V   = Σ  d_t.x (x0 + x1 + x2) / 6           d_t = (p1 - p0) x (p2 - p0), one term per triangle
c   = (1/V) ∫_V x dV                        first moments: one cubic polynomial of the corners per triangle
I_o = ∫_V (|r|² 1 - r rᵀ) dV                second moments: one quartic polynomial per triangle
I_c = I_o - V (|c|² 1 - c cᵀ)               moved to the centre of mass (parallel axis)
```

Every integral over the solid becomes a sum over its triangles (Eberly, *Polyhedral Mass Properties*), so the volume, the centre of mass and the full inertia tensor (products of inertia included) are exact for any closed model. The surface must be closed and wound one way; a model wound inward gives every integral with the opposite sign and is turned outward. The tensor is stored per kilogram, so `I = mass · I/kg` follows the mass the file or the menu gives.

### [F28] Bounding volume tree

```
node box   = union of the boxes of its triangles
split      = median of the triangle centres along the node's longest side    → depth ≤ log2(n / 4) + 1
query      = visit only the nodes whose box meets the query box ([F11])
```

A sphere, a box or a vertex near the model reaches a few leaves instead of every triangle: the 1 768 triangles of the humanoid are a tree of 1 023 nodes.

### [F29] Closest point of a triangle

```
regions of p:  corner a, b, c  |  edge ab, bc, ca  |  face
edge ab:  q = a + t (b - a)                    t = d1 / (d1 - d3)
face:     q = a + v (b - a) + w (c - a)        v = vb / (va + vb + vc),  w = vc / (va + vb + vc)
```

Ericson's test of the Voronoi regions of the triangle (*Real-Time Collision Detection* 5.1.5), with `d1..d6` the projections of `p` on the two edges from each corner. It also says which feature `q` lies on, which [F30] needs.

### [F30] Inside or outside: angle-weighted pseudo-normals

```
n_face   = (p1 - p0) x (p2 - p0) / |...|
n_edge   = normalize(n_1 + n_2)                         the two faces of the edge
n_vertex = normalize(Σ α_i n_i)                         α_i = angle of face i at the vertex
inside   ⇔  (p - q) · n_feature < 0                     q = closest point, on that feature
```

The sign of the plain face normal is wrong near a convex edge or corner; the pseudo-normal of the feature the closest point lies on is right everywhere on a closed surface (Bærentzen and Aanæs). It gives the contacts against meshes:

```
sphere:  every triangle within R:  n = (c - q) / |c - q|,  depth = R - |c - q|
         centre inside the solid:  one contact through the closest feature, depth = R + |c - q|
plane:   every vertex below the surface and over the square ([F13]), reduced like a box
mesh:    every vertex of each inside the other, depth = |p - q|, normal = n_feature
         every edge of each crossing the other's surface ([F37]): the middle of each part inside, as a vertex
```

### [F31] Box against triangle: 13 axes

```
axes      = n_t,  the 3 face normals of the box,  the 9 products  box edge x triangle edge
overlap L = min( c·L + r(L) - min p_i·L ,  max p_i·L - (c·L - r(L)) )         r(L) as in [F15]
n_t       : depth = p0·n_t - (c·n_t - r(n_t)),  one-sided: the box is always pushed along +n_t
```

One negative overlap separates the box from the triangle. The triangle's normal wins unless another axis is clearly better (the bias of [F15]); then the box face facing the triangle is clipped to it ([F16]), or the triangle to a box face, or the two edges give their closest points. Being one-sided, the triangle normal never pushes a box into the solid, and the seams between flat triangles do not catch a sliding box.

### [F37] Where a segment crosses a triangle

```
d = q - p,  e1 = b - a,  e2 = c - a,  h = d x e2,  det = e1 · h        det ≈ 0: parallel, no crossing
u = (p - a) · h / det                                                   inside if 0 ≤ u ≤ 1
v = d · ((p - a) x e1) / det                                            inside if v ≥ 0 and u + v ≤ 1
t = e2 · ((p - a) x e1) / det                                           the crossing is p + t d, 0 ≤ t ≤ 1
```

Möller and Trumbore's test, with the segment instead of a ray. Two models can cross edge to edge with no vertex of either inside the other (two bars crossed like a plus sign); every edge of one is therefore tested against the triangles of the other that the tree hands over. The crossings along an edge alternate entering and leaving, so each pair of them bounds a part of the edge inside the solid; the middle of that part goes through the inside test of [F30] like a vertex, and gives a contact with its depth and normal.

## Joints

`srcs/physics/joint_list.c`, `srcs/response/joint_point.c`, `srcs/response/joint_limit.c`, `srcs/response/correction.c`

A joint holds two bodies at a shared point and keeps their relative rotation within limits. Its frames are stored in each body's space and coincide when the joint is made, which is where every angle is 0. It is solved with the contacts, by the same accumulate-and-clamp sequential impulses.

### [F32] Ball and socket: the shared point

```
C  = (x_b + R_b a_b) - (x_a + R_a a_a) = 0                 a = anchor in each body's frame
Ċ  = v_b + w_b x r_b - v_a - w_a x r_a
K  = (1/m_a + 1/m_b) 1 - [r_a]x I_a^-1 [r_a]x - [r_b]x I_b^-1 [r_b]x
P  = -K^-1 Ċ                  applied +P at b's anchor, -P at a's ([F9])
```

The three directions are solved at once: `K` is the 3x3 form of [F19], `[r]x` the matrix of the cross product. The accumulated `P` is applied again at the start of the next step (warm start).

### [F33] Swing, twist and hinge angle

```
swing  = acos(t_a · t_b)                                   t = twist axis of each body, in the world
q_s    = shortest arc taking t_b onto t_a                  undoes the swing
twist  = atan2( (u_a x q_s u_b) · t_a ,  u_a · q_s u_b )   u = reference perpendicular of each body
hinge  = the same angle, about the hinge axis
```

### [F34] Angular rows: locks and limits

```
k      = u · (I_a^-1 + I_b^-1) u                           u = axis of the row
λ      = -((w_b - w_a) · u - bias) / k
lock   : λ_total free, bias = 0                            a hinge's two directions across its axis
limit  : acts from JOINT_LIMIT_MARGIN (0.05 rad) before it is reached
         λ_total ≥ 0 at the bottom of a range, ≤ 0 at the top
         bias = -gap / dt at the bottom, +gap / dt at the top    gap = what is left before the limit, 0 once past it
apply  : +λ u as an angular impulse on b, -λ u on a
```

A cone has a swing row (about `t_a x t_b`) and a twist row; a hinge has two locks and an angle row. A limit only pushes back, the way a contact only pushes. It starts acting just before it is reached, with a bias that lets the joint close the gap left in one step but not cross it: a limit that only switched on once crossed would flicker on and off every few steps under a resting load, and kick the body each time.

### [F35] Joint drift removed on positions

```
anchors : P = -K^-1 (JOINT_PERCENT · gap)                  gap capped at 20 cm
angles  : turn = JOINT_PERCENT · error / k   about the row axis, b one way and a the other
```

What the impulses leave is closed in the positional passes of [F23], half per pass, measured again on the pose each pass leaves. Like the contact correction it moves poses, not velocities, so it adds no energy.

### [F36] Joint friction

```
J = -JOINT_SPIN_DAMPING · (I_a^-1 + I_b^-1)^-1 (w_b - w_a)          once per step, +J on b, -J on a
```

Each step a joint takes 1% of the relative spin of its two bodies, through their angular effective mass. Without it a limb hanging off a pile is a pendulum that only air damping ([F4]) slows down, and the whole pile stays awake for minutes. It does not hold a pose: gravity still folds the ragdoll, only the swinging dies out.

### [F38] Joint projection

```
after the positional passes, for each joint, parent before child, twice:
gap = |anchor_b - anchor_a|  >  JOINT_PROJECTION_DISTANCE (1 cm)  →  x_b -= (gap - 1 cm) · direction
                                                                       v_b loses its part moving the anchors apart
error past a limit  >  JOINT_PROJECTION_ANGLE (5°)                 →  b turned back about the row axis, around the anchor
                                                                       w_b loses its part turning further past
```

The impulses and the positional passes are approximate: without projection, a 20 kg apple at 30 m/s left a joint open by 11 cm and a limb 39° past its limit for a few steps, and a pile of 12 ragdolls kept the anchors of the bottom one 3.6 cm open at rest. Projection bounds what any step can leave: the child body is put back within 1 cm of its anchor and within 5° of its limits, and loses the velocity that would open the joint again. It only acts beyond those bounds: a single ragdoll at rest never reaches them, and a pile whose bottom joints sit at the bound still comes to rest (12 ragdolls asleep within 20 s).
