# Collisions

How the engine finds out which bodies touch, what it does about it, and how it avoids the classic problems of collision handling.

A collision is handled in three stages every physics step:

```
broad-phase  ->  narrow-phase  ->  response
"who might touch?"   "where exactly, how deep?"   "what impulses fix it?"
```

## 1. Colliders

Each body has one of three shapes (`srcs/collision/collider.c`):

| Shape | Defined by | Used for |
|---|---|---|
| Sphere | radius | apples |
| Box | half extents, oriented by the body's quaternion | blocks, the trebuchet's parts |
| Plane | normal, offset (`n . x = offset`) and a square extent | the ground |

The plane is **bounded**: it covers exactly the square that is drawn, and it counts as solid down to `PLANE_THICKNESS` (2 m) under its surface. A body whose support point is past the edge is not held up. It falls, and once it is 20 m under the ground it is removed from the world (`srcs/physics/cull.c`), which also removes it from the object counter and from every later collision test.

## 2. Broad-phase: sweep and prune

`srcs/collision/broadphase.c`

Checking every pair of `n` bodies exactly costs `n²/2` checks: about 45 000 for 300 blocks. The broad-phase cuts that down with a cheap, conservative filter.

1. Each body gets an **axis-aligned bounding box** (AABB) [F10]. For a rotated box it is the box of its rotated extents; for the plane it is the ground square extruded 2 m downward.
2. The AABBs are **sorted by their minimum X**.
3. Walking the sorted list, each box is compared only with the following ones that start before it ends on X. As soon as one starts after it ends, no later box can overlap it on X, so the inner loop stops.
4. The survivors must also overlap on Y and Z [F11], and at least one of the two bodies must be **awake and dynamic**. Two static bodies, or two sleeping ones, cannot change anything and are skipped.

The cost is `O(n log n)` for the sort plus the pairs actually found, instead of `O(n²)`. With 321 bodies awake the step takes a few milliseconds, and a scene at rest produces no pairs at all.

The pairs are stored with the lower index first and sorted, so a given pair keeps the same orientation (and contact normal) from one step to the next. The solver relies on that.

## 3. Narrow-phase: the exact checks

`srcs/collision/narrowphase.c` dispatches each pair to its test. Every test writes contacts with the normal from its first body toward its second, and the dispatcher flips it when the arguments were swapped, so a contact's normal always points from `a` to `b`.

A contact is `(normal, point, depth)`. A pair gives up to four.

| Pair | Test | File |
|---|---|---|
| sphere / sphere | distance between centers against the sum of radii [F12] | `contact_sphere.c` |
| sphere / plane | signed distance of the center, and the center over the square [F13] | `contact_sphere.c` |
| sphere / box | closest point of the box, clamped in the box's frame [F14] | `contact_sphere.c` |
| box / plane | each of the 8 corners below the surface and over the square [F13] | `contact_box.c` |
| box / box | separating axis test [F15], then face clipping [F16] or an edge contact | `sat.c`, `contact_box.c`, `manifold.c` |

### Box against box

`sat.c` projects both boxes onto 15 axes: the 3 face normals of each box and the 9 cross products of their edges. If the two shadows are disjoint on any axis, that axis separates the boxes and there is no contact. Otherwise the axis with the smallest overlap is the normal and the overlap is the depth.

- **Face normal** (a resting stack, a box lying on another): the face of the other box most opposed to the normal is clipped against the side planes of the reference face (Sutherland-Hodgman). The points that end up below the reference face are the contacts (`contact_box.c`, `manifold.c`).
- **Edge normal** (two boxes crossing edge to edge): one contact, midway between the closest points of the two edges.

A box resting flat can produce up to eight candidate points. They are reduced to four spread ones: the deepest, the farthest from it, and the two making the largest triangles on either side. Four well spread points hold a box flat; four points bunched on one side would make it rock.

## 4. Response: sequential impulses

`srcs/response/resolver.c`, `impulse.c`, `correction.c`

Every step, over all the contacts:

1. **Wake**: a sleeping body touched by an awake dynamic body wakes up.
2. **Prepare** each contact: two friction directions, the effective masses [F19], and the elasticity target [F20], read from the velocities before any impulse of the step.
3. **Warm start**: a contact that existed last step at almost the same point (within 5 cm) starts from last step's impulses. A resting stack then converges over steps instead of being re-solved from zero every step.
4. **Iterate** 16 times over all contacts (Gauss-Seidel). For each contact:
   - rolling resistance, if a sphere is involved [F26];
   - friction on the two tangent directions, bounded by `mu * jn` [F22];
   - the normal impulse, which only pushes [F21].

   Each impulse acts at the contact point, on both bodies, in opposite directions [F9]. That is where rotation comes from.
5. **Correct positions**: the overlap left beyond 5 mm is removed in 3 passes, moving and slightly turning the bodies [F23].
6. **Remember** the contacts and their impulses for the next step's warm start.

### Why the scene always comes back to rest

- Slow impacts (under 1 m/s) do not bounce, so a body does not micro-bounce forever.
- Friction removes sliding energy, and rolling resistance removes rolling energy from spheres.
- Angular damping removes the spin nothing else sees.
- **Island sleeping** (`srcs/physics/sleep.c`): bodies linked by contacts form an island, and the island sleeps when all of its bodies have stayed below 5 cm/s and 0.05 rad/s for half a second. A pile switches off as a whole; putting its boxes to sleep one by one would pull the support from under their neighbours.

## 5. Placing new bodies without overlaps

`srcs/collision/query.c`, `srcs/game/structure.c`, `srcs/game/trebuchet.c`, `srcs/game/scene.c`

A body born inside another one starts with a deep overlap, and the positional correction pushes the two apart violently. The engine never creates one:

- **Structures (keys 1 to 4)** appear at a fixed spot. If any of their blocks would sink into something there, the whole structure is lifted to rest on top of it, a centimetre above, and lands on it.
- **Apples**: if a new apple would overlap anything at the launch point (for instance the previous apple fired at very low speed), it is not created, and the terminal says why.
- **The starting scene** is checked when the files are loaded. A pyramid that would overlap the trebuchet, sink into the ground or stand outside the ground square is refused, with the reason and the files involved.

"Overlap" means deeper than 1 mm: bodies that just touch, like blocks stacked with their small gap, are fine.

## 6. Classic collision problems and what the engine does

| Problem | What it looks like | How it is handled here |
|---|---|---|
| **Tunneling** | a fast body passes through a thin one between two steps | fixed 1/120 s step whatever the time scale; launch speed capped at 60 m/s, which is 0.5 m per step, well inside a 1 m block plus a 0.5 m apple; the plane is solid 2 m deep, so something that sank in is pushed back out, not through |
| **Jitter** | resting bodies shake | accumulated impulses with warm starting; contacts kept in the same order every step; a face normal preferred over an edge normal unless clearly better; 4 well spread manifold points |
| **Sinking stacks** | a tower slowly sinks into itself | several passes of positional correction per step, so a push travels up the stack within one step |
| **Energy gain and explosions** | a push sends bodies flying | correction on positions (not velocities), capped at 20 cm per contact and pass; elasticity read before the step's impulses; new bodies never created overlapping |
| **Endless bouncing and rolling** | nothing ever settles | bounce threshold, friction, rolling resistance, angular damping and island sleeping |
| **Missed pairs** | two bodies overlap but never collide | the broad-phase is conservative (AABBs contain their shapes) and inclusive (touching boxes count) |
| **Wrong normal sign** | bodies pulled into each other | every test writes its normal from its first body to its second; swapped pairs are flipped; pairs stored in index order |
| **Stale contact memory** | a removed body's impulse reused by another pair | removing a body forgets the remembered contacts of its index and of the body moved into that index |
| **Numerical drift** | orientation no longer a rotation, energy creeping | quaternion renormalized every step; semi-implicit Euler |

## 7. Seeing it

`F1` draws every collider as a wireframe over the scene: spheres, boxes, and the ground square that is the actual extent of the plane. Slow motion (`T`) shows impacts step by step, and pause (`P`) freezes a contact to look at it from any angle with the free camera.
