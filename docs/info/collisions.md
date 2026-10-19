# Collisions

How the engine finds out which bodies touch, what it does about it, and how it avoids the classic problems of collision handling.

A collision is handled in three stages every physics step:

```
broad-phase  ->  narrow-phase  ->  response
"who might touch?"   "where exactly, how deep?"   "what impulses fix it?"
```

## 1. Colliders

Each body has one of four shapes (`srcs/collision/collider.c`):

| Shape | Defined by | Used for |
|---|---|---|
| Sphere | radius | apples, the test rain, the ragdoll's head |
| Box | half extents, oriented by the body's quaternion | blocks, the trebuchet's parts, the ragdoll's other parts |
| Mesh | the triangles of an `.obj` model, oriented by the body's quaternion | every model of `assets/models/` (the humanoid, the logo and the torus ship) |
| Plane | normal, offset (`n . x = offset`) and a square extent | the ground |

The plane is **bounded**: it covers exactly the square that is drawn, and it counts as solid down to `PLANE_THICKNESS` (2 m) under its surface. A body whose support point is past the edge is not held up. It falls, and once it is 20 m under the ground it is removed from the world (`srcs/physics/cull.c`), which also removes it from the object counter and from every later collision test.

## 2. Broad-phase: sweep and prune

`srcs/collision/broadphase.c`

Checking every pair of `n` bodies exactly costs `n²/2` checks: about 45 000 for 300 blocks. The broad-phase cuts that down with a cheap, conservative filter.

1. Each body gets an **axis-aligned bounding box** (AABB) [F10]. For a rotated box it is the box of its rotated extents; for the plane it is the ground square extruded 2 m downward.
2. The AABBs are **sorted by their minimum X**.
3. Walking the sorted list, each box is compared only with the following ones that start before it ends on X. As soon as one starts after it ends, no later box can overlap it on X, so the inner loop stops.
4. The survivors must also overlap on Y and Z [F11], and at least one of the two bodies must be **awake and dynamic**. Two static bodies, or two sleeping ones, cannot change anything and are skipped, and so are two parts of a ragdoll whose joint keeps them apart (their boxes overlap around the shared point).

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
| sphere / mesh | closest point of every nearby triangle [F29]; a centre inside the solid is pushed out through the closest feature [F30] | `contact_mesh.c` |
| box / mesh | the 13-axis test of every nearby triangle [F31], then clipping [F16] or an edge contact | `contact_box_mesh.c` |
| mesh / plane | each vertex below the surface and over the square [F13] | `contact_mesh.c` |
| mesh / mesh | each vertex of one inside the other, and each edge of one crossing the other's surface [F37], through the inside test [F30] | `contact_mesh.c` |

### Box against box

`sat.c` projects both boxes onto 15 axes: the 3 face normals of each box and the 9 cross products of their edges. If the two shadows are disjoint on any axis, that axis separates the boxes and there is no contact. Otherwise the axis with the smallest overlap is the normal and the overlap is the depth.

- **Face normal** (a resting stack, a box lying on another): the face of the other box most opposed to the normal is clipped against the side planes of the reference face (Sutherland-Hodgman). The points that end up below the reference face are the contacts (`contact_box.c`, `manifold.c`).
- **Edge normal** (two boxes crossing edge to edge): one contact, midway between the closest points of the two edges.

A box resting flat can produce up to eight candidate points. They are reduced to four spread ones: the deepest, the farthest from it, and the two making the largest triangles on either side. Four well spread points hold a box flat; four points bunched on one side would make it rock.

### Mesh colliders

A model is its own collider: the triangles of its `.obj` file, not a box, a hull or a set of spheres around it. `srcs/collision/mesh_shape.c` turns a model into one, once, at start-up:

1. the `.obj` is read (vertices first, then the faces, each polygon cut into triangles by ear clipping) and only the vertices a face uses are kept;
2. it is scaled so the longest side of its box is the `size` of `assets/models.csv`;
3. it is made a closed surface wound outward (`mesh_repair.c`): split vertices are welded, repeated faces dropped, each piece oriented, holes closed by fans and inward pieces turned; then every edge is shared by exactly two triangles running it in opposite directions (`mesh_topology.c`), so the surface has an inside. A surface that cannot be closed is refused and the model skipped;
4. its volume, centre of mass and inertia tensor come from the surface [F27], and it is moved so the centre of mass is the body's origin;
5. a bounding volume tree indexes its triangles [F28].

Every body built from the model shares that surface. A pair is tested in the mesh's own frame: the other body is brought into it, the tree hands over the triangles near it, and the contacts go back to the world. Whether a point is inside the solid is decided by the angle-weighted pseudo-normal of the closest feature [F30], exact on a closed surface; that is what lets a ball that went fast and deep still be pushed out on the right side, and what finds the vertices of one model inside another.

A mesh is dynamic like any body: it falls, slides, tips over and sleeps. Against another mesh, the vertices of each inside the other and the edges of each crossing the other's surface are both tested [F37], so two models crossing edge to edge with no vertex inside the other (two bars crossed like a plus sign) touch exactly where their edges cross, and a model placed across another is lifted onto it instead of starting inside it.

## 4. Response: sequential impulses

`srcs/response/resolver.c`, `impulse.c`, `correction.c`

Every step, over all the contacts:

1. **Wake**: a sleeping body touched by, or jointed to, an awake dynamic body wakes up.
2. **Prepare** each contact: two friction directions, the effective masses [F19], and the elasticity target [F20], read from the velocities before any impulse of the step. Where a light body holds up one more than 10 times heavier, the light one counts as only 10 times lighter, and at most 10 times its own mass [F39]. Each joint gets its 3x3 point mass [F32] and the rows of its locks and of the limits it is about to reach [F33] [F34], applies last step's impulses again, and takes its share of the relative spin [F36].
3. **Warm start**: a contact that existed last step at almost the same point (within 5 cm) starts from last step's impulses. A resting stack then converges over steps instead of being re-solved from zero every step.
4. **Iterate** 16 times over the joints, then over all contacts (Gauss-Seidel). For each joint: the shared point, then the locks and the reached limits. For each contact:
   - rolling resistance about the two tangent axes and the normal, if a sphere is involved [F26];
   - friction on the two tangent directions, bounded by `mu * jn` [F22];
   - the normal impulse, which only pushes [F21].

   Each impulse acts at the contact point, on both bodies, in opposite directions [F9]. That is where rotation comes from.
5. **Correct positions**: in each of 3 passes the joints close half of their drift [F35], then the overlap left beyond 5 mm is reduced, moving and slightly turning the bodies [F23].
6. **Project joints**: an anchor still open by more than 1 cm, or a limit passed by more than 5°, puts the child body back within those bounds [F38].
7. **Remember** the contacts and their impulses for the next step's warm start.

### Why the scene always comes back to rest

- Slow impacts (under 1 m/s) do not bounce, so a body does not micro-bounce forever.
- Friction removes sliding energy, and rolling resistance removes the rolling and the spinning in place of spheres.
- Angular damping removes the spin nothing else sees.
- **Island sleeping** (`srcs/physics/sleep.c`): bodies linked by contacts or joints form an island, and the island sleeps when all of its bodies have stayed below 5 cm/s and 0.05 rad/s for half a second. A pile switches off as a whole; putting its boxes to sleep one by one would pull the support from under their neighbours. A ragdoll is one island. A body shaking in place without going anywhere counts as still, and an island creeping at under four times the sleep speeds sleeps after three seconds [F24], so a ragdoll crushed under a heavy model does not keep everything awake.

## 5. Placing new bodies without overlaps

`srcs/collision/overlap.c`, `srcs/game/placement.c`, `srcs/game/trebuchet.c`, `srcs/game/scene_check.c`, `srcs/game/camera_collision.c`

A body born inside another one starts with a deep overlap, and the positional correction pushes the two apart violently. The engine never creates one:

- **Structures (keys 1 to 4)** appear at a fixed spot. If any of their blocks would sink into something there, the whole structure is lifted to rest on top of it, a centimetre above, and lands on it. The ragdoll, the models and the test rain (keys 5, 6 and 9, and the models page of the menu) are placed the same way.
- **Apples**: if a new apple would overlap anything at the launch point (for instance the previous apple fired at very low speed), it is not created, and the terminal says why.
- **The starting scene** is checked when the files are loaded, every problem reported with the files involved and the position: a trebuchet or a pyramid that would sink into the ground, overlap each other or stand outside the ground square, an apple spawn point taken by another body, a camera that would start under the ground or inside a body. The program then exits with status 1, and **Reload from files** keeps the running scene.
- **The camera** is a 0.3 m sphere that bodies push out: moved in 10 cm steps, pushed back along the deepest contact with each body after each step and each physics step, out of a model that swallows it through the closest point of the surface, and to the nearest free spot when pinned. It never affects the bodies.

"Overlap" means deeper than 1 mm: bodies that just touch, like blocks stacked with their small gap, are fine. A body swallowed whole by a model touches none of its triangles, so its centre is also checked against the inside of the model [F30]: a model never appears around a ball or a block.

## 6. Classic collision problems and what the engine does

| Problem | What it looks like | How it is handled here |
|---|---|---|
| **Tunneling** | a fast body passes through a thin one between two steps | fixed 1/120 s step whatever the time scale; launch speed capped at 60 m/s, which is 0.5 m per step, well inside a 1 m block plus a 0.5 m apple; an apple's radius is at least 0.25 m, so it never moves more than its own diameter per step and cannot skip even a thin forearm or a single triangle; the plane is solid 2 m deep, so something that sank in is pushed back out, not through |
| **Jitter** | resting bodies shake | accumulated impulses with warm starting; contacts kept in the same order every step; a face normal preferred over an edge normal unless clearly better; 4 well spread manifold points, and depths within 1 mm count as equal, so a body lying on many points does not swap its 4 every step |
| **Sinking stacks** | a tower slowly sinks into itself | several passes of positional correction per step, so a push travels up the stack within one step |
| **Energy gain and explosions** | a push sends bodies flying | correction on positions (not velocities), capped at 20 cm per contact and pass; elasticity read before the step's impulses; new bodies never created overlapping |
| **Endless bouncing and rolling** | nothing ever settles | bounce threshold, friction, rolling resistance, angular damping and island sleeping |
| **Missed pairs** | two bodies overlap but never collide | the broad-phase is conservative (AABBs contain their shapes) and inclusive (touching boxes count) |
| **Wrong normal sign** | bodies pulled into each other | every test writes its normal from its first body to its second; swapped pairs are flipped; pairs stored in index order |
| **Stale contact memory** | a removed body's impulse reused by another pair | removing a body forgets the remembered contacts of its index and of the body moved into that index |
| **Numerical drift** | orientation no longer a rotation, energy creeping | quaternion renormalized every step; semi-implicit Euler |
| **Ghost collisions on mesh seams** | a box sliding over a flat model catches on the edges between its triangles | the triangle's own normal is preferred and one-sided [F31]; another axis wins only when clearly better |
| **Hulls that lie** | a ball stopped in the air over the hole of a ring | the collider is the model's own triangles, not a hull or a box around them |
| **Thin parts of a model** | a ball whose centre went past the surface pushed out the far side | the inside test of the closest feature [F30] pushes it back out the side it came from |
| **Joints coming apart** | the limbs of a ragdoll separate, or bend past their limits, under a violent impact or a heavy load | the shared point is solved as one 3x3 constraint [F32], warm started, and its drift closed on positions [F35]; whatever a step still leaves beyond 1 cm, or 5° past a limit, is projected back [F38], even for a 50 kg apple at 60 m/s |
| **Heavy on light** | an 844 kg model sinks 15 cm into the ragdoll under it | a light body under a heavy one counts as at most 10 times lighter, so it holds the heavy one up like a rigid support; never more than 10 times its own mass, so its other contacts still agree with it [F39] |
| **Edges crossing** | two models pass through each other where only their edges cross | every edge of each mesh is tested against the other's triangles [F37] |
| **A pile of ragdolls that never rests** | a limb keeps twitching or swinging | limits act just before they are reached, so they do not flicker on and off [F34]; joint friction stops a hanging limb from swinging [F36]; a ragdoll crushed under a heavy model may shake in place without going anywhere, which the sleep test counts as rest [F24] |

## 7. Seeing it

`F1` draws every collider as a wireframe over the scene: spheres, boxes, the triangles of each model, and the ground square that is the actual extent of the plane. It also draws every contact of the last step: a red marker where it is and a cyan stick along its normal. Slow motion (`T`) shows impacts step by step, and pause (`P`) freezes a contact to look at it from any angle with the free camera.

## 8. Showing the precision of a mesh collider

A collider is precise when things touch the model exactly where its surface is, and nowhere else. The scene shows it without recompiling:

1. `6` (or a click on the models page of the menu) places the selected model at x = 2 m; `7` and `8` change the selection.
2. `9` drops the test rain on it: 196 small balls (7 cm) in a 14 x 14 square, a metre above the highest body there.
3. `F1`, `T` and `P` show the triangles, the contacts and their normals while the balls land.

What each shipped model shows:

| Model | Where a box or a hull would fail | What the triangles do |
|---|---|---|
| Humanoid | balls would rest on the box around it, in the air beside the arms and between the legs | balls hit the head, the shoulders, the arms, and fall between the legs and under the arms to the ground |
| Logo | balls would rest on the hull, a flat lid over the open parts of its strokes | balls stay on top of the strokes and drop into the open parts, onto the strokes below |
| Torus | a hull would close the hole | balls fall through the hole; the ones on the ring roll off its curved top |

With the contacts drawn, every red marker sits on the surface, and every cyan normal is perpendicular to the triangle under it, or points to the ball's centre at an edge or a corner. Apples fired low at the models (`I` `K` `J` `L`) show the same at speed.

Measured over the shipped models and scop's (teapot, crown, star prism) with 196 balls each: no ball ends with its centre inside a model, the deepest overlap at rest is 6 mm (the contact slop is 5 mm), and every model and ball comes to rest and sleeps. The inside test agrees with the exact distance function of a torus on every one of 7 020 points sampled within 25 cm of its surface.
