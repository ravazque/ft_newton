*This project has been created as part of the 42 curriculum by ravazque and acerezo-.*

---

## Description

ft_newton is a **3D rigid-body physics engine** written from scratch in C, in the spirit of PhysX, Havok or Bullet, demonstrated through an Angry-Birds-style game: a catapult launches apples against unstable composed structures, and every reaction — translation, rotation, elastic collisions, gravity — is computed by the engine itself.

Rendering is done on the GPU with **OpenGL 3.3 Core** (GLFW for the window and input, GLAD as the function loader), using hand-written shaders and an explicit render pipeline. All the mathematics (vectors, matrices, quaternions, inertia tensors) and all the physics (integration, collision detection and response) are implemented manually — no external calculation or collision library is used.

The engine features:

- Three primitive colliders: **sphere, box and plane**, with a broad-phase / narrow-phase collision pipeline that survives many simultaneous objects.
- Rigid-body dynamics with a fixed-timestep integrator, impulse-based collision response and a system that settles back to a stable state.
- Live controls over time scale, gravity, and the projectile's speed, direction and mass — plus on-demand spawning of structures.
- An FPS counter, an object counter, and a toggleable debug display that renders every collider as a wireframe.

Gameplay happens on a 2D plane, but the physics and the renderer are fully 3D. A single `make` builds the project.
