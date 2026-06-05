/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   newton.h                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: ravazque <ravazque@student.42madrid.com    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2026/06/02 22:53:53 by ravazque          #+#    #+#             */
/*   Updated: 2026/06/05 13:33:37 by ravazque         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef NEWTON_H
# define NEWTON_H

# include "defines.h"

# include <stdlib.h>
# include <string.h>
# include <math.h>
# include <stdio.h>
# include <glad/gl.h>
# include <GLFW/glfw3.h>

/*
 * Single public header for the whole engine. Every .c includes only this file.
 * It contains: the type definitions (structs/enums) and ALL function
 * declarations, grouped 00:00:00 by module. Implementations live under srcs, one .c per
 * area (math, render, physics, collision, game).
 *
 * Naming convention:
 *   - Types are PascalCase:     Vec3, Mat4, RigidBody, Renderer ...
 *   - Functions are snake_case with a module prefix:
 *       vec3_add, mat4_perspective, quat_normalized, mesh_cube,
 *       window_init, renderer_draw, rb_integrate, world_step ...
 *
 * Status legend used in this header:
 *   [DONE]  implemented and ready to use
 *   [TODO]  a stub for you to implement (the physics / math you must write)
*/

/* Forward declaration so this header does not drag GLFW/GLAD into every .c.
 * Only the render .c files (and input handling) include the real GLFW headers. */
struct GLFWwindow;

/* ========================================================================== */
/*  MATH TYPES                                                                 */
/* ========================================================================== */

/* 3D vector: positions, velocities, forces, torques, normals. */
typedef struct Vec3
{
	float x;
	float y;
	float z;
}	Vec3;

/* 3x3 matrix, column-major: element (col c, row r) lives at m[c * 3 + r].
 * Used for the inertia tensor and its world-space rotation R*I*R^T. */
typedef struct Mat3
{
	float m[9];
}	Mat3;

/* 4x4 matrix, column-major: element (col c, row r) lives at m[c * 4 + r].
 * This is the layout OpenGL expects, so it uploads to a uniform with no
 * transpose. It carries the Model/View/Projection transforms. */
typedef struct Mat4
{
	float m[16];
}	Mat4;

/* Unit quaternion: a body's orientation, stored as (w, x, y, z). Quaternions
 * avoid gimbal lock and integrate angular velocity cheaply and stably. */
typedef struct Quat
{
	float w;
	float x;
	float y;
	float z;
}	Quat;

/* ========================================================================== */
/*  RENDER TYPES                                                               */
/* ========================================================================== */

/* Owns the OS window + its OpenGL 3.3 Core context (created with GLFW, loaded
 * with GLAD). The window is the framebuffer the GPU draws into. */
typedef struct Window
{
	struct GLFWwindow	*handle;
	int					width;
	int					height;
}	Window;

/* A compiled+linked GPU program (vertex shader + fragment shader). */
typedef struct Shader
{
	unsigned int	program;
}	Shader;

/* Geometry living in GPU memory. Bundles the three OpenGL objects:
 *   vbo: raw vertex data (position + normal)
 *   ebo: indices joining vertices into triangles
 *   vao: the "recipe" describing how to read the vbo */
typedef struct Mesh
{
	unsigned int	vao;
	unsigned int	vbo;
	unsigned int	ebo;
	int				indexCount;
}	Mesh;

/* Produces the View and Projection matrices. 3D even if gameplay is 2D. */
typedef struct Camera
{
	Vec3	position;
	Vec3	target;
	Vec3	up;
	float	fovYDegrees;
	float	nearPlane;
	float	farPlane;
}	Camera;

/* Drawing front-end: hides the OpenGL state machine and owns the shader. */
typedef struct Renderer
{
	Shader	shader;
	Mat4	view;
	Mat4	proj;
}	Renderer;

/* ========================================================================== */
/*  COLLISION TYPES                                                            */
/* ========================================================================== */

/* The 3 mandatory primitive shapes. A plain enum (no virtual dispatch) keeps an
 * array of bodies compact and lets the narrow-phase switch on 'type'. */
typedef enum ShapeType
{
	SHAPE_SPHERE,
	SHAPE_BOX,
	SHAPE_PLANE
}	ShapeType;

/* Shape + dimensions of a body. Only the fields relevant to 'type' are read. */
typedef struct Collider
{
	ShapeType	type;
	float		radius;       /* SHAPE_SPHERE                          */
	Vec3		halfExtents;  /* SHAPE_BOX (half-size on each axis)    */
	Vec3		normal;       /* SHAPE_PLANE orientation               */
	float		offset;       /* SHAPE_PLANE distance along the normal */
}	Collider;

/* A broad-phase candidate: two bodies that might overlap (indices into World). */
typedef struct Pair
{
	int	a;
	int	b;
}	Pair;

/* One contact point produced 00:00:00 by the narrow-phase and consumed 00:00:00 by the resolver. */
typedef struct Contact
{
	int		a;
	int		b;
	Vec3	normal;       /* unit vector from A toward B           */
	Vec3	point;        /* contact location in world space       */
	float	penetration;  /* overlap depth along 'normal'          */
}	Contact;

/* ========================================================================== */
/*  PHYSICS TYPES                                                              */
/* ========================================================================== */

/* The core data the whole engine revolves around: an object that translates and
 * rotates but never deforms. The renderer reads position + orientation; the
 * simulation drives everything else. */
typedef struct RigidBody
{
	/* linear state */
	Vec3	position;
	Vec3	velocity;
	Vec3	forceAccum;
	float	mass;
	float	invMass;          /* 1/mass; 0 => infinite mass (static) */
	/* angular state */
	Quat	orientation;
	Vec3	angularVelocity;
	Vec3	torqueAccum;
	Mat3	invInertiaLocal;  /* inverse inertia in body space (const) */
	Mat3	invInertiaWorld;  /* R * invInertiaLocal * R^T, per step   */
	/* material */
	float	restitution;      /* 0 = no bounce, 1 = perfectly elastic */
	/* shape */
	Collider	collider;
	/* sleeping: resting bodies stop integrating (kills jitter, saves CPU) */
	int		awake;
	float	sleepTimer;
}	RigidBody;

/* The simulation orchestrator. Owns every body and advances the world one fixed
 * step at a time. The pairs/contacts arrays are scratch space for collision. */
typedef struct World
{
	RigidBody	*bodies;
	int			bodyCount;
	int			bodyCapacity;
	Vec3		gravity;          /* runtime-tweakable (subject requirement) */
	Pair		*pairs;           /* broad-phase output (scratch)            */
	int			pairCount;
	int			pairCapacity;
	Contact		*contacts;        /* narrow-phase output (scratch)           */
	int			contactCount;
	int			contactCapacity;
	int			solverIterations;
}	World;

/* ========================================================================== */
/*  GAME TYPES                                                                 */
/* ========================================================================== */

/* The launcher. Holds the live-tunable launch parameters (mass / speed /
 * direction) the player must be able to control without recompiling. */
typedef struct Catapult
{
	Vec3	position;
	Vec3	launchDirection;
	float	launchSpeed;
	float	projectileMass;
	float	projectileRadius;
}	Catapult;

/* On-screen overlay: mandatory FPS + object counters. Hideable for a clean view. */
typedef struct Hud
{
	int		visible;
	float	fps;
}	Hud;

/* The mandatory debug display: draws colliders as wireframe, toggleable live. */
typedef struct DebugDraw
{
	int	enabled;
}	DebugDraw;

/* Top-level application: wires every module together and runs the main loop. */
typedef struct Game
{
	Window		window;
	World		world;
	Renderer	renderer;
	Camera		camera;
	DebugDraw	debug;
	Hud			hud;
	Catapult	catapult;
	float		fixedDt;
	float		timeScale;   /* runtime control over "time" */
	int			running;
	Mesh		cubeMesh;
	Mesh		sphereMesh;
	Mesh		planeMesh;
}	Game;

/* ========================================================================== */
/*  MATH FUNCTIONS                                                             */
/* ========================================================================== */

/* ---- Vec3 [DONE] ---- */
Vec3	vec3(float x, float y, float z);
Vec3	vec3_add(Vec3 a, Vec3 b);
Vec3	vec3_sub(Vec3 a, Vec3 b);
Vec3	vec3_neg(Vec3 a);
Vec3	vec3_scale(Vec3 a, float s);
float	vec3_dot(Vec3 a, Vec3 b);
Vec3	vec3_cross(Vec3 a, Vec3 b);
float	vec3_length(Vec3 a);
float	vec3_length_sq(Vec3 a);
Vec3	vec3_normalized(Vec3 a);

/* ---- Mat4 [DONE] ---- */
Mat4	mat4_identity(void);
Mat4	mat4_translation(Vec3 t);
Mat4	mat4_scale(Vec3 s);
Mat4	mat4_from_quat(Quat q);
Mat4	mat4_transform(Vec3 pos, Quat rot, Vec3 scale);
Mat4	mat4_perspective(float fovy_radians, float aspect, float near_p, float far_p);
Mat4	mat4_look_at(Vec3 eye, Vec3 target, Vec3 up);
Mat4	mat4_mul(Mat4 a, Mat4 b);

/* ---- Quat ---- */
Quat	quat_identity(void);                              /* [DONE] */
Quat	quat_from_axis_angle(Vec3 axis, float radians);   /* [DONE] */
Quat	quat_mul(Quat a, Quat b);                         /* [DONE] */
Quat	quat_normalized(Quat q);                          /* [DONE] */
Vec3	quat_rotate(Quat q, Vec3 v);                      /* [DONE] */
Quat	quat_integrate(Quat q, Vec3 angular_velocity, float dt); /* [TODO] */

/* ---- Mat3 [TODO] (inertia tensor math: yours to implement) ---- */
Mat3	mat3_identity(void);
Mat3	mat3_diagonal(Vec3 d);
Mat3	mat3_transpose(Mat3 a);
Mat3	mat3_inverse(Mat3 a);
Mat3	mat3_mul(Mat3 a, Mat3 b);
Vec3	mat3_mul_vec3(Mat3 a, Vec3 v);

/* ========================================================================== */
/*  RENDER FUNCTIONS  [DONE]                                                   */
/* ========================================================================== */

/* ---- Window ---- */
int		window_init(Window *win, int width, int height, const char *title);
void	window_destroy(Window *win);
int		window_should_close(const Window *win);
void	window_poll_events(Window *win);
void	window_swap_buffers(Window *win);
float	window_aspect(const Window *win);
struct GLFWwindow	*window_handle(const Window *win);

/* ---- Shader ---- */
int		shader_load(Shader *sh, const char *vert_path, const char *frag_path);
void	shader_use(const Shader *sh);
void	shader_set_mat4(const Shader *sh, const char *name, Mat4 value);
void	shader_set_vec3(const Shader *sh, const char *name, Vec3 value);
void	shader_set_float(const Shader *sh, const char *name, float value);
void	shader_set_int(const Shader *sh, const char *name, int value);

/* ---- Mesh ---- */
Mesh	mesh_cube(void);
Mesh	mesh_sphere(int segments);
Mesh	mesh_plane(float size);
void	mesh_draw(const Mesh *mesh);
void	mesh_release(Mesh *mesh);

/* ---- Camera ---- */
Camera	camera_default(void);
Mat4	camera_view(const Camera *cam);
Mat4	camera_projection(const Camera *cam, float aspect);

/* ---- Renderer ---- */
int		renderer_init(Renderer *r);
void	renderer_begin_frame(Renderer *r, const Camera *cam, float aspect);
void	renderer_draw(Renderer *r, const Mesh *mesh, Mat4 model, Vec3 color);
void	renderer_draw_flat(Renderer *r, const Mesh *mesh, Mat4 model, Vec3 color);
void	renderer_set_wireframe(Renderer *r, int on);

/* ========================================================================== */
/*  PHYSICS FUNCTIONS  [TODO] (the engine you must write)                      */
/* ========================================================================== */

/* ---- RigidBody ---- */
RigidBody	rb_make(void);
void		rb_set_mass(RigidBody *b, float mass);
void		rb_make_static(RigidBody *b);
void		rb_apply_force(RigidBody *b, Vec3 force);
void		rb_apply_force_at_point(RigidBody *b, Vec3 force, Vec3 world_point);
void		rb_apply_impulse(RigidBody *b, Vec3 impulse);
void		rb_apply_impulse_at_point(RigidBody *b, Vec3 impulse, Vec3 world_point);
void		rb_clear_accumulators(RigidBody *b);

/* ---- Integrator ---- */
void		integrator_integrate(RigidBody *b, Vec3 gravity, float dt);

/* ---- World ---- */
void		world_init(World *w);
void		world_destroy(World *w);
int			world_add_body(World *w, RigidBody body);
void		world_remove_body(World *w, int handle);
void		world_clear(World *w);
void		world_step(World *w, float dt);

/* ---- Collision pipeline ---- */
void		broadphase_compute_pairs(World *w);
void		narrowphase_generate_contacts(World *w);
void		resolver_resolve(World *w, float dt);

/* ---- Collider ---- */
Collider	collider_sphere(float radius);
Collider	collider_box(Vec3 half_extents);
Collider	collider_plane(Vec3 normal, float offset);
Mat3		collider_compute_inertia(const Collider *c, float mass);

/* ========================================================================== */
/*  GAME FUNCTIONS  [TODO] (objects, environment, gameplay)                    */
/* ========================================================================== */

/* ---- Catapult ---- */
Catapult	catapult_default(void);
int			catapult_fire(Catapult *c, World *w);

/* ---- Projectile (the "apple") ---- */
RigidBody	projectile_make_apple(Vec3 position, Vec3 velocity, float mass, float radius);

/* ---- Structure (walls / towers / pyramids of boxes) ---- */
void		structure_spawn_wall(World *w, Vec3 origin, int columns, int rows);
void		structure_spawn_pyramid(World *w, Vec3 origin, int base_count);
void		structure_spawn_tower(World *w, Vec3 origin, int height);

/* ---- Hud ---- */
Hud			hud_default(void);
void		hud_update(Hud *h, float frame_time_seconds);
void		hud_draw(const Hud *h, const World *w);

/* ---- DebugDraw ---- */
void		debugdraw_draw_colliders(const DebugDraw *d, const World *w, Renderer *r, const Mesh *cube, const Mesh *sphere, const Mesh *plane);

/* ---- Game ---- */
int			game_init(Game *g);
void		game_run(Game *g);

#endif
