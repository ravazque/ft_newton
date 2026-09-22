
#ifndef NEWTON_H
# define NEWTON_H

# include "defines.h"

# include <stdlib.h>
# include <errno.h>
# include <string.h>
# include <math.h>
# include <stdio.h>
# include <ctype.h>
# include <unistd.h>
# include <stddef.h>
# include <stdarg.h>
# include <time.h>
# include <glad/gl.h>
# include <GLFW/glfw3.h>

/*
 * Naming convention:
 *   - Types are PascalCase:     Vec3, Mat4, RigidBody, Renderer ...
 *   - Functions are snake_case with a module prefix:
 *       vec3_add, mat4_perspective, quat_normalized, mesh_cube,
 *       window_init, renderer_draw, rb_apply_impulse, world_step ...
 *   - Small math types travel by value; stateful objects by pointer.
*/

/* Forward declaration so this header does not drag GLFW/GLAD into every .c.
 * Only the render .c files (and input handling) include the real GLFW headers. */
struct GLFWwindow;

/* ========================================================================== */
/*  MATH TYPES                                                                */
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
/*  RENDER TYPES                                                              */
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

/* Produces the View and Projection matrices. A free-flying eye: it sits at
 * 'position' and looks along (yaw around Y, pitch above the horizon), so it
 * can be walked anywhere in the scene instead of circling a fixed point. */
typedef struct Camera
{
	Vec3	position;
	Vec3	up;
	float	yaw;
	float	pitch;
	float	moveSpeed;    /* m/s while a movement key is held */
	float	fovYDegrees;
	float	nearPlane;
	float	farPlane;
	int		win_width;
	int		win_height;
}	Camera;

/* Batched 2D overlay: every rectangle and glyph pixel of a frame goes into
 * one vertex buffer and is drawn in a single call over the 3D scene. */
typedef struct Ui
{
	unsigned int	vao;
	unsigned int	vbo;
	float			*vertices;    /* CPU side batch, UI_VERTEX_FLOATS per vertex */
	int				vertexCount;
	float			width;        /* window size the frame is laid out for */
	float			height;
}	Ui;

/* Drawing front-end: hides the OpenGL state machine and owns the shader. */
typedef struct Renderer
{
	Shader	shader;
	Mat4	view;
	Mat4	proj;
}	Renderer;

/* ========================================================================== */
/*  COLLISION TYPES                                                           */
/* ========================================================================== */

/* The 3 primitive shapes. A plain enum (no virtual dispatch) keeps an
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

/* One contact point produced by the narrow-phase and consumed by the resolver.
 * The fields after 'penetration' are solver scratch filled every step. */
typedef struct Contact
{
	int		a;
	int		b;
	Vec3	normal;       /* unit vector from A toward B           */
	Vec3	point;        /* contact location in world space       */
	float	penetration;  /* overlap depth along 'normal'          */
	Vec3	t1;           /* friction directions (normal, t1, t2 orthonormal) */
	Vec3	t2;
	float	massNormal;   /* effective mass along the normal       */
	float	massT1;
	float	massT2;
	float	bias;         /* elasticity target separating speed   */
	float	jn;           /* accumulated impulses (normal, tangents) */
	float	jt1;
	float	jt2;
}	Contact;

/* ========================================================================== */
/*  PHYSICS TYPES                                                             */
/* ========================================================================== */

/* What a body was spawned as, so a reloaded definition can find its bodies. */
typedef enum ObjectKind
{
	KIND_BLOCK,
	KIND_APPLE,
	KIND_GROUND,
	KIND_TREBUCHET
}	ObjectKind;

/* The core data the whole engine revolves around: an object that translates and
 * rotates but never deforms. The renderer reads position, orientation and
 * color; the simulation drives everything else. */
typedef struct RigidBody
{
	ObjectKind	kind;
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
	float	elasticity;      /* 0 = no bounce, 1 = perfectly elastic */
	float	friction;         /* Coulomb coefficient, 0 = ice         */
	Vec3	color;            /* render color (RGB 0..1)              */
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
	Vec3		gravity;          /* runtime-tweakable                       */
	Pair		*pairs;           /* broad-phase output (scratch)            */
	int			pairCount;
	int			pairCapacity;
	Contact		*contacts;        /* narrow-phase output (scratch)           */
	int			contactCount;
	int			contactCapacity;
	Contact		*prevContacts;    /* last step's contacts: warm-start source  */
	int			prevContactCount;
	int			prevContactCapacity;
	int			*islandParent;    /* per-body scratch: sleeping islands ...   */
	float		*islandTimer;
	Vec3		*startPositions;  /* ... and displacement/rotation applied by  */
	Vec3		*rotationDelta;   /*     the positional correction this step   */
	int			scratchCapacity;  /* bodies the per-body scratch arrays hold  */
	int			solverIterations;
}	World;

/* ========================================================================== */
/*  DATA FILE TYPES                                                           */
/* ========================================================================== */

/* One "property,value1,value2,value3" row of an object file: up to 3
 * numbers, or one word. */
typedef struct CsvEntry
{
	char	key[CSV_KEY_LEN];
	char	word[CSV_KEY_LEN];
	float	v[3];
	int		count;            /* numeric values stored (0 when 'word' is set) */
	int		line;             /* 1-based line in the file, for error messages */
}	CsvEntry;

typedef struct CsvFile
{
	char		path[CSV_PATH_LEN];
	CsvEntry	entries[CSV_MAX_ENTRIES];
	int			count;
	int			errors;       /* problems reported by the getters and checks */
}	CsvFile;

/* Everything a spawner needs to build one kind of body, read from a .csv. */
typedef struct ObjectDef
{
	ObjectKind	kind;
	ShapeType	shape;
	float		radius;       /* sphere                                  */
	Vec3		size;         /* box: full extents                       */
	Vec3		normal;       /* plane                                   */
	float		offset;       /* plane                                   */
	float		extent;       /* plane: size of the drawn ground square  */
	float		mass;         /* > 0 (planes have none and are static)   */
	float		friction;
	float		elasticity;
	Vec3		color;
}	ObjectDef;

/* ========================================================================== */
/*  GAME TYPES                                                                */
/* ========================================================================== */

/* The launcher: five static boxes (sill, two A-frame legs, throwing arm and
 * counterweight) plus the live-tunable launch parameters (speed / angle /
 * mass) the player controls at runtime. */
typedef struct Trebuchet
{
	Vec3	position;          /* sill center on the ground                   */
	Vec3	baseSize;          /* full extents of the sill box                */
	float	frameHeight;       /* m: the pivot, above the ground              */
	float	armLength;         /* m: the whole arm, both sides of the pivot   */
	float	armThickness;
	float	armAngle;          /* degrees the throwing side sits above +X     */
	Vec3	counterweightSize; /* full extents of the weight on the short side */
	float	launchSpeed;       /* m/s                                         */
	float	launchAngle;       /* degrees above +X                            */
	float	projectileMass;    /* kg                                          */
	float	projectileRadius;  /* m (from the apple definition)               */
	float	friction;
	float	elasticity;
	Vec3	color;
	Vec3	launchPoint;       /* the arm tip, where apples are released      */
	float	spawnDistance;     /* m past the tip, the same for every angle    */
}	Trebuchet;

/* On-screen overlay: FPS, object counter and live values. Hideable for a clean view. */
typedef struct Hud
{
	int		visible;
	float	fps;
	float	refreshTimer;
	int		refresh;          /* 1 => rewrite the title on the next draw */
}	Hud;

/* One line of the menu. The layout fills the rectangles every frame and the
 * input code hit-tests them, so mouse and keyboard drive the same rows. */
typedef enum MenuRowKind
{
	MENU_SECTION,   /* a title, not selectable            */
	MENU_VALUE,     /* a number with [-] and [+] buttons  */
	MENU_ACTION,    /* a button that does something       */
	MENU_HINT       /* a line of the controls panel       */
}	MenuRowKind;

typedef enum MenuAction
{
	ACTION_NONE,
	ACTION_APPLY,
	ACTION_RELOAD,
	ACTION_SAVE,
	ACTION_RESET,
	ACTION_RESUME,
	ACTION_QUIT
}	MenuAction;

typedef struct MenuRow
{
	MenuRowKind	kind;
	const char	*label;
	float		*value;      /* MENU_VALUE: what [-] and [+] change */
	float		min;
	float		max;
	float		step;
	int			decimals;    /* digits shown after the point */
	MenuAction	action;
	int			column;      /* 0 = left (values), 1 = right (controls, actions) */
	float		x;           /* layout, recomputed every frame */
	float		y;
	float		w;
	float		h;
	float		minusX;      /* left edge of the [-] and [+] boxes */
	float		plusX;
	float		buttonW;
}	MenuRow;

/* The pause menu: every tunable value, the controls reference and the
 * actions. Opening it pauses the simulation. */
typedef struct Menu
{
	int			open;
	MenuRow		rows[MENU_MAX_ROWS];
	int			rowCount;
	int			selected;      /* keyboard cursor (a selectable row)  */
	int			hoverRow;      /* row under the mouse, -1 when none   */
	int			hoverPart;     /* -1 none, 0 minus, 1 plus, 2 button  */
	int			heldRow;       /* row whose button is being held      */
	int			heldPart;
	float		repeatTimer;
	int			valueChanged;  /* set when a value moved this frame   */
	int			dirty;         /* edits are waiting for Apply         */
	MenuAction	pending;       /* action clicked this frame           */
	float		panelX;        /* geometry, recomputed by menu_layout */
	float		panelY;
	float		panelW;
	float		panelH;
	float		scale;         /* font pixel size (a whole number)    */
	float		line;          /* height of one row                   */
	float		pad;
}	Menu;

/* The debug display: draws colliders as wireframe, toggleable live. */
typedef struct DebugDraw
{
	int	enabled;
}	DebugDraw;

/* What the menu edits. It is a copy, never the running simulation: the rows
 * move these numbers and nothing happens until Apply pushes them across (or
 * Save writes them to the files). That way a value can be dialled in without
 * the scene reacting halfway through. */
typedef struct Draft
{
	ObjectDef	apple;
	ObjectDef	block;
	ObjectDef	ground;
	Trebuchet	trebuchet;
	float		gravityY;
	float		timeScale;
}	Draft;

/* Top-level application: wires every module together and runs the main loop. */
typedef struct Game
{
	Window		window;
	World		world;
	Renderer	renderer;
	Ui			ui;
	Menu		menu;
	Camera		camera;
	DebugDraw	debug;
	Hud			hud;
	Trebuchet	trebuchet;     /* the live one: launch settings change with the keys */
	Trebuchet	trebuchetDef;  /* what a reset goes back to                        */
	ObjectDef	appleDef;
	ObjectDef	blockDef;
	ObjectDef	groundDef;
	Draft		draft;         /* the menu's working copy, committed by Apply      */
	float		fixedDt;
	float		timeScale;   /* runtime control over "time" */
	int			paused;      /* set by P, or while the menu is open */
	int			quit;        /* the menu's Quit button */
	Mesh		cubeMesh;
	Mesh		sphereMesh;
	Mesh		planeMesh;
}	Game;

/* ========================================================================== */
/*  MATH FUNCTIONS                                                            */
/* ========================================================================== */

/* ---- Vec3 ---- */
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

/* ---- Mat4 ---- */
Mat4	mat4_identity(void);
Mat4	mat4_translation(Vec3 t);
Mat4	mat4_scale(Vec3 s);
Mat4	mat4_from_quat(Quat q);
Mat4	mat4_transform(Vec3 pos, Quat rot, Vec3 scale);
Mat4	mat4_perspective(float fovy_radians, float aspect, float near_p, float far_p);
Mat4	mat4_look_at(Vec3 eye, Vec3 target, Vec3 up);
Mat4	mat4_mul(Mat4 a, Mat4 b);

/* ---- Quat ---- */
Quat	quat_identity(void);
Quat	quat_from_axis_angle(Vec3 axis, float radians);
Quat	quat_from_to(Vec3 from, Vec3 to);
Quat	quat_mul(Quat a, Quat b);
Quat	quat_normalized(Quat q);
Vec3	quat_rotate(Quat q, Vec3 v);
Quat	quat_integrate(Quat q, Vec3 angular_velocity, float dt);

/* ---- Mat3 ---- */
Mat3	mat3_identity(void);
Mat3	mat3_zero(void);
Mat3	mat3_diagonal(Vec3 d);
Mat3	mat3_from_quat(Quat q);
Mat3	mat3_transpose(Mat3 a);
Mat3	mat3_inverse(Mat3 a);
Mat3	mat3_mul(Mat3 a, Mat3 b);
Vec3	mat3_mul_vec3(Mat3 a, Vec3 v);

/* ========================================================================== */
/*  RENDER FUNCTIONS                                                          */
/* ========================================================================== */

/* ---- Window ---- */
int		window_init(Window *win, int width, int height, const char *title);
void	window_destroy(Window *win);
int		window_should_close(const Window *win);
void	window_poll_events(Window *win);
void	window_swap_buffers(Window *win);
float	window_aspect(const Window *win);
void	window_size(const Window *win, float *width, float *height);
void	window_cursor(const Window *win, float *x, float *y);
int		window_mouse_down(const Window *win);
int		window_key_down(const Window *win, int key);
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
void	camera_look(Camera *cam, float delta_yaw, float delta_pitch);
void	camera_move(Camera *cam, Vec3 local_delta);
void	camera_change_speed(Camera *cam, float delta);
Mat4	camera_view(const Camera *cam);
Mat4	camera_projection(const Camera *cam, float aspect);

/* ---- Bitmap font ---- */
const unsigned char	*font_glyph(char c);

/* ---- 2D overlay ---- */
int		ui_init(Ui *ui);
void	ui_destroy(Ui *ui);
void	ui_begin(Ui *ui, float width, float height);
void	ui_rect(Ui *ui, float x, float y, float w, float h, Vec3 color);
void	ui_border(Ui *ui, float x, float y, float w, float h, float t, Vec3 color);
void	ui_text(Ui *ui, const char *text, float x, float y, float scale, Vec3 color);
void	ui_text_right(Ui *ui, const char *text, float right, float y, float scale, Vec3 color);
float	ui_text_width(const char *text, float scale);
void	ui_end(Ui *ui, Renderer *r);

/* ---- Menu ---- */
void	menu_add_section(Menu *m, const char *label, int column);
void	menu_add_value(Menu *m, const char *label, float *value, float min_value, float max_value, float step, int decimals);
void	menu_add_action(Menu *m, const char *label, MenuAction action, int column);
void	menu_add_hint(Menu *m, const char *label);
void	menu_layout(Menu *m, float width, float height);
void	menu_update(Menu *m, Window *win, float frame_time);
void	menu_draw(const Menu *m, Ui *ui);

/* ---- Renderer ---- */
int		renderer_init(Renderer *r);
void	renderer_begin_frame(Renderer *r, const Camera *cam, float aspect);
void	renderer_draw(Renderer *r, const Mesh *mesh, Mat4 model, Vec3 color);
void	renderer_draw_flat(Renderer *r, const Mesh *mesh, Mat4 model, Vec3 color);
void	renderer_set_wireframe(Renderer *r, int on);

/* ========================================================================== */
/*  PHYSICS FUNCTIONS                                                         */
/* ========================================================================== */

/* ---- RigidBody ---- */
RigidBody	rb_make(void);
void		rb_set_mass(RigidBody *b, float mass);
void		rb_make_static(RigidBody *b);
void		rb_update_inertia_world(RigidBody *b);
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
void		world_wake_all(World *w);
int			world_grow_scratch(World *w);
void		world_step(World *w, float dt);

/* ---- Collision pipeline ----
 * world_step runs broadphase (candidate pairs) -> narrowphase (contacts with
 * normal / point / penetration) -> resolver (impulses + positional correction). */
void		broadphase_compute_pairs(World *w);
void		narrowphase_generate_contacts(World *w);
void		resolver_resolve(World *w);

/* ---- Per-pair contact generators (used by the narrow-phase) ----
 * Each writes up to MAX_CONTACTS_PER_PAIR contacts into 'out' (indices a/b
 * left unset) and returns how many it found. The normal always points from
 * the FIRST argument's body toward the SECOND's. */
int			contact_sphere_sphere(const RigidBody *a, const RigidBody *b, Contact *out);
int			contact_sphere_plane(const RigidBody *sphere, const RigidBody *plane, Contact *out);
int			contact_box_plane(const RigidBody *box, const RigidBody *plane, Contact *out);
int			contact_sphere_box(const RigidBody *sphere, const RigidBody *box, Contact *out);
int			contact_box_box(const RigidBody *a, const RigidBody *b, Contact *out);

/* ---- Contact query: "are these two bodies touching?" -> 1 / 0 ---- */
int			bodies_in_contact(const RigidBody *a, const RigidBody *b);

/* ---- Collider ---- */
Collider	collider_sphere(float radius);
Collider	collider_box(Vec3 half_extents);
Collider	collider_plane(Vec3 normal, float offset);
Mat3		collider_compute_inertia(const Collider *c, float mass);
Vec3		collider_plane_origin(const Collider *c);
Quat		collider_plane_rotation(const Collider *c);

/* ========================================================================== */
/*  DATA FILE FUNCTIONS                                                       */
/* ========================================================================== */

/* ---- CsvFile: strict "property,value1,value2,value3" object files ---- */
int			csv_load(CsvFile *c, const char *path);
int			csv_reject_unknown(CsvFile *c, const char **allowed, int allowed_count);
float		csv_get_float(CsvFile *c, const char *key);
Vec3		csv_get_vec3(CsvFile *c, const char *key);
const char	*csv_get_word(CsvFile *c, const char *key);
void		csv_error(CsvFile *c, const char *key, const char *what);
int			csv_expect_positive(CsvFile *c, const char *key, float v);
int			csv_expect_min(CsvFile *c, const char *key, float v, float lo);
int			csv_expect_range(CsvFile *c, const char *key, float v, float lo, float hi);
int			csv_expect_positive_vec3(CsvFile *c, const char *key, Vec3 v);
int			csv_expect_range_vec3(CsvFile *c, const char *key, Vec3 v, float lo, float hi);

/* ---- ObjectDef ---- */
int			objectdef_load(ObjectDef *def, const char *path, ObjectKind kind);
RigidBody	objectdef_make_body(const ObjectDef *def, Vec3 position);
int			objectdef_save(const ObjectDef *def, const char *path);

/* ========================================================================== */
/*  GAME FUNCTIONS                                                            */
/* ========================================================================== */

/* ---- Trebuchet ---- */
int			trebuchet_load(Trebuchet *c, const char *path, const ObjectDef *apple);
int			trebuchet_save(const Trebuchet *c, const char *path);
void		trebuchet_build(Trebuchet *c, World *w);
Vec3		trebuchet_direction(const Trebuchet *c);
Vec3		trebuchet_spawn_point(const Trebuchet *c);
int			trebuchet_fire(const Trebuchet *c, const ObjectDef *apple, World *w);

/* ---- Projectile (the apple) ---- */
RigidBody	projectile_make_apple(const ObjectDef *apple, Vec3 position, Vec3 velocity, float mass);

/* ---- Structure (walls / towers / pyramids of blocks) ---- */
void		structure_spawn_wall(World *w, const ObjectDef *block, Vec3 origin, int columns, int rows);
void		structure_spawn_pyramid(World *w, const ObjectDef *block, Vec3 origin, int base_count);
void		structure_spawn_tower(World *w, const ObjectDef *block, Vec3 origin, int height);

/* ---- Hud ---- */
Hud			hud_default(void);
void		hud_update(Hud *h, float frame_time_seconds);
void		hud_draw(Hud *h, const World *w, const Trebuchet *c, float time_scale, Window *win, int paused);

/* ---- DebugDraw ---- */
void		debugdraw_draw_colliders(const DebugDraw *d, const World *w, Renderer *r, const Mesh *cube, const Mesh *sphere, const Mesh *plane);

/* ---- Game: start-up, the frame and the loop (srcs/game/game.c) ---- */
int			game_init(Game *g, int argc, char **argv);
void		game_run(Game *g);
void		game_shutdown(Game *g);

/* ---- Scene: what is in the world, and what the menu does to it (scene.c) ---- */
int			scene_load_assets(Game *g);
void		scene_build(Game *g);
void		scene_build_menu(Game *g);
void		scene_sync_draft(Game *g);
void		scene_apply_definitions(Game *g);
void		scene_run_action(Game *g, MenuAction action);

/* ---- Input: every key the game reads (srcs/game/input.c) ---- */
void		input_poll(Game *g, float frame_time);

/* ---- Utils ---- */
void		check_input(int argc, char **argv, Camera *camera);

#endif
