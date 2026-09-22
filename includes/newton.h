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
 * The only header of the project: every type, then every prototype grouped by
 * the source file that defines it (srcs/<folder>/<file>.c), in pipeline order:
 * math -> physics -> collision -> response -> render -> menu -> data -> game.
*/

/* ========================================================================== */
/*  MATH TYPES                                                                */
/* ========================================================================== */

typedef struct Vec3
{
	float x;
	float y;
	float z;
}	Vec3;

/* Column-major: element (col c, row r) lives at m[c * 3 + r]. */
typedef struct Mat3
{
	float m[9];
}	Mat3;

/* Column-major, the layout OpenGL uploads without a transpose. */
typedef struct Mat4
{
	float m[16];
}	Mat4;

/* Unit quaternion (w, x, y, z): an orientation without gimbal lock. */
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

typedef struct Window
{
	struct GLFWwindow	*handle;
	int					width;
	int					height;
}	Window;

typedef struct Shader
{
	unsigned int	program;
}	Shader;

/* Geometry in GPU memory: vbo = vertices (position + normal), ebo = indices,
 * vao = how the vertex shader reads the vbo. */
typedef struct Mesh
{
	unsigned int	vao;
	unsigned int	vbo;
	unsigned int	ebo;
	int				indexCount;
}	Mesh;

/* Free-flying eye: a position looking along (yaw, pitch). */
typedef struct Camera
{
	Vec3	position;
	Vec3	up;
	float	yaw;
	float	pitch;
	float	moveSpeed;    /* m/s */
	float	fovYDegrees;
	float	nearPlane;
	float	farPlane;
	int		win_width;
	int		win_height;
}	Camera;

/* Batched 2D overlay: one vertex buffer, one draw call per frame. */
typedef struct Ui
{
	unsigned int	vao;
	unsigned int	vbo;
	float			*vertices;    /* UI_VERTEX_FLOATS per vertex */
	int				vertexCount;
	float			width;        /* window size the frame is laid out for */
	float			height;
}	Ui;

typedef struct Renderer
{
	Shader	shader;
	Mat4	view;
	Mat4	proj;
}	Renderer;

/* ========================================================================== */
/*  COLLISION TYPES                                                           */
/* ========================================================================== */

typedef enum ShapeType
{
	SHAPE_SPHERE,
	SHAPE_BOX,
	SHAPE_PLANE
}	ShapeType;

/* Only the fields of 'type' are read. A plane is normal . x = offset, limited
 * to a square of side 2 * halfSize centred on normal * offset. */
typedef struct Collider
{
	ShapeType	type;
	float		radius;       /* sphere */
	Vec3		halfExtents;  /* box */
	Vec3		normal;       /* plane */
	float		offset;       /* plane */
	float		halfSize;     /* plane */
}	Collider;

/* A broad-phase candidate: two body indices, a < b. */
typedef struct Pair
{
	int	a;
	int	b;
}	Pair;

/* One contact point. The narrow-phase fills the first five fields, the
 * solver the rest every step. */
typedef struct Contact
{
	int		a;
	int		b;
	Vec3	normal;       /* unit, from a toward b */
	Vec3	point;        /* world space */
	float	penetration;  /* overlap depth along the normal */
	Vec3	t1;           /* friction directions: (normal, t1, t2) orthonormal */
	Vec3	t2;
	float	massNormal;   /* effective masses along normal, t1, t2 */
	float	massT1;
	float	massT2;
	float	massR1;       /* effective angular masses about t1, t2 (rolling) and the normal (spinning) */
	float	massR2;
	float	massRn;
	float	bias;         /* elasticity target separating speed */
	float	rolling;      /* rolling resistance arm of the sphere: coefficient * radius (m) */
	float	jn;           /* accumulated impulses: normal, friction, rolling, spinning */
	float	jt1;
	float	jt2;
	float	jr1;
	float	jr2;
	float	jrn;
}	Contact;

/* An oriented box: center, the three world axes and the half extents. */
typedef struct Obb
{
	Vec3	c;
	Vec3	ax[3];
	float	h[3];
}	Obb;

/* Axis of least penetration found by the separating axis test. */
typedef struct SatResult
{
	float	pen;
	Vec3	n;          /* unit, from A toward B */
	int		faceOwner;  /* 0 = face of A, 1 = face of B, 2 = edge against edge */
	int		faceIdx;
	int		edgeA;
	int		edgeB;
}	SatResult;

/* ========================================================================== */
/*  PHYSICS TYPES                                                             */
/* ========================================================================== */

/* What a body was spawned as, so a reloaded definition finds its bodies. */
typedef enum ObjectKind
{
	KIND_BLOCK,
	KIND_APPLE,
	KIND_GROUND,
	KIND_TREBUCHET
}	ObjectKind;

/* An object that translates and rotates but never deforms. invMass == 0
 * marks a static body (ground, trebuchet). */
typedef struct RigidBody
{
	ObjectKind	kind;
	Vec3		position;
	Vec3		velocity;
	Vec3		forceAccum;
	float		mass;
	float		invMass;
	Quat		orientation;
	Vec3		angularVelocity;
	Vec3		torqueAccum;
	Mat3		invInertiaLocal;    /* body space, constant */
	Mat3		invInertiaWorld;    /* R * invInertiaLocal * R^T */
	float		elasticity;         /* 0 = no bounce, 1 = perfectly elastic */
	float		friction;           /* Coulomb coefficient */
	float		rollingResistance;  /* spheres: resisting torque / (normal force * radius) */
	Vec3		color;
	Collider	collider;
	int			awake;
	float		sleepTimer;         /* s spent below the sleep thresholds */
}	RigidBody;

/* Every body plus the per-step scratch of the collision pipeline. */
typedef struct World
{
	RigidBody	*bodies;
	int			bodyCount;
	int			bodyCapacity;
	Vec3		gravity;
	Pair		*pairs;             /* broad-phase output */
	int			pairCount;
	int			pairCapacity;
	Contact		*contacts;          /* narrow-phase output */
	int			contactCount;
	int			contactCapacity;
	Contact		*prevContacts;      /* last step's contacts, for warm starting */
	int			prevContactCount;
	int			prevContactCapacity;
	int			*islandParent;      /* per-body scratch: sleep islands ... */
	float		*islandTimer;
	Vec3		*startPositions;    /* ... and what the positional correction moved */
	Vec3		*rotationDelta;
	int			scratchCapacity;
	int			solverIterations;
}	World;

/* ========================================================================== */
/*  DATA FILE TYPES                                                           */
/* ========================================================================== */

/* One "property,value1,value2,value3" row: up to 3 numbers, or one word. */
typedef struct CsvEntry
{
	char	key[CSV_KEY_LEN];
	char	word[CSV_KEY_LEN];
	float	v[3];
	int		count;  /* numbers stored (0 when 'word' is set) */
	int		line;   /* 1-based, for error messages */
}	CsvEntry;

typedef struct CsvFile
{
	char		path[CSV_PATH_LEN];
	CsvEntry	entries[CSV_MAX_ENTRIES];
	int			count;
	int			errors;  /* problems reported by the getters and checks */
}	CsvFile;

/* Everything needed to build one kind of body, read from its .csv. */
typedef struct ObjectDef
{
	ObjectKind	kind;
	ShapeType	shape;
	float		radius;       /* sphere */
	Vec3		size;         /* box: full extents */
	Vec3		normal;       /* plane */
	float		offset;       /* plane */
	float		extent;       /* plane: side of the ground square */
	float		mass;         /* planes have none: they are static */
	float		friction;
	float		elasticity;
	float		rollingResistance;  /* sphere */
	Vec3		color;
}	ObjectDef;

/* ========================================================================== */
/*  GAME TYPES                                                                */
/* ========================================================================== */

/* The launcher: five static boxes plus the live launch parameters. */
typedef struct Trebuchet
{
	Vec3	position;           /* sill center on the ground */
	Vec3	baseSize;           /* full extents of the sill */
	float	frameHeight;        /* m: height of the pivot */
	float	armLength;          /* m: both sides of the pivot */
	float	armThickness;
	float	armAngle;           /* deg of the throwing side above +X */
	Vec3	counterweightSize;
	float	launchSpeed;        /* m/s */
	float	launchAngle;        /* deg above +X */
	float	projectileMass;     /* kg of the next apple */
	float	projectileRadius;   /* m */
	float	friction;
	float	elasticity;
	Vec3	color;
	Vec3	launchPoint;        /* the arm tip */
	float	spawnDistance;      /* m past the tip where apples appear */
}	Trebuchet;

/* FPS, object counter and live values, written to the window title. */
typedef struct Hud
{
	int		visible;
	float	fps;
	float	refreshTimer;
	int		refresh;  /* rewrite the title on the next draw */
}	Hud;

typedef enum MenuRowKind
{
	MENU_SECTION,
	MENU_VALUE,
	MENU_ACTION,
	MENU_HINT
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

/* One line of the menu. The layout writes its rectangles every frame and the
 * input hit-tests the same ones. */
typedef struct MenuRow
{
	MenuRowKind	kind;
	const char	*label;
	float		*value;     /* MENU_VALUE: what [-] and [+] change */
	float		min;
	float		max;
	float		step;
	int			decimals;
	MenuAction	action;
	int			column;     /* 0 = values, 1 = controls and actions */
	float		x;
	float		y;
	float		w;
	float		h;
	float		minusX;
	float		plusX;
	float		buttonW;
}	MenuRow;

typedef struct Menu
{
	int			open;
	MenuRow		rows[MENU_MAX_ROWS];
	int			rowCount;
	int			selected;      /* keyboard cursor */
	int			hoverRow;      /* row under the mouse, -1 when none */
	int			hoverPart;     /* -1 none, 0 minus, 1 plus, 2 button */
	int			heldRow;
	int			heldPart;
	float		repeatTimer;
	int			valueChanged;  /* a value moved this frame */
	int			dirty;         /* edits wait for Apply */
	MenuAction	pending;       /* action clicked this frame */
	float		panelX;
	float		panelY;
	float		panelW;
	float		panelH;
	float		scale;         /* screen pixels per font pixel */
	float		line;          /* height of one row */
	float		pad;
}	Menu;

typedef struct DebugDraw
{
	int	enabled;
}	DebugDraw;

/* What the menu edits: a copy of the simulation values, committed by Apply. */
typedef struct Draft
{
	ObjectDef	apple;
	ObjectDef	block;
	ObjectDef	ground;
	Trebuchet	trebuchet;
	float		gravityY;
	float		timeScale;
}	Draft;

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
	Trebuchet	trebuchet;     /* live: the keys change its launch settings */
	Trebuchet	trebuchetDef;  /* what a reset returns to */
	ObjectDef	appleDef;
	ObjectDef	blockDef;
	ObjectDef	groundDef;
	Draft		draft;
	float		fixedDt;
	float		timeScale;
	int			paused;
	int			quit;
	Mesh		cubeMesh;
	Mesh		sphereMesh;
	Mesh		planeMesh;
}	Game;

/* ========================================================================== */
/*  srcs/math                                                                 */
/* ========================================================================== */

/* ---- vec3.c ---- */
Vec3		vec3(float x, float y, float z);
Vec3		vec3_add(Vec3 a, Vec3 b);
Vec3		vec3_sub(Vec3 a, Vec3 b);
Vec3		vec3_neg(Vec3 a);
Vec3		vec3_scale(Vec3 a, float s);
float		vec3_dot(Vec3 a, Vec3 b);
Vec3		vec3_cross(Vec3 a, Vec3 b);
float		vec3_length(Vec3 a);
float		vec3_length_sq(Vec3 a);
Vec3		vec3_normalized(Vec3 a);

/* ---- scalar.c ---- */
float		clampf(float v, float lo, float hi);

/* ---- mat3.c ---- */
Mat3		mat3_identity(void);
Mat3		mat3_zero(void);
Mat3		mat3_diagonal(Vec3 d);
Mat3		mat3_from_quat(Quat q);
Mat3		mat3_transpose(Mat3 a);
Mat3		mat3_inverse(Mat3 a);
Mat3		mat3_mul(Mat3 a, Mat3 b);
Vec3		mat3_mul_vec3(Mat3 a, Vec3 v);

/* ---- mat4.c ---- */
Mat4		mat4_identity(void);
Mat4		mat4_translation(Vec3 t);
Mat4		mat4_scale(Vec3 s);
Mat4		mat4_from_quat(Quat q);
Mat4		mat4_transform(Vec3 pos, Quat rot, Vec3 scale);
Mat4		mat4_perspective(float fovy_radians, float aspect, float near_p, float far_p);
Mat4		mat4_look_at(Vec3 eye, Vec3 target, Vec3 up);
Mat4		mat4_mul(Mat4 a, Mat4 b);

/* ---- quat.c ---- */
Quat		quat_identity(void);
Quat		quat_from_axis_angle(Vec3 axis, float radians);
Quat		quat_from_to(Vec3 from, Vec3 to);
Quat		quat_mul(Quat a, Quat b);
Quat		quat_normalized(Quat q);
Quat		quat_conjugate(Quat q);
Vec3		quat_rotate(Quat q, Vec3 v);
Quat		quat_integrate(Quat q, Vec3 angular_velocity, float dt);

/* ========================================================================== */
/*  srcs/physics                                                              */
/* ========================================================================== */

/* ---- rigidbody.c ---- */
RigidBody	rb_make(void);
void		rb_set_mass(RigidBody *b, float mass);
void		rb_make_static(RigidBody *b);
void		rb_apply_force(RigidBody *b, Vec3 force);
void		rb_apply_force_at_point(RigidBody *b, Vec3 force, Vec3 world_point);
void		rb_apply_impulse(RigidBody *b, Vec3 impulse);
void		rb_apply_impulse_at_point(RigidBody *b, Vec3 impulse, Vec3 world_point);
void		rb_apply_angular_impulse(RigidBody *b, Vec3 angular_impulse);
void		rb_clear_accumulators(RigidBody *b);

/* ---- inertia.c ---- */
Mat3		inertia_local_inverse(const Collider *c, float mass);
void		inertia_update_world(RigidBody *b);

/* ---- integrator.c ---- */
void		integrator_integrate(RigidBody *b, Vec3 gravity, float dt);

/* ---- world.c ---- */
void		world_init(World *w);
void		world_destroy(World *w);
int			world_add_body(World *w, RigidBody body);
void		world_remove_body(World *w, int handle);
void		world_clear(World *w);
void		world_wake_all(World *w);
int			world_grow_scratch(World *w);
void		world_step(World *w, float dt);

/* ---- sleep.c ---- */
void		sleep_update(World *w, float dt);

/* ---- cull.c ---- */
void		cull_lost_bodies(World *w);

/* ========================================================================== */
/*  srcs/collision                                                            */
/* ========================================================================== */

/* ---- collider.c ---- */
Collider	collider_sphere(float radius);
Collider	collider_box(Vec3 half_extents);
Collider	collider_plane(Vec3 normal, float offset, float size);
Vec3		collider_plane_origin(const Collider *c);
Quat		collider_plane_rotation(const Collider *c);
float		collider_plane_distance(const Collider *c, Vec3 point);
int			collider_plane_covers(const Collider *c, Vec3 point);
void		collider_bounds(const RigidBody *b, Vec3 *mn, Vec3 *mx);

/* ---- broadphase.c ---- */
void		broadphase_compute_pairs(World *w);

/* ---- narrowphase.c ---- */
void		narrowphase_generate_contacts(World *w);
int			narrowphase_pair(const RigidBody *a, const RigidBody *b, Contact *out);

/* ---- query.c ---- */
int			bodies_in_contact(const RigidBody *a, const RigidBody *b);
int			bodies_overlap(const RigidBody *a, const RigidBody *b);
int			world_first_overlap(const World *w, const RigidBody *candidate);

/* ---- contact_sphere.c / contact_box.c: each writes up to
 * MAX_CONTACTS_PER_PAIR contacts, normal from the FIRST body toward the
 * SECOND, and returns how many ---- */
int			contact_sphere_sphere(const RigidBody *a, const RigidBody *b, Contact *out);
int			contact_sphere_plane(const RigidBody *sphere, const RigidBody *plane, Contact *out);
int			contact_sphere_box(const RigidBody *sphere, const RigidBody *box, Contact *out);
int			contact_box_plane(const RigidBody *box, const RigidBody *plane, Contact *out);
int			contact_box_box(const RigidBody *a, const RigidBody *b, Contact *out);

/* ---- obb.c ---- */
Obb			obb_from_body(const RigidBody *b);
float		obb_radius(const Obb *o, Vec3 n);
Vec3		obb_corner(const Obb *o, int i);
void		obb_support_edge(const Obb *o, int dir_idx, Vec3 n, Vec3 *p0, Vec3 *p1);

/* ---- sat.c ---- */
int			sat_boxes(const Obb *a, const Obb *b, SatResult *res);

/* ---- manifold.c ---- */
int			manifold_reduce(const Contact *cand, int n, Vec3 axis, Contact *out);
int			manifold_clip(Vec3 *poly, int count, Vec3 n, float off);

/* ========================================================================== */
/*  srcs/response                                                             */
/* ========================================================================== */

/* ---- resolver.c ---- */
void		resolver_resolve(World *w);

/* ---- impulse.c ---- */
void		impulse_prepare(const World *w, Contact *c);
void		impulse_apply(RigidBody *a, RigidBody *b, const Contact *c, Vec3 impulse);
void		impulse_solve(World *w, Contact *c);

/* ---- correction.c ---- */
void		correction_apply(World *w);

/* ========================================================================== */
/*  srcs/render                                                               */
/* ========================================================================== */

/* ---- window.c ---- */
int			window_init(Window *win, int width, int height, const char *title);
void		window_destroy(Window *win);
int			window_should_close(const Window *win);
void		window_poll_events(Window *win);
void		window_swap_buffers(Window *win);
float		window_aspect(const Window *win);
void		window_size(const Window *win, float *width, float *height);
void		window_cursor(const Window *win, float *x, float *y);
int			window_mouse_down(const Window *win);
int			window_key_down(const Window *win, int key);
struct GLFWwindow	*window_handle(const Window *win);

/* ---- shader.c ---- */
int			shader_load(Shader *sh, const char *vert_path, const char *frag_path);
void		shader_use(const Shader *sh);
void		shader_set_mat4(const Shader *sh, const char *name, Mat4 value);
void		shader_set_vec3(const Shader *sh, const char *name, Vec3 value);
void		shader_set_float(const Shader *sh, const char *name, float value);
void		shader_set_int(const Shader *sh, const char *name, int value);

/* ---- mesh.c ---- */
Mesh		mesh_cube(void);
Mesh		mesh_sphere(int segments);
Mesh		mesh_plane(void);
void		mesh_draw(const Mesh *mesh);
void		mesh_release(Mesh *mesh);

/* ---- camera.c ---- */
Camera		camera_default(void);
void		camera_look(Camera *cam, float delta_yaw, float delta_pitch);
void		camera_move(Camera *cam, Vec3 local_delta);
void		camera_change_speed(Camera *cam, float delta);
Mat4		camera_view(const Camera *cam);
Mat4		camera_projection(const Camera *cam, float aspect);

/* ---- renderer.c ---- */
int			renderer_init(Renderer *r);
void		renderer_begin_frame(Renderer *r, const Camera *cam, float aspect);
void		renderer_draw(Renderer *r, const Mesh *mesh, Mat4 model, Vec3 color);
void		renderer_draw_flat(Renderer *r, const Mesh *mesh, Mat4 model, Vec3 color);
void		renderer_set_wireframe(Renderer *r, int on);
Mat4		renderer_body_model(const RigidBody *b, float inflate);
const Mesh	*renderer_body_mesh(const RigidBody *b, const Mesh *cube, const Mesh *sphere, const Mesh *plane);

/* ---- debugdraw.c ---- */
void		debugdraw_draw_colliders(const DebugDraw *d, const World *w, Renderer *r, const Mesh *cube, const Mesh *sphere, const Mesh *plane);

/* ---- font.c ---- */
const unsigned char	*font_glyph(char c);

/* ---- ui.c ---- */
int			ui_init(Ui *ui);
void		ui_destroy(Ui *ui);
void		ui_begin(Ui *ui, float width, float height);
void		ui_rect(Ui *ui, float x, float y, float w, float h, Vec3 color);
void		ui_border(Ui *ui, float x, float y, float w, float h, float t, Vec3 color);
void		ui_text(Ui *ui, const char *text, float x, float y, float scale, Vec3 color);
void		ui_text_right(Ui *ui, const char *text, float right, float y, float scale, Vec3 color);
float		ui_text_width(const char *text, float scale);
void		ui_end(Ui *ui, Renderer *r);

/* ========================================================================== */
/*  srcs/menu                                                                 */
/* ========================================================================== */

/* ---- menu.c ---- */
void		menu_add_section(Menu *m, const char *label, int column);
void		menu_add_value(Menu *m, const char *label, float *value, float min_value, float max_value, float step, int decimals);
void		menu_add_action(Menu *m, const char *label, MenuAction action, int column);
void		menu_add_hint(Menu *m, const char *label);
int			menu_is_selectable(const MenuRow *row);
void		menu_step_value(Menu *m, MenuRow *row, int direction);

/* ---- menu_layout.c ---- */
void		menu_layout(Menu *m, float width, float height);

/* ---- menu_input.c ---- */
void		menu_update(Menu *m, Window *win, float frame_time);

/* ---- menu_draw.c ---- */
void		menu_draw(const Menu *m, Ui *ui);

/* ========================================================================== */
/*  srcs/data                                                                 */
/* ========================================================================== */

/* ---- csvfile.c ---- */
int			csv_load(CsvFile *c, const char *path);
const CsvEntry	*csv_find(const CsvFile *c, const char *key);

/* ---- csv_values.c ---- */
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

/* ---- objectdef.c ---- */
int			objectdef_load(ObjectDef *def, const char *path, ObjectKind kind);
RigidBody	objectdef_make_body(const ObjectDef *def, Vec3 position);
int			objectdef_save(const ObjectDef *def, const char *path);

/* ---- trebuchet_file.c ---- */
int			trebuchet_load(Trebuchet *t, const char *path, const ObjectDef *apple);
int			trebuchet_save(const Trebuchet *t, const char *path);

/* ========================================================================== */
/*  srcs/game                                                                 */
/* ========================================================================== */

/* ---- game.c ---- */
int			game_init(Game *g, int argc, char **argv);
void		game_run(Game *g);
void		game_shutdown(Game *g);

/* ---- draw.c ---- */
void		draw_frame(Game *g);

/* ---- input.c ---- */
void		input_poll(Game *g, float frame_time);

/* ---- hud.c ---- */
Hud			hud_default(void);
void		hud_update(Hud *h, float frame_time_seconds);
void		hud_draw(Hud *h, const World *w, const Trebuchet *t, float time_scale, Window *win, int paused);

/* ---- scene.c ---- */
int			scene_load_assets(Game *g);
int			scene_check_start(const ObjectDef *ground, const ObjectDef *block, const Trebuchet *trebuchet);
void		scene_build(Game *g);
void		scene_apply_definitions(Game *g);

/* ---- actions.c ---- */
void		action_sync_draft(Game *g);
void		action_run(Game *g, MenuAction action);

/* ---- menu_rows.c ---- */
void		menu_rows_build(Game *g);

/* ---- trebuchet.c ---- */
void		trebuchet_parts(const Trebuchet *t, Vec3 *center, Vec3 *half, Quat *rotation);
Vec3		trebuchet_launch_point(const Trebuchet *t);
Vec3		trebuchet_direction(const Trebuchet *t);
Vec3		trebuchet_spawn_point(const Trebuchet *t);
void		trebuchet_build(Trebuchet *t, World *w);
int			trebuchet_fire(const Trebuchet *t, const ObjectDef *apple, World *w);

/* ---- trebuchet_clearance.c ---- */
float		trebuchet_spawn_distance(const Trebuchet *t);

/* ---- projectile.c ---- */
RigidBody	projectile_make_apple(const ObjectDef *apple, Vec3 position, Vec3 velocity, float mass);

/* ---- structure.c ---- */
float		structure_spawn_wall(World *w, const ObjectDef *block, Vec3 origin, int columns, int rows);
float		structure_spawn_pyramid(World *w, const ObjectDef *block, Vec3 origin, int base_count);
float		structure_spawn_tower(World *w, const ObjectDef *block, Vec3 origin, int height);

/* ========================================================================== */
/*  srcs/utils                                                                */
/* ========================================================================== */

/* ---- start_check.c ---- */
void		check_input(int argc, char **argv, Camera *camera);

#endif
