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
# include <limits.h>
# include <dirent.h>
# include <sys/stat.h>
# include <glad/gl.h>
# include <GLFW/glfw3.h>

// Every type of the engine, then every prototype grouped by source folder in pipeline order (math to utils).

// ---- Maths ----

typedef struct Vec3
{
	float	x;
	float	y;
	float	z;
}	Vec3;

// Column-major: row r of column c is cells[c * 3 + r] (MAT3_AT); OpenGL uploads Mat4 without a transpose.
typedef struct Mat3
{
	float	cells[9];
}	Mat3;

typedef struct Mat4
{
	float	cells[16];
}	Mat4;

// Unit quaternion: an orientation.
typedef struct Quat
{
	float	w;
	float	x;
	float	y;
	float	z;
}	Quat;

// ---- Rendering ----

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

// Geometry on the GPU: vertex buffer (position + normal), index buffer, and the vertex array that reads them.
typedef struct Mesh
{
	unsigned int	vao;
	unsigned int	vbo;
	unsigned int	ebo;
	int				indexCount;
}	Mesh;

// Free-flying eye looking along (yaw, pitch); also carries the window size chosen on the command line.
typedef struct Camera
{
	Vec3	position;
	Vec3	up;
	float	yaw;
	float	pitch;
	float	moveSpeed;
	float	fovYDegrees;
	float	nearPlane;
	float	farPlane;
	int		windowWidth;
	int		windowHeight;
}	Camera;

// Batched 2D overlay: one vertex buffer of UI_VERTEX_FLOATS floats per vertex, drawn in one call per frame.
typedef struct Ui
{
	unsigned int	vao;
	unsigned int	vbo;
	float			*vertices;
	int				vertexCount;
	float			width;
	float			height;
}	Ui;

typedef struct Renderer
{
	Shader	shader;
	Mat4	view;
	Mat4	projection;
}	Renderer;

// The unit primitives every sphere, box and plane is drawn with, and one mesh per loaded model.
typedef struct Meshes
{
	Mesh	cube;
	Mesh	sphere;
	Mesh	plane;
	Mesh	models[MODEL_MAX];
}	Meshes;

// ---- Collision ----

// In the order the narrow-phase pairs them: every test takes the lower shape first.
typedef enum ShapeType
{
	SHAPE_SPHERE,
	SHAPE_BOX,
	SHAPE_MESH,
	SHAPE_PLANE
}	ShapeType;

// A node of a mesh's bounding volume tree: a leaf (left == -1) holds triangleCount entries of triangleOrder.
typedef struct BvhNode
{
	Vec3	boundsMin;
	Vec3	boundsMax;
	int		left;
	int		right;
	int		firstIndex;
	int		triangleCount;
}	BvhNode;

// A closed triangle surface in its body's frame, centre of mass at the origin, shared by every body of the model.
typedef struct MeshShape
{
	int		modelIndex;
	char	path[CSV_PATH_LEN];
	float	size;
	Vec3	*vertices;
	int		vertexCount;
	int		*corners;          // three vertex indices per triangle, counter-clockwise seen from outside
	int		*neighbours;       // per triangle, the triangle across edge k (from corner k to corner k + 1)
	int		triangleCount;
	int		*edges;            // two vertex indices per edge, each edge once
	int		edgeCount;
	Vec3	*faceNormals;
	Vec3	*vertexNormals;    // angle-weighted pseudo-normals
	BvhNode	*nodes;
	int		nodeCount;
	int		*triangleOrder;    // triangle indices grouped by tree leaf
	Vec3	boundsMin;
	Vec3	boundsMax;
	float	volume;
	Mat3	inertiaPerKg;      // inertia tensor of one kilogram of the solid
}	MeshShape;

// Only the fields of 'type' are used. A plane is normal . x = offset over a square of side 2 * halfSize.
typedef struct Collider
{
	ShapeType		type;
	float			radius;
	Vec3			halfExtents;
	Vec3			normal;
	float			offset;
	float			halfSize;
	const MeshShape	*mesh;
}	Collider;

// A broad-phase candidate, bodyA < bodyB.
typedef struct Pair
{
	int	bodyA;
	int	bodyB;
}	Pair;

// One contact point: the narrow-phase fills normal, point and penetration, the solver the rest every step.
typedef struct Contact
{
	int		bodyA;
	int		bodyB;
	Vec3	normal;            // unit, from bodyA toward bodyB
	Vec3	point;
	float	penetration;
	Vec3	tangent1;          // (normal, tangent1, tangent2) is orthonormal
	Vec3	tangent2;
	float	massNormal;        // effective masses along the normal and the tangents
	float	massTangent1;
	float	massTangent2;
	float	massRoll1;         // effective angular masses about the tangents (rolling) and the normal (spinning)
	float	massRoll2;
	float	massSpin;
	float	restitutionBias;   // separating speed the elasticity asks for
	float	rollingArm;        // rolling resistance coefficient times the sphere's radius
	float	scaleA;            // inverse mass scales: below 1 for a light body holding up a much heavier one [F39]
	float	scaleB;
	float	impulseNormal;     // accumulated impulses
	float	impulseTangent1;
	float	impulseTangent2;
	float	impulseRoll1;
	float	impulseRoll2;
	float	impulseSpin;
}	Contact;

// An oriented box: centre, the three world axes and the half extents along them.
typedef struct Obb
{
	Vec3	center;
	Vec3	axis[3];
	float	half[3];
}	Obb;

// The axis of least penetration of two boxes; faceOwner is 0 for a face of A, 1 for a face of B, 2 for two edges.
typedef struct SatResult
{
	float	penetration;
	Vec3	normal;
	int		faceOwner;
	int		faceIndex;
	int		edgeA;
	int		edgeB;
}	SatResult;

// The contact points of one mesh pair before the reduction; a full set keeps the deepest ones.
typedef struct ContactCandidates
{
	Contact	contacts[MESH_CANDIDATES];
	int		count;
}	ContactCandidates;

// The surface point of a mesh closest to a query point, its outward normal, and whether the query point is inside.
typedef struct MeshHit
{
	Vec3	point;
	Vec3	normal;
	float	distance;
	int		inside;
}	MeshHit;

// ---- Physics ----

// What a body was built from, so a changed definition finds its bodies.
typedef enum ObjectKind
{
	KIND_BLOCK,
	KIND_APPLE,
	KIND_GROUND,
	KIND_TREBUCHET,
	KIND_RAGDOLL,
	KIND_MODEL,
	KIND_RAIN
}	ObjectKind;

// A body that moves and turns but never deforms; inverseMass == 0 makes it static.
typedef struct RigidBody
{
	ObjectKind	kind;
	Vec3		position;
	Vec3		velocity;
	Vec3		forceAccum;
	float		mass;
	float		inverseMass;
	Quat		orientation;
	Vec3		angularVelocity;
	Vec3		torqueAccum;
	Mat3		inverseInertiaLocal;
	Mat3		inverseInertiaWorld;
	float		elasticity;
	float		friction;
	float		rollingResistance;
	Vec3		color;
	Collider	collider;
	int			awake;
	float		sleepTimer;
	Vec3		restPosition;      // pose, path and time since the body was last seen moving away [F24]
	Quat		restOrientation;
	float		restPath;
	float		restTurnPath;
	float		restWindow;
	float		creepTimer;        // time spent creeping: no faster on average than SLEEP_CREEP_FACTOR times the sleep speeds
	Vec3		creepPosition;     // pose when the creeping started, the creep speed is measured from it
	Quat		creepOrientation;
	int			ragdollId;         // 0 for a body that is no ragdoll part
	int			ragdollPart;
}	RigidBody;

typedef enum JointType
{
	JOINT_CONE,
	JOINT_HINGE
}	JointType;

// One angular row of a joint: a lock or a limit, along a world axis, with its accumulated impulse.
typedef struct JointRow
{
	Vec3	axis;
	float	mass;
	float	error;             // angle bodyB must still turn, for the positional pass
	float	bias;              // relative spin still allowed toward a limit this step
	float	lowerBound;
	float	upperBound;
	float	impulse;
	int		active;
}	JointRow;

// Two bodies held at a shared point; every frame is in its own body's space, and all angles are 0 where it was made.
typedef struct Joint
{
	JointType	type;
	int			bodyA;
	int			bodyB;
	int			collides;
	Vec3		anchorA;
	Vec3		anchorB;
	Vec3		axisA;             // twist axis of a cone, rotation axis of a hinge
	Vec3		axisB;
	Vec3		referenceA;        // perpendicular to the axis, where the twist or the hinge angle is 0
	Vec3		referenceB;
	float		swing;             // rad: half-angle of a cone
	float		angleMin;          // rad: twist range of a cone, angle range of a hinge
	float		angleMax;
	Vec3		armA;              // world arms of the anchors, this step
	Vec3		armB;
	Mat3		pointMass;         // inverse of the 3x3 effective mass of the shared point
	Vec3		pointImpulse;
	JointRow	rows[JOINT_ROWS];
}	Joint;

// Every body and joint, and the scratch of one step.
typedef struct World
{
	RigidBody	*bodies;
	int			bodyCount;
	int			bodyCapacity;
	Joint		*joints;
	int			jointCount;
	int			jointCapacity;
	int			ragdollCount;
	float		dt;
	Vec3		gravity;
	Pair		*pairs;
	int			pairCount;
	int			pairCapacity;
	Contact		*contacts;
	int			contactCount;
	int			contactCapacity;
	Contact		*previousContacts;
	int			previousContactCount;
	int			previousContactCapacity;
	int			*islandParent;
	float		*islandTimer;
	Vec3		*startPositions;
	Vec3		*rotationDelta;
	int			scratchCapacity;
	int			solverIterations;
}	World;

// ---- Data files ----

// One "property,value1,value2,value3" row: up to three numbers, or one word.
typedef struct CsvEntry
{
	char	key[CSV_KEY_LEN];
	char	word[CSV_KEY_LEN];
	float	values[3];
	int		valueCount;
	int		line;
}	CsvEntry;

typedef struct CsvFile
{
	char		path[CSV_PATH_LEN];
	CsvEntry	entries[CSV_MAX_ENTRIES];
	int			count;
	int			errors;
}	CsvFile;

// One kind of object read from its CSV file; only the fields of its shape are used.
typedef struct ObjectDef
{
	ObjectKind	kind;
	ShapeType	shape;
	float		radius;
	Vec3		size;
	Vec3		normal;
	float		offset;
	float		extent;
	float		modelSize;
	float		density;
	float		mass;
	float		friction;
	float		elasticity;
	float		rollingResistance;
	Vec3		color;
}	ObjectDef;

// A Wavefront .obj file reduced to positions and triangles.
typedef struct ObjMesh
{
	Vec3	*positions;
	int		positionCount;
	int		positionCapacity;
	int		*corners;
	int		triangleCount;
	int		cornerCapacity;
	float	minArea;
}	ObjMesh;

// An f line kept by the first pass, with the number of v lines before it (for negative indices).
typedef struct ObjFace
{
	char	*text;
	int		line;
	int		verticesBefore;
}	ObjFace;

// Scratch of one .obj load: the face lines, and the polygon being cut into triangles.
typedef struct ObjParser
{
	const char	*path;
	ObjFace		*faces;
	int			faceCount;
	int			faceCapacity;
	int			*polygon;
	int			polygonCount;
	int			polygonCapacity;
	int			*remaining;
	int			remainingCapacity;
	float		*projected;
	int			projectedCapacity;
}	ObjParser;

// ---- Game ----

// The launcher: five static boxes and the live launch settings.
typedef struct Trebuchet
{
	Vec3	position;
	Vec3	baseSize;
	float	frameHeight;
	float	armLength;
	float	armThickness;
	float	armAngle;
	Vec3	counterweightSize;
	float	launchSpeed;
	float	launchAngle;
	float	projectileMass;
	float	projectileRadius;
	float	friction;
	float	elasticity;
	Vec3	color;
	Vec3	launchPoint;
	float	spawnDistance;
}	Trebuchet;

typedef struct RagdollDef
{
	float	height;
	float	mass;
	float	friction;
	float	elasticity;
	Vec3	color;
}	RagdollDef;

// The models found in MODEL_DIR at start-up, and the one keys 6, 7 and 8 work with.
typedef struct ModelLibrary
{
	MeshShape	shapes[MODEL_MAX];
	char		names[MODEL_MAX][MODEL_NAME_LEN];
	char		labels[MODEL_MAX][MENU_LABEL_LEN];
	int			count;
	int			selected;
}	ModelLibrary;

// The window title readout: FPS, counters and live values.
typedef struct Hud
{
	int		visible;
	float	fps;
	float	refreshTimer;
	int		refresh;
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
	ACTION_QUIT,
	ACTION_MODELS,
	ACTION_BACK,
	ACTION_PLACE_MODEL
}	MenuAction;

typedef enum MenuPage
{
	MENU_PAGE_MAIN,
	MENU_PAGE_MODELS
}	MenuPage;

// One line of the menu; the layout writes its rectangles every frame and the input hit-tests the same ones.
typedef struct MenuRow
{
	MenuRowKind	kind;
	const char	*label;
	float		*value;
	float		min;
	float		max;
	float		step;
	int			decimals;
	MenuAction	action;
	int			model;
	int			column;
	float		x;
	float		y;
	float		width;
	float		height;
	float		minusX;
	float		plusX;
	float		buttonWidth;
}	MenuRow;

// hoverPart and heldPart: -1 nothing, 0 the [-] button, 1 the [+] button, 2 an action button.
typedef struct Menu
{
	int			open;
	MenuPage	page;
	MenuRow		rows[MENU_MAX_ROWS];
	int			rowCount;
	int			column;
	int			selected;
	int			hoverRow;
	int			hoverPart;
	int			heldRow;
	int			heldPart;
	float		repeatTimer;
	int			valueChanged;
	int			dirty;
	MenuAction	pending;
	int			pendingModel;
	float		panelX;
	float		panelY;
	float		panelWidth;
	float		panelHeight;
	float		scale;
	float		lineHeight;
	float		padding;
}	Menu;

typedef struct DebugDraw
{
	int	enabled;
}	DebugDraw;

// What the menu edits: a copy of the definitions and settings, committed by Apply.
typedef struct Draft
{
	ObjectDef	apple;
	ObjectDef	block;
	ObjectDef	ground;
	Trebuchet	trebuchet;
	RagdollDef	ragdoll;
	ObjectDef	model;
	float		gravityY;
	float		timeScale;
}	Draft;

typedef struct Game
{
	Window			window;
	World			world;
	Renderer		renderer;
	Ui				ui;
	Menu			menu;
	Camera			camera;
	DebugDraw		debug;
	Hud				hud;
	Trebuchet		trebuchet;         // live: the keys change its launch settings
	Trebuchet		trebuchetDef;      // what a reset returns to
	ObjectDef		appleDef;
	ObjectDef		blockDef;
	ObjectDef		groundDef;
	RagdollDef		ragdollDef;
	ObjectDef		modelDef;
	ModelLibrary	models;
	Draft			draft;
	float			fixedDt;
	float			timeScale;
	float			gravityBeforeOff;  // what the gravity on / off key (0) restores
	int				paused;
	int				quit;
	Meshes			meshes;
}	Game;

// ---- srcs/math ----

Vec3		vec3(float x, float y, float z);
Vec3		vec3_add(Vec3 a, Vec3 b);
Vec3		vec3_sub(Vec3 a, Vec3 b);
Vec3		vec3_neg(Vec3 vector);
Vec3		vec3_scale(Vec3 vector, float factor);
float		vec3_dot(Vec3 a, Vec3 b);
Vec3		vec3_cross(Vec3 a, Vec3 b);
float		vec3_length(Vec3 vector);
float		vec3_length_sq(Vec3 vector);
Vec3		vec3_normalized(Vec3 vector);
Vec3		vec3_min(Vec3 a, Vec3 b);
Vec3		vec3_max(Vec3 a, Vec3 b);

float		clampf(float value, float low, float high);

Mat3		mat3_identity(void);
Mat3		mat3_zero(void);
Mat3		mat3_diagonal(Vec3 diagonal);
Mat3		mat3_from_quat(Quat rotation);
Mat3		mat3_transpose(Mat3 matrix);
Mat3		mat3_inverse(Mat3 matrix);
Mat3		mat3_mul(Mat3 a, Mat3 b);
Vec3		mat3_mul_vec3(Mat3 matrix, Vec3 vector);
Mat3		mat3_add(Mat3 a, Mat3 b);
Mat3		mat3_scale(Mat3 matrix, float factor);
Mat3		mat3_skew(Vec3 vector);

Mat4		mat4_identity(void);
Mat4		mat4_translation(Vec3 offset);
Mat4		mat4_scale(Vec3 factors);
Mat4		mat4_from_quat(Quat rotation);
Mat4		mat4_transform(Vec3 position, Quat rotation, Vec3 scale);
Mat4		mat4_perspective(float fovy_radians, float aspect, float near_plane, float far_plane);
Mat4		mat4_look_at(Vec3 eye, Vec3 target, Vec3 up);
Mat4		mat4_mul(Mat4 a, Mat4 b);

Quat		quat_identity(void);
Quat		quat_from_axis_angle(Vec3 axis, float radians);
Quat		quat_from_to(Vec3 from, Vec3 to);
Quat		quat_mul(Quat a, Quat b);
Quat		quat_normalized(Quat rotation);
Quat		quat_conjugate(Quat rotation);
Vec3		quat_rotate(Quat rotation, Vec3 vector);
Quat		quat_integrate(Quat rotation, Vec3 angular_velocity, float dt);

// ---- srcs/physics ----

RigidBody	rigidbody_make(void);
void		rigidbody_set_mass(RigidBody *body, float mass);
void		rigidbody_make_static(RigidBody *body);
void		rigidbody_apply_force(RigidBody *body, Vec3 force);
void		rigidbody_apply_force_at_point(RigidBody *body, Vec3 force, Vec3 world_point);
void		rigidbody_apply_impulse_at_point(RigidBody *body, Vec3 impulse, Vec3 world_point);
void		rigidbody_apply_angular_impulse(RigidBody *body, Vec3 angular_impulse);
void		rigidbody_clear_accumulators(RigidBody *body);

Mat3		inertia_local_inverse(const Collider *collider, float mass);
void		inertia_update_world(RigidBody *body);

float		mesh_mass_properties(const MeshShape *mesh, Vec3 *center, Mat3 *inertia);

void		integrator_integrate(RigidBody *body, Vec3 gravity, float dt);

void		world_init(World *world);
void		world_destroy(World *world);
int			world_add_body(World *world, RigidBody body);
void		world_remove_body(World *world, int index);
void		world_clear(World *world);
void		world_wake_all(World *world);
int			world_grow_scratch(World *world);
void		world_step(World *world, float dt);

Joint		joint_make(const World *world, int body_a, int body_b, Vec3 anchor, Vec3 axis_a, Vec3 axis_b, Vec3 reference);
int			joint_add(World *world, Joint joint);
void		joint_forget_body(World *world, int removed, int moved);
int			joint_ignores_pair(const World *world, int body_a, int body_b);

void		sleep_update(World *world, float dt);

void		cull_lost_bodies(World *world);

// ---- srcs/collision ----

Collider	collider_sphere(float radius);
Collider	collider_box(Vec3 half_extents);
Collider	collider_plane(Vec3 normal, float offset, float size);
Collider	collider_mesh(const MeshShape *mesh);
Vec3		collider_plane_origin(const Collider *collider);
Quat		collider_plane_rotation(const Collider *collider);
float		collider_plane_distance(const Collider *collider, Vec3 point);
int			collider_plane_covers(const Collider *collider, Vec3 point);
void		collider_bounds(const RigidBody *body, Vec3 *bounds_min, Vec3 *bounds_max);

void		broadphase_compute_pairs(World *world);

void		narrowphase_generate_contacts(World *world);
int			narrowphase_pair(const RigidBody *body_a, const RigidBody *body_b, Contact *out);

int			overlap_bodies(const RigidBody *body_a, const RigidBody *body_b);
int			overlap_find(const World *world, const RigidBody *candidate);

// Each writes up to MAX_CONTACTS_PER_PAIR contacts, normal from the first body toward the second, and returns how many.
int			contact_sphere_sphere(const RigidBody *sphere_a, const RigidBody *sphere_b, Contact *out);
int			contact_sphere_plane(const RigidBody *sphere, const RigidBody *plane, Contact *out);
int			contact_sphere_box(const RigidBody *sphere, const RigidBody *box, Contact *out);
int			contact_box_plane(const RigidBody *box, const RigidBody *plane, Contact *out);
int			contact_box_box(const RigidBody *box_a, const RigidBody *box_b, Contact *out);
int			contact_sphere_mesh(const RigidBody *sphere, const RigidBody *mesh, Contact *out);
int			contact_mesh_mesh(const RigidBody *mesh_a, const RigidBody *mesh_b, Contact *out);
int			contact_mesh_plane(const RigidBody *mesh, const RigidBody *plane, Contact *out);
int			contact_box_mesh(const RigidBody *box, const RigidBody *mesh, Contact *out);

Obb			obb_from_body(const RigidBody *body);
float		obb_radius(const Obb *box, Vec3 direction);
Vec3		obb_corner(const Obb *box, int corner);
void		obb_support_edge(const Obb *box, int axis_index, Vec3 direction, Vec3 *start, Vec3 *end);
void		obb_incident_face(const Obb *box, Vec3 reference_normal, Vec3 *polygon);

int			sat_boxes(const Obb *box_a, const Obb *box_b, SatResult *result);
int			sat_clearly_better(float penetration, float best);

int			manifold_clip(Vec3 *polygon, int count, Vec3 normal, float offset);
int			manifold_reduce(const Contact *candidates, int count, Vec3 axis, Contact *out);
void		manifold_add_candidate(ContactCandidates *set, Vec3 point, Vec3 normal, float depth);
int			manifold_reduce_candidates(const ContactCandidates *set, Contact *out);

int			mesh_shape_load(MeshShape *mesh, const char *path, float size, int model_index);
void		mesh_shape_rescale(MeshShape *mesh, float size);
void		mesh_shape_free(MeshShape *mesh);
Vec3		mesh_to_local(const RigidBody *body, Vec3 world_point);
Vec3		mesh_to_world(const RigidBody *body, Vec3 local_point);

int			mesh_repair(MeshShape *mesh, const char *path);

int			mesh_topology_build(MeshShape *mesh, const char *path);
int			mesh_normals_build(MeshShape *mesh);

int			bvh_build(MeshShape *mesh);
void		bvh_query(const MeshShape *mesh, Vec3 bounds_min, Vec3 bounds_max, void (*visit)(void *context, int triangle), void *context);

Vec3		closest_on_triangle(Vec3 point, Vec3 a, Vec3 b, Vec3 c, int *feature);
int			segment_crosses_triangle(Vec3 start, Vec3 end, const Vec3 *corner, float *fraction);
void		closest_on_segments(Vec3 start1, Vec3 end1, Vec3 start2, Vec3 end2, Vec3 *closest1, Vec3 *closest2);

void		mesh_triangle_corners(const MeshShape *mesh, int triangle, Vec3 *corner);
Vec3		mesh_feature_normal(const MeshShape *mesh, int triangle, int feature);
int			mesh_closest_point(const MeshShape *mesh, Vec3 point, float radius, MeshHit *hit);

// ---- srcs/response ----

void		resolver_resolve(World *world);

void		impulse_prepare(const World *world, Contact *contact);
void		impulse_apply(RigidBody *body_a, RigidBody *body_b, const Contact *contact, Vec3 impulse);
void		impulse_solve(World *world, Contact *contact);

void		correction_apply(World *world);
void		correction_push(World *world, int body, Vec3 arm, Vec3 push);
void		correction_turn(World *world, int body, Vec3 angular_push);

void		joint_point_prepare(World *world, Joint *joint);
void		joint_point_solve(World *world, Joint *joint);
void		joint_point_correct(World *world, Joint *joint);
void		joint_point_project(World *world, Joint *joint);

void		joint_limit_prepare(World *world, Joint *joint);
void		joint_limit_solve(World *world, Joint *joint);
void		joint_limit_correct(World *world, Joint *joint);
void		joint_limit_project(World *world, Joint *joint);

// ---- srcs/render ----

int			window_init(Window *window, int width, int height, const char *title);
void		window_destroy(Window *window);
int			window_should_close(const Window *window);
void		window_poll_events(Window *window);
void		window_swap_buffers(Window *window);
float		window_aspect(const Window *window);
void		window_size(const Window *window, float *width, float *height);
void		window_cursor(const Window *window, float *x, float *y);
int			window_mouse_down(const Window *window);
int			window_key_down(const Window *window, int key);
struct GLFWwindow	*window_handle(const Window *window);

int			shader_load(Shader *shader, const char *vertex_path, const char *fragment_path);
void		shader_use(const Shader *shader);
void		shader_set_mat4(const Shader *shader, const char *name, Mat4 value);
void		shader_set_vec3(const Shader *shader, const char *name, Vec3 value);
void		shader_set_int(const Shader *shader, const char *name, int value);

Mesh		mesh_cube(void);
Mesh		mesh_sphere(int segments);
Mesh		mesh_plane(void);
Mesh		mesh_from_shape(const MeshShape *shape);
void		mesh_draw(const Mesh *mesh);
void		mesh_release(Mesh *mesh);

Camera		camera_default(void);
void		camera_look(Camera *camera, float delta_yaw, float delta_pitch);
Vec3		camera_world_delta(const Camera *camera, Vec3 local_delta);
void		camera_change_speed(Camera *camera, float delta);
Mat4		camera_view(const Camera *camera);
Mat4		camera_projection(const Camera *camera, float aspect);

int			renderer_init(Renderer *renderer);
void		renderer_begin_frame(Renderer *renderer, const Camera *camera, float aspect);
void		renderer_draw(Renderer *renderer, const Mesh *mesh, Mat4 model, Vec3 color);
void		renderer_draw_flat(Renderer *renderer, const Mesh *mesh, Mat4 model, Vec3 color);
void		renderer_set_wireframe(Renderer *renderer, int enabled);
Mat4		renderer_body_model(const RigidBody *body, float inflate);
const Mesh	*renderer_body_mesh(const RigidBody *body, const Meshes *meshes);

void		debugdraw_draw(const DebugDraw *debug, const World *world, Renderer *renderer, const Meshes *meshes);

const unsigned char	*font_glyph(char character);

int			ui_init(Ui *ui);
void		ui_destroy(Ui *ui);
void		ui_begin(Ui *ui, float width, float height);
void		ui_rect(Ui *ui, float x, float y, float width, float height, Vec3 color);
void		ui_border(Ui *ui, float x, float y, float width, float height, float thickness, Vec3 color);
void		ui_text(Ui *ui, const char *text, float x, float y, float scale, Vec3 color);
void		ui_text_right(Ui *ui, const char *text, float right, float y, float scale, Vec3 color);
float		ui_text_width(const char *text, float scale);
void		ui_end(Ui *ui, Renderer *renderer);

// ---- srcs/menu ----

void		menu_add_section(Menu *menu, const char *label, int column);
void		menu_add_value(Menu *menu, const char *label, float *value, float min_value, float max_value, float step, int decimals);
void		menu_add_action(Menu *menu, const char *label, MenuAction action);
void		menu_add_model(Menu *menu, const char *label, int model);
void		menu_add_hint(Menu *menu, const char *label);
int			menu_is_selectable(const MenuRow *row);
void		menu_step_value(Menu *menu, MenuRow *row, int direction);

void		menu_layout(Menu *menu, float width, float height);

void		menu_update(Menu *menu, Window *window, float frame_time);

void		menu_draw(const Menu *menu, Ui *ui);

// ---- srcs/data ----

int			csv_load(CsvFile *csv, const char *path);
const CsvEntry	*csv_find(const CsvFile *csv, const char *key);

int			csv_reject_unknown(CsvFile *csv, const char **allowed, int allowed_count);
float		csv_get_float(CsvFile *csv, const char *key);
Vec3		csv_get_vec3(CsvFile *csv, const char *key);
const char	*csv_get_word(CsvFile *csv, const char *key);
void		csv_error(CsvFile *csv, const char *key, const char *expected);
int			csv_expect_positive(CsvFile *csv, const char *key, float value);
int			csv_expect_min(CsvFile *csv, const char *key, float value, float low);
int			csv_expect_range(CsvFile *csv, const char *key, float value, float low, float high);
int			csv_expect_positive_vec3(CsvFile *csv, const char *key, Vec3 value);
int			csv_expect_range_vec3(CsvFile *csv, const char *key, Vec3 value, float low, float high);

int			object_def_load(ObjectDef *def, const char *path, ObjectKind kind);
RigidBody	object_def_make_body(const ObjectDef *def, Vec3 position);
int			object_def_save(const ObjectDef *def, const char *path);

int			trebuchet_load(Trebuchet *trebuchet, const char *path, const ObjectDef *apple);
int			trebuchet_save(const Trebuchet *trebuchet, const char *path);

int			ragdoll_load(RagdollDef *ragdoll, const char *path);
int			ragdoll_save(const RagdollDef *ragdoll, const char *path);

int			obj_load(const char *path, ObjMesh *obj);
void		obj_free(ObjMesh *obj);
char		*obj_token(char **cursor);
int			obj_error(const ObjParser *parser, int line, const char *message, const char *token);

int			obj_parse_face(ObjMesh *obj, ObjParser *parser, const ObjFace *face);

int			obj_triangulate(ObjMesh *obj, ObjParser *parser, int corner_count);

// ---- srcs/game ----

int			game_init(Game *game, int argc, char **argv);
void		game_run(Game *game);
void		game_shutdown(Game *game);

void		draw_frame(Game *game);

void		input_poll(Game *game, float frame_time);

Hud			hud_default(void);
void		hud_update(Hud *hud, float frame_time);
void		hud_draw(Hud *hud, const Game *game);

int			scene_load_assets(Game *game);
int			scene_check_start(const ObjectDef *ground, const ObjectDef *block, const Trebuchet *trebuchet);
void		scene_build(Game *game);
void		scene_apply_definitions(Game *game);

int			model_library_load(ModelLibrary *library, float size);
void		model_library_free(ModelLibrary *library);
void		model_library_step(ModelLibrary *library, int direction);
void		model_library_resize(ModelLibrary *library, float size);

int			model_spawn(World *world, const ObjectDef *def, const MeshShape *shape, Vec3 origin);
void		model_resize_all(World *world, ModelLibrary *library, float size);

int			rain_drop(World *world, const ObjectDef *apple, Vec3 center);

float		camera_body_overlap(Vec3 center, const RigidBody *body, Vec3 *normal);
void		camera_keep_clear(Camera *camera, const World *world);
void		camera_fly(Camera *camera, const World *world, Vec3 local_delta);

float		placement_add(World *world, RigidBody *bodies, int count);

void		action_sync_draft(Game *game);
void		action_run(Game *game, MenuAction action);

void		menu_rows_build(Game *game);

void		trebuchet_parts(const Trebuchet *trebuchet, Vec3 *center, Vec3 *half, Quat *rotation);
Vec3		trebuchet_launch_point(const Trebuchet *trebuchet);
Vec3		trebuchet_direction(const Trebuchet *trebuchet);
Vec3		trebuchet_spawn_point(const Trebuchet *trebuchet);
void		trebuchet_build(Trebuchet *trebuchet, World *world);
int			trebuchet_fire(const Trebuchet *trebuchet, const ObjectDef *apple, World *world);

float		trebuchet_spawn_distance(const Trebuchet *trebuchet);

RigidBody	projectile_make_apple(const ObjectDef *apple, Vec3 position, Vec3 velocity, float mass);

float		structure_spawn_wall(World *world, const ObjectDef *block, Vec3 origin, int columns, int rows);
float		structure_spawn_pyramid(World *world, const ObjectDef *block, Vec3 origin, int base_count);
float		structure_spawn_tower(World *world, const ObjectDef *block, Vec3 origin, int height);

int			ragdoll_spawn(World *world, const RagdollDef *def, Vec3 origin);
float		ragdoll_mass_share(int part);

// ---- srcs/utils ----

void		arguments_read(int argc, char **argv, Camera *camera);

void		*array_reserve(void *data, int *capacity, int need, size_t element_size);

char		*file_read(const char *path);

#endif
