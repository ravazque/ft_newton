
#include "newton.h"

/*
 * The catapult is two static boxes (base + arm) read from catapult.csv, plus
 * the launch parameters the player tunes live. Birds spawn just past the arm
 * tip, along the arm, so they never start inside a collider.
*/

static const char	*g_catapult_keys[] = {"position", "base_size", "arm_length", "arm_thickness", "arm_angle", "launch_speed", "launch_angle", "friction", "restitution", "color"};

/* The launch values must start inside the range the live controls keep them
 * in, otherwise the first key press would jump them there. */
static void	validate(const Catapult *c, CsvFile *csv)
{
	csv_expect_positive_vec3(csv, "base_size", c->baseSize);
	csv_expect_positive(csv, "arm_length", c->armLength);
	csv_expect_positive(csv, "arm_thickness", c->armThickness);
	csv_expect_range(csv, "arm_angle", c->armAngle, 0.0f, ARM_ANGLE_MAX);
	csv_expect_range(csv, "launch_speed", c->launchSpeed, LAUNCH_SPEED_MIN, LAUNCH_SPEED_MAX);
	csv_expect_range(csv, "launch_angle", c->launchAngle, LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX);
	csv_expect_min(csv, "friction", c->friction, 0.0f);
	csv_expect_range(csv, "restitution", c->restitution, 0.0f, 1.0f);
	csv_expect_range_vec3(csv, "color", c->color, 0.0f, 1.0f);
}

int	catapult_load(Catapult *c, const char *path, const ObjectDef *bird)
{
	CsvFile	csv;

	memset(c, 0, sizeof(*c));
	if (!csv_load(&csv, path))
		return (0);
	csv_reject_unknown(&csv, g_catapult_keys, 10);
	c->position = csv_get_vec3(&csv, "position");
	c->baseSize = csv_get_vec3(&csv, "base_size");
	c->armLength = csv_get_float(&csv, "arm_length");
	c->armThickness = csv_get_float(&csv, "arm_thickness");
	c->armAngle = csv_get_float(&csv, "arm_angle");
	c->launchSpeed = csv_get_float(&csv, "launch_speed");
	c->launchAngle = csv_get_float(&csv, "launch_angle");
	c->friction = csv_get_float(&csv, "friction");
	c->restitution = csv_get_float(&csv, "restitution");
	c->color = csv_get_vec3(&csv, "color");
	c->projectileMass = bird->mass;
	c->projectileRadius = bird->radius;
	c->launchPoint = c->position;
	if (csv.errors)
		return (0);
	validate(c, &csv);
	if (c->projectileMass < PROJECTILE_MASS_MIN)
		return (fprintf(stderr, "%s: the projectile mass (%g, from the bird file) must be >= %g\n", path, (double)c->projectileMass, (double)PROJECTILE_MASS_MIN), 0);
	return (csv.errors == 0);
}

/* Writes the catapult back in the file's own format (see objectdef_save).
 * The launch values written are the ones in effect right now. */
int	catapult_save(const Catapult *c, const char *path)
{
	FILE	*f = fopen(path, "w");

	if (!f)
		return (fprintf(stderr, "%s: cannot write file\n", path), 0);
	fprintf(f, "%s\n", CSV_HEADER);
	fprintf(f, "position,%g,%g,%g\n", (double)c->position.x, (double)c->position.y, (double)c->position.z);
	fprintf(f, "base_size,%g,%g,%g\n", (double)c->baseSize.x, (double)c->baseSize.y, (double)c->baseSize.z);
	fprintf(f, "arm_length,%g,,\narm_thickness,%g,,\narm_angle,%g,,\n", (double)c->armLength, (double)c->armThickness, (double)c->armAngle);
	fprintf(f, "launch_speed,%g,,\nlaunch_angle,%g,,\n", (double)c->launchSpeed, (double)c->launchAngle);
	fprintf(f, "friction,%g,,\nrestitution,%g,,\ncolor,%g,%g,%g\n", (double)c->friction, (double)c->restitution, (double)c->color.x, (double)c->color.y, (double)c->color.z);
	if (fclose(f) != 0)
		return (fprintf(stderr, "%s: write failed\n", path), 0);
	return (1);
}

/* The arm as a box: where its center sits, how big it is and how it is
 * turned. catapult_build stamps it into the world and catapult_spawn_point
 * needs the same numbers to keep a new bird outside it. */
static void	arm_box(const Catapult *c, Vec3 *center, Vec3 *half, Quat *rotation)
{
	Vec3	base_half = vec3_scale(c->baseSize, 0.5f);
	Vec3	pivot = vec3_add(c->position, vec3(-base_half.x + c->armThickness * 0.5f, c->baseSize.y, 0.0f));
	Vec3	along;

	*rotation = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(c->armAngle));
	along = quat_rotate(*rotation, vec3(1.0f, 0.0f, 0.0f));
	*center = vec3_add(pivot, vec3_scale(along, c->armLength * 0.5f));
	*half = vec3(c->armLength * 0.5f, c->armThickness * 0.5f, c->armThickness * 0.5f);
}

static RigidBody	static_box(const Catapult *c, Vec3 center, Vec3 half_extents, Quat orientation)
{
	RigidBody	b;

	b = rb_make();
	b.kind = KIND_CATAPULT;
	b.position = center;
	b.orientation = orientation;
	b.collider = collider_box(half_extents);
	b.friction = c->friction;
	b.restitution = c->restitution;
	b.color = c->color;
	rb_make_static(&b);
	return (b);
}

/* The arm pivots on the rear top edge of the base and points 'armAngle'
 * degrees above +X; the launch point sits one bird radius past its tip. */
void	catapult_build(Catapult *c, World *w)
{
	Vec3	half = vec3_scale(c->baseSize, 0.5f);
	Vec3	center;
	Vec3	arm_half;
	Quat	rot;
	Vec3	along;

	arm_box(c, &center, &arm_half, &rot);
	along = quat_rotate(rot, vec3(1.0f, 0.0f, 0.0f));
	world_add_body(w, static_box(c, vec3_add(c->position, vec3(0.0f, half.y, 0.0f)), half, quat_identity()));
	world_add_body(w, static_box(c, center, arm_half, rot));
	c->launchPoint = vec3_add(center, vec3_scale(along, c->armLength * 0.5f));
}

Vec3	catapult_direction(const Catapult *c)
{
	return (vec3(cosf(DEG2RAD(c->launchAngle)), sinf(DEG2RAD(c->launchAngle)), 0.0f));   /* [F25] */
}

/* How far an oriented box reaches from its center along 'direction': the sum
 * of its half extents projected on it. Moving past this distance guarantees
 * the box is left behind along that direction (the separating axis idea of
 * [F15], used here to place a body instead of to detect a contact). */
static float	box_reach(Vec3 half, Quat rotation, Vec3 direction)
{
	float	reach;

	reach = fabsf(half.x * vec3_dot(quat_rotate(rotation, vec3(1.0f, 0.0f, 0.0f)), direction));
	reach += fabsf(half.y * vec3_dot(quat_rotate(rotation, vec3(0.0f, 1.0f, 0.0f)), direction));
	reach += fabsf(half.z * vec3_dot(quat_rotate(rotation, vec3(0.0f, 0.0f, 1.0f)), direction));
	return (reach);
}

/* Distance to travel from 'origin' along 'direction' to leave one box behind:
 * how far its center is along that direction, plus the box's own reach. */
static float	clearance_for(Vec3 origin, Vec3 direction, Vec3 center, Vec3 half, Quat rotation)
{
	return (vec3_dot(vec3_sub(center, origin), direction) + box_reach(half, rotation, direction));
}

/*
 * Where a bird appears: on the firing line, past both parts of the catapult.
 *
 * Offsetting along the ARM (what the launch point itself is) only keeps the
 * bird clear while it is fired roughly along the arm; aiming steeply down
 * would start it inside the arm box and the solver would eject it sideways,
 * which is the "birds come out the front" bug. Offsetting along the LAUNCH
 * direction by each box's reach leaves both behind whatever the angle is.
 */
Vec3	catapult_spawn_point(const Catapult *c)
{
	Vec3	direction = catapult_direction(c);
	Vec3	base_center = vec3_add(c->position, vec3(0.0f, c->baseSize.y * 0.5f, 0.0f));
	Vec3	arm_center;
	Vec3	arm_half;
	Quat	arm_rotation;
	float	distance;
	Vec3	point;

	arm_box(c, &arm_center, &arm_half, &arm_rotation);
	distance = clearance_for(c->launchPoint, direction, base_center, vec3_scale(c->baseSize, 0.5f), quat_identity());
	distance = fmaxf(distance, clearance_for(c->launchPoint, direction, arm_center, arm_half, arm_rotation));
	distance = fmaxf(distance, 0.0f) + c->projectileRadius + LAUNCH_CLEARANCE;
	point = vec3_add(c->launchPoint, vec3_scale(direction, distance));
	if (point.y < c->projectileRadius + LAUNCH_CLEARANCE)
		point.y = c->projectileRadius + LAUNCH_CLEARANCE;
	return (point);
}

/* Builds a bird with the catapult's CURRENT parameters and adds it. */
int	catapult_fire(const Catapult *c, const ObjectDef *bird, World *w)
{
	return (world_add_body(w, projectile_make_bird(bird, catapult_spawn_point(c), vec3_scale(catapult_direction(c), c->launchSpeed), c->projectileMass)));   /* [F25] */
}
