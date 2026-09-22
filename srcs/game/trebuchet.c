
#include "newton.h"

/*
 * THE TREBUCHET: the machine, and where a shot starts.
 *
 * Five static boxes read from trebuchet.csv - the sill lying on the ground,
 * the two legs of the A-frame carrying the pivot, the throwing arm turning on
 * that pivot, and the counterweight hanging off its short side. Only the
 * silhouette is built as bodies; the shot itself stays the launch speed and
 * angle the player sets, so those remain a direct control.
 *
 * Apples leave from the tip of the throwing arm, always the same distance down
 * the firing line, so the spawn point and the aim bar agree at every angle.
 *
 * No physics lives here beyond the launch velocity [F25]: the trajectory is
 * gravity's doing, in srcs/physics/. The maths is in srcs/math/.
*/

static const char	*g_trebuchet_keys[] = {"position", "base_size", "frame_height", "arm_length", "arm_thickness", "arm_angle", "counterweight_size", "launch_speed", "launch_angle", "friction", "elasticity", "color"};

/* ========================================================================== */
/*  GEOMETRY: where every part of the machine sits                            */
/* ========================================================================== */

/* The pivot the arm turns on: the top of the A-frame. */
static Vec3	pivot_point(const Trebuchet *t)
{
	return (vec3_add(t->position, vec3(0.0f, t->frameHeight, 0.0f)));
}

/* Unit vector along the throwing side of the arm. */
static Vec3	arm_direction(const Trebuchet *t)
{
	return (quat_rotate(quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(t->armAngle)), vec3(1.0f, 0.0f, 0.0f)));
}

/* The arm tip on the throwing side: where an apple leaves the machine. */
static Vec3	launch_point(const Trebuchet *t)
{
	return (vec3_add(pivot_point(t), vec3_scale(arm_direction(t), t->armLength * TREB_PIVOT_RATIO)));
}

Vec3	trebuchet_direction(const Trebuchet *t)
{
	return (vec3(cosf(DEG2RAD(t->launchAngle)), sinf(DEG2RAD(t->launchAngle)), 0.0f));   /* [F25] */
}

/* One leg of the A-frame, as a box running from its foot on the sill up to
 * the pivot. The cross section is square, so the roll quat_from_to leaves
 * free does not matter. */
static void	leg_box(Vec3 foot, Vec3 pivot, float thickness, Vec3 *center, Vec3 *half, Quat *rotation)
{
	Vec3	span = vec3_sub(pivot, foot);
	float	length = vec3_length(span);

	*center = vec3_scale(vec3_add(foot, pivot), 0.5f);
	*half = vec3(length * 0.5f, thickness * 0.5f, thickness * 0.5f);
	*rotation = quat_from_to(vec3(1.0f, 0.0f, 0.0f), vec3_scale(span, 1.0f / length));
}

/* Every box the machine is made of, in world space. trebuchet_build stamps
 * them into the world and the spawn distance below is measured against the
 * very same numbers, so the two can never drift apart. */
static void	trebuchet_parts(const Trebuchet *t, Vec3 *center, Vec3 *half, Quat *rotation)
{
	Vec3	pivot = pivot_point(t);
	Vec3	along = arm_direction(t);
	float	spread = t->baseSize.x * TREB_LEG_SPREAD_RATIO;
	float	thickness = t->frameHeight * TREB_LEG_THICK_RATIO;
	Vec3	short_end = vec3_sub(pivot, vec3_scale(along, t->armLength * (1.0f - TREB_PIVOT_RATIO)));

	center[0] = vec3_add(t->position, vec3(0.0f, t->baseSize.y * 0.5f, 0.0f));
	half[0] = vec3_scale(t->baseSize, 0.5f);
	rotation[0] = quat_identity();
	leg_box(vec3_add(t->position, vec3(-spread, t->baseSize.y, 0.0f)), pivot, thickness, &center[1], &half[1], &rotation[1]);
	leg_box(vec3_add(t->position, vec3(spread, t->baseSize.y, 0.0f)), pivot, thickness, &center[2], &half[2], &rotation[2]);
	center[3] = vec3_add(pivot, vec3_scale(along, t->armLength * (TREB_PIVOT_RATIO - 0.5f)));
	half[3] = vec3(t->armLength * 0.5f, t->armThickness * 0.5f, t->armThickness * 0.5f);
	rotation[3] = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(t->armAngle));
	center[4] = vec3_sub(short_end, vec3(0.0f, t->counterweightSize.y * 0.5f, 0.0f));
	half[4] = vec3_scale(t->counterweightSize, 0.5f);
	rotation[4] = quat_identity();
}

/* ========================================================================== */
/*  SPAWN DISTANCE: one number, measured once, for every launch angle         */
/* ========================================================================== */

/* One axis of the slab test: narrows [enter, leave] to the stretch of the ray
 * that is inside the box on that axis. A ray parallel to the slab is either
 * always inside it or never, which is what the zero-direction branch says. */
static void	clip_slab(float origin, float direction, float extent, float *enter, float *leave)
{
	float	near_t;
	float	far_t;
	float	swap;

	if (fabsf(direction) < 1e-6f)
	{
		if (fabsf(origin) > extent)
			*leave = -1.0f;
		return ;
	}
	near_t = (-extent - origin) / direction;
	far_t = (extent - origin) / direction;
	if (near_t > far_t)
	{
		swap = near_t;
		near_t = far_t;
		far_t = swap;
	}
	*enter = fmaxf(*enter, near_t);
	*leave = fminf(*leave, far_t);
}

/* How far along 'direction' a ray from 'origin' leaves a box grown by
 * 'radius', or 0 when it never enters it. Growing the box by the apple's
 * radius turns "where does its center stop overlapping" into a plain ray
 * test; it is exact on the faces and slightly generous at the corners, which
 * is the safe side when the answer decides where to put a body. */
static float	box_exit_distance(Vec3 origin, Vec3 direction, Vec3 center, Vec3 half, Quat rotation, float radius)
{
	Quat	inverse = {rotation.w, -rotation.x, -rotation.y, -rotation.z};
	Vec3	local = quat_rotate(inverse, vec3_sub(origin, center));
	Vec3	along = quat_rotate(inverse, direction);
	float	enter = -INFINITY;
	float	leave = INFINITY;

	clip_slab(local.x, along.x, half.x + radius, &enter, &leave);
	clip_slab(local.y, along.y, half.y + radius, &enter, &leave);
	clip_slab(local.z, along.z, half.z + radius, &enter, &leave);
	if (leave < enter || leave <= 0.0f)
		return (0.0f);
	return (leave);
}

/* How far an apple fired at 'angle' must travel from the arm tip before it is
 * clear of every part of the machine. */
static float	clearance_at(const Trebuchet *t, float angle, const Vec3 *center, const Vec3 *half, const Quat *rotation)
{
	Vec3	direction = vec3(cosf(DEG2RAD(angle)), sinf(DEG2RAD(angle)), 0.0f);
	float	worst = 0.0f;
	int		i;

	i = 0;
	while (i < TREB_PARTS)
	{
		worst = fmaxf(worst, box_exit_distance(t->launchPoint, direction, center[i], half[i], rotation[i], t->projectileRadius));
		i++;
	}
	return (worst);
}

/*
 * The single distance every apple travels from the arm tip before it appears.
 *
 * Measuring it per shot is what made an apple creep forward as the aim went
 * down, and it left the yellow bar and the real spawn point disagreeing. The
 * worst case over the whole range of angles the controls allow gives ONE
 * number, so an apple always leaves from the same place: the tip of the bar.
 * It is measured from the real geometry rather than hard-coded, so a different
 * frame in the file still gets a distance that clears it.
 */
static float	spawn_distance(const Trebuchet *t)
{
	Vec3	center[TREB_PARTS];
	Vec3	half[TREB_PARTS];
	Quat	rotation[TREB_PARTS];
	float	worst = 0.0f;
	float	angle = LAUNCH_ANGLE_MIN;

	trebuchet_parts(t, center, half, rotation);
	while (angle <= LAUNCH_ANGLE_MAX)
	{
		worst = fmaxf(worst, clearance_at(t, angle, center, half, rotation));
		angle += TREB_ANGLE_STEP;
	}
	return (worst + LAUNCH_CLEARANCE);
}

/* Where an apple appears: always the same distance down the firing line, so
 * it leaves exactly at the tip of the aim bar whatever the launch angle is. */
Vec3	trebuchet_spawn_point(const Trebuchet *t)
{
	return (vec3_add(t->launchPoint, vec3_scale(trebuchet_direction(t), t->spawnDistance)));
}

/* ========================================================================== */
/*  THE FILE: strict load and save                                            */
/* ========================================================================== */

/* The launch values must start inside the range the live controls keep them
 * in, otherwise the first key press would jump them there. */
static void	validate(const Trebuchet *t, CsvFile *csv)
{
	csv_expect_positive_vec3(csv, "base_size", t->baseSize);
	csv_expect_positive(csv, "frame_height", t->frameHeight);
	csv_expect_positive(csv, "arm_length", t->armLength);
	csv_expect_positive(csv, "arm_thickness", t->armThickness);
	csv_expect_range(csv, "arm_angle", t->armAngle, 0.0f, ARM_ANGLE_MAX);
	csv_expect_positive_vec3(csv, "counterweight_size", t->counterweightSize);
	csv_expect_range(csv, "launch_speed", t->launchSpeed, LAUNCH_SPEED_MIN, LAUNCH_SPEED_MAX);
	csv_expect_range(csv, "launch_angle", t->launchAngle, LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX);
	csv_expect_min(csv, "friction", t->friction, 0.0f);
	csv_expect_range(csv, "elasticity", t->elasticity, 0.0f, 1.0f);
	csv_expect_range_vec3(csv, "color", t->color, 0.0f, 1.0f);
	if (t->frameHeight <= t->baseSize.y)
		csv_error(csv, "frame_height", "above the sill, so the arm can turn");
}

int	trebuchet_load(Trebuchet *t, const char *path, const ObjectDef *apple)
{
	CsvFile	csv;

	memset(t, 0, sizeof(*t));
	if (!csv_load(&csv, path))
		return (0);
	csv_reject_unknown(&csv, g_trebuchet_keys, 12);
	t->position = csv_get_vec3(&csv, "position");
	t->baseSize = csv_get_vec3(&csv, "base_size");
	t->frameHeight = csv_get_float(&csv, "frame_height");
	t->armLength = csv_get_float(&csv, "arm_length");
	t->armThickness = csv_get_float(&csv, "arm_thickness");
	t->armAngle = csv_get_float(&csv, "arm_angle");
	t->counterweightSize = csv_get_vec3(&csv, "counterweight_size");
	t->launchSpeed = csv_get_float(&csv, "launch_speed");
	t->launchAngle = csv_get_float(&csv, "launch_angle");
	t->friction = csv_get_float(&csv, "friction");
	t->elasticity = csv_get_float(&csv, "elasticity");
	t->color = csv_get_vec3(&csv, "color");
	t->projectileMass = apple->mass;
	t->projectileRadius = apple->radius;
	if (csv.errors)
		return (0);
	validate(t, &csv);
	if (csv.errors)
		return (0);
	if (t->projectileMass < PROJECTILE_MASS_MIN)
		return (fprintf(stderr, "%s: the projectile mass (%g, from %s) must be >= %g\n", path, (double)t->projectileMass, ASSET_APPLE, (double)PROJECTILE_MASS_MIN), 0);
	t->launchPoint = launch_point(t);
	t->spawnDistance = spawn_distance(t);
	return (1);
}

/* Writes the trebuchet back in the file's own format (see objectdef_save).
 * The launch values written are the ones in effect right now. */
int	trebuchet_save(const Trebuchet *t, const char *path)
{
	FILE	*f = fopen(path, "w");

	if (!f)
		return (fprintf(stderr, "%s: cannot write file\n", path), 0);
	fprintf(f, "%s\n", CSV_HEADER);
	fprintf(f, "position,%g,%g,%g\n", (double)t->position.x, (double)t->position.y, (double)t->position.z);
	fprintf(f, "base_size,%g,%g,%g\n", (double)t->baseSize.x, (double)t->baseSize.y, (double)t->baseSize.z);
	fprintf(f, "frame_height,%g,,\n", (double)t->frameHeight);
	fprintf(f, "arm_length,%g,,\narm_thickness,%g,,\narm_angle,%g,,\n", (double)t->armLength, (double)t->armThickness, (double)t->armAngle);
	fprintf(f, "counterweight_size,%g,%g,%g\n", (double)t->counterweightSize.x, (double)t->counterweightSize.y, (double)t->counterweightSize.z);
	fprintf(f, "launch_speed,%g,,\nlaunch_angle,%g,,\n", (double)t->launchSpeed, (double)t->launchAngle);
	fprintf(f, "friction,%g,,\nelasticity,%g,,\ncolor,%g,%g,%g\n", (double)t->friction, (double)t->elasticity, (double)t->color.x, (double)t->color.y, (double)t->color.z);
	if (fclose(f) != 0)
		return (fprintf(stderr, "%s: write failed\n", path), 0);
	return (1);
}

/* ========================================================================== */
/*  THE WORLD: stamping the machine in, and firing                            */
/* ========================================================================== */

static RigidBody	static_box(const Trebuchet *t, Vec3 center, Vec3 half_extents, Quat orientation)
{
	RigidBody	b;

	b = rb_make();
	b.kind = KIND_TREBUCHET;
	b.position = center;
	b.orientation = orientation;
	b.collider = collider_box(half_extents);
	b.friction = t->friction;
	b.elasticity = t->elasticity;
	b.color = t->color;
	rb_make_static(&b);
	return (b);
}

void	trebuchet_build(Trebuchet *t, World *w)
{
	Vec3	center[TREB_PARTS];
	Vec3	half[TREB_PARTS];
	Quat	rotation[TREB_PARTS];
	int		i;

	trebuchet_parts(t, center, half, rotation);
	i = 0;
	while (i < TREB_PARTS)
	{
		world_add_body(w, static_box(t, center[i], half[i], rotation[i]));
		i++;
	}
	t->launchPoint = launch_point(t);
	t->spawnDistance = spawn_distance(t);
}

/* Builds an apple with the trebuchet's CURRENT parameters and adds it. */
int	trebuchet_fire(const Trebuchet *t, const ObjectDef *apple, World *w)
{
	return (world_add_body(w, projectile_make_apple(apple, trebuchet_spawn_point(t), vec3_scale(trebuchet_direction(t), t->launchSpeed), t->projectileMass)));   /* [F25] */
}
