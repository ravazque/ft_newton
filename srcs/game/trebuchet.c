#include "newton.h"

/*
 * The trebuchet: five static boxes (sill, two A-frame legs, throwing arm,
 * counterweight) built from trebuchet.csv, and the shot. The shot is not
 * simulated from the counterweight: an apple appears past the arm tip with
 * velocity speed * (cos a, sin a, 0) [F25], so speed and direction stay direct
 * controls, and from then on gravity alone draws the parabola.
*/

static Vec3	pivot_point(const Trebuchet *t)
{
	return (vec3_add(t->position, vec3(0.0f, t->frameHeight, 0.0f)));
}

/* Unit vector along the throwing side of the arm. */
static Vec3	arm_direction(const Trebuchet *t)
{
	return (quat_rotate(quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(t->armAngle)), vec3(1.0f, 0.0f, 0.0f)));
}

/* The tip of the throwing side: where apples leave the machine. */
Vec3	trebuchet_launch_point(const Trebuchet *t)
{
	return (vec3_add(pivot_point(t), vec3_scale(arm_direction(t), t->armLength * TREB_PIVOT_RATIO)));
}

Vec3	trebuchet_direction(const Trebuchet *t)
{
	return (vec3(cosf(DEG2RAD(t->launchAngle)), sinf(DEG2RAD(t->launchAngle)), 0.0f));   /* [F25] */
}

/* A leg runs from its foot on the sill to the pivot; its square section makes the roll irrelevant. */
static void	leg_box(Vec3 foot, Vec3 pivot, float thickness, Vec3 *center, Vec3 *half, Quat *rotation)
{
	Vec3	span = vec3_sub(pivot, foot);
	float	length = vec3_length(span);

	*center = vec3_scale(vec3_add(foot, pivot), 0.5f);
	*half = vec3(length * 0.5f, thickness * 0.5f, thickness * 0.5f);
	*rotation = quat_from_to(vec3(1.0f, 0.0f, 0.0f), vec3_scale(span, 1.0f / length));
}

/* Every box of the machine in world space; the build and the spawn clearance both read these. */
void	trebuchet_parts(const Trebuchet *t, Vec3 *center, Vec3 *half, Quat *rotation)
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

/* Always the same distance down the firing line, so apples leave from the tip of the aim bar. */
Vec3	trebuchet_spawn_point(const Trebuchet *t)
{
	return (vec3_add(t->launchPoint, vec3_scale(trebuchet_direction(t), t->spawnDistance)));
}

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
	t->launchPoint = trebuchet_launch_point(t);
	t->spawnDistance = trebuchet_spawn_distance(t);
}

/* An apple with the CURRENT launch settings; none when something occupies the spawn point. */
int	trebuchet_fire(const Trebuchet *t, const ObjectDef *apple, World *w)
{
	Vec3		velocity = vec3_scale(trebuchet_direction(t), t->launchSpeed);   /* [F25] v0 = speed (cos a, sin a, 0) */
	RigidBody	shot = projectile_make_apple(apple, trebuchet_spawn_point(t), velocity, t->projectileMass);

	if (world_first_overlap(w, &shot) >= 0)
		return (printf("Launch point blocked: an apple would start inside another body\n"), -1);
	return (world_add_body(w, shot));
}
