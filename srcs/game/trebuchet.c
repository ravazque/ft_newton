#include "newton.h"

// The trebuchet: five static boxes (sill, two legs, arm, counterweight) and the shot, an apple leaving the arm tip.

static Vec3	pivot_point(const Trebuchet *trebuchet)
{
	return (vec3_add(trebuchet->position, vec3(0.0f, trebuchet->frameHeight, 0.0f)));
}

static Vec3	arm_direction(const Trebuchet *trebuchet)
{
	return (quat_rotate(quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(trebuchet->armAngle)), vec3(1.0f, 0.0f, 0.0f)));
}

Vec3	trebuchet_launch_point(const Trebuchet *trebuchet)
{
	return (vec3_add(pivot_point(trebuchet), vec3_scale(arm_direction(trebuchet), trebuchet->armLength * TREB_PIVOT_RATIO)));
}

Vec3	trebuchet_direction(const Trebuchet *trebuchet)
{
	return (vec3(cosf(DEG2RAD(trebuchet->launchAngle)), sinf(DEG2RAD(trebuchet->launchAngle)), 0.0f));   // [F25]
}

static void	leg_box(Vec3 foot, Vec3 pivot, float thickness, Vec3 *center, Vec3 *half, Quat *rotation)
{
	Vec3	span = vec3_sub(pivot, foot);
	float	length = vec3_length(span);

	*center = vec3_scale(vec3_add(foot, pivot), 0.5f);
	*half = vec3(length * 0.5f, thickness * 0.5f, thickness * 0.5f);
	*rotation = quat_from_to(vec3(1.0f, 0.0f, 0.0f), vec3_scale(span, 1.0f / length));
}

void	trebuchet_parts(const Trebuchet *trebuchet, Vec3 *center, Vec3 *half, Quat *rotation)
{
	Vec3	pivot = pivot_point(trebuchet);
	Vec3	along = arm_direction(trebuchet);
	float	spread = trebuchet->baseSize.x * TREB_LEG_SPREAD_RATIO;
	float	thickness = trebuchet->frameHeight * TREB_LEG_THICK_RATIO;
	Vec3	short_end = vec3_sub(pivot, vec3_scale(along, trebuchet->armLength * (1.0f - TREB_PIVOT_RATIO)));

	center[0] = vec3_add(trebuchet->position, vec3(0.0f, trebuchet->baseSize.y * 0.5f, 0.0f));
	half[0] = vec3_scale(trebuchet->baseSize, 0.5f);
	rotation[0] = quat_identity();
	leg_box(vec3_add(trebuchet->position, vec3(-spread, trebuchet->baseSize.y, 0.0f)), pivot, thickness, &center[1], &half[1], &rotation[1]);
	leg_box(vec3_add(trebuchet->position, vec3(spread, trebuchet->baseSize.y, 0.0f)), pivot, thickness, &center[2], &half[2], &rotation[2]);
	center[3] = vec3_add(pivot, vec3_scale(along, trebuchet->armLength * (TREB_PIVOT_RATIO - 0.5f)));
	half[3] = vec3(trebuchet->armLength * 0.5f, trebuchet->armThickness * 0.5f, trebuchet->armThickness * 0.5f);
	rotation[3] = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(trebuchet->armAngle));
	center[4] = vec3_sub(short_end, vec3(0.0f, trebuchet->counterweightSize.y * 0.5f, 0.0f));
	half[4] = vec3_scale(trebuchet->counterweightSize, 0.5f);
	rotation[4] = quat_identity();
}

Vec3	trebuchet_spawn_point(const Trebuchet *trebuchet)
{
	return (vec3_add(trebuchet->launchPoint, vec3_scale(trebuchet_direction(trebuchet), trebuchet->spawnDistance)));
}

static RigidBody	static_box(const Trebuchet *trebuchet, Vec3 center, Vec3 half_extents, Quat orientation)
{
	RigidBody	body = rigidbody_make();

	body.kind = KIND_TREBUCHET;
	body.position = center;
	body.orientation = orientation;
	body.collider = collider_box(half_extents);
	body.friction = trebuchet->friction;
	body.elasticity = trebuchet->elasticity;
	body.color = trebuchet->color;
	rigidbody_make_static(&body);
	return (body);
}

void	trebuchet_build(Trebuchet *trebuchet, World *world)
{
	Vec3	center[TREB_PARTS];
	Vec3	half[TREB_PARTS];
	Quat	rotation[TREB_PARTS];
	int		part;

	trebuchet_parts(trebuchet, center, half, rotation);
	part = 0;
	while (part < TREB_PARTS)
	{
		world_add_body(world, static_box(trebuchet, center[part], half[part], rotation[part]));
		part++;
	}
	trebuchet->launchPoint = trebuchet_launch_point(trebuchet);
	trebuchet->spawnDistance = trebuchet_spawn_distance(trebuchet);
}

// An apple with the current launch settings, unless something occupies the spawn point.
int	trebuchet_fire(const Trebuchet *trebuchet, const ObjectDef *apple, World *world)
{
	Vec3		velocity = vec3_scale(trebuchet_direction(trebuchet), trebuchet->launchSpeed);   // [F25]
	RigidBody	shot = projectile_make_apple(apple, trebuchet_spawn_point(trebuchet), velocity, trebuchet->projectileMass);

	if (overlap_find(world, &shot) >= 0)
	{
		printf("Launch point blocked: an apple would start inside another body\n");
		return (-1);
	}
	return (world_add_body(world, shot));
}
