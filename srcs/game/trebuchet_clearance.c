#include "newton.h"

// How far past the arm tip an apple appears: the farthest exit of the firing ray from the machine, over every angle.

// One slab of the ray and box test (box grown by the apple radius): narrows [enter, leave] along this axis.
static void	clip_slab(float origin, float direction, float extent, float *enter, float *leave)
{
	float	near_distance;
	float	far_distance;
	float	swap;

	if (fabsf(direction) < 1e-6f)
	{
		if (fabsf(origin) > extent)
			*leave = -1.0f;
		return ;
	}
	near_distance = (-extent - origin) / direction;
	far_distance = (extent - origin) / direction;
	if (near_distance > far_distance)
	{
		swap = near_distance;
		near_distance = far_distance;
		far_distance = swap;
	}
	*enter = fmaxf(*enter, near_distance);
	*leave = fminf(*leave, far_distance);
}

static float	box_exit_distance(Vec3 origin, Vec3 direction, Vec3 center, Vec3 half, Quat rotation, float radius)
{
	Quat	inverse = quat_conjugate(rotation);
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

static float	clearance_at(const Trebuchet *trebuchet, float angle, const Vec3 *center, const Vec3 *half, const Quat *rotation)
{
	Vec3	direction = vec3(cosf(DEG2RAD(angle)), sinf(DEG2RAD(angle)), 0.0f);
	float	worst = 0.0f;
	int		part;

	part = 0;
	while (part < TREB_PARTS)
	{
		worst = fmaxf(worst, box_exit_distance(trebuchet->launchPoint, direction, center[part], half[part], rotation[part], trebuchet->projectileRadius));
		part++;
	}
	return (worst);
}

float	trebuchet_spawn_distance(const Trebuchet *trebuchet)
{
	Vec3	center[TREB_PARTS];
	Vec3	half[TREB_PARTS];
	Quat	rotation[TREB_PARTS];
	float	worst = 0.0f;
	float	angle = LAUNCH_ANGLE_MIN;

	trebuchet_parts(trebuchet, center, half, rotation);
	while (angle <= LAUNCH_ANGLE_MAX)
	{
		worst = fmaxf(worst, clearance_at(trebuchet, angle, center, half, rotation));
		angle += TREB_ANGLE_STEP;
	}
	return (worst + LAUNCH_CLEARANCE);
}
