#include "newton.h"

/*
 * How far past the arm tip an apple must appear so it never starts inside the
 * machine. A ray along the firing line is tested against every box grown by the
 * apple's radius, at every launch angle the controls allow; the worst case gives
 * ONE distance, so the spawn point and the aim bar agree whatever the angle.
*/

/* One slab of the ray / box test: narrows [enter, leave] to where the ray is inside on this axis. */
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

/* Distance along 'direction' at which the ray leaves the box grown by 'radius', 0 if it never enters. */
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

float	trebuchet_spawn_distance(const Trebuchet *t)
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
