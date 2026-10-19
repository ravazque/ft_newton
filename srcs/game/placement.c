#include "newton.h"

// New bodies never start inside anything: a group whose spot is taken is lifted as one until it lands on what is there.

static float	needed_lift(const World *world, const RigidBody *body)
{
	Vec3	other_min;
	Vec3	other_max;
	Vec3	body_min;
	Vec3	body_max;
	int		hit;

	hit = overlap_find(world, body);
	if (hit < 0)
		return (0.0f);
	collider_bounds(&world->bodies[hit], &other_min, &other_max);
	collider_bounds(body, &body_min, &body_max);
	return (other_max.y - body_min.y + SPAWN_CLEARANCE);
}

static float	lift_until_free(const World *world, RigidBody *bodies, int count)
{
	float	lift = 0.0f;
	float	needed = 1.0f;
	int		rounds;
	int		i;

	rounds = 0;
	while (needed > 0.0f && rounds <= world->bodyCount)
	{
		needed = 0.0f;
		i = 0;
		while (i < count)
		{
			needed = fmaxf(needed, needed_lift(world, &bodies[i]));
			i++;
		}
		i = 0;
		while (i < count)
		{
			bodies[i].position.y += needed;
			i++;
		}
		lift += needed;
		rounds++;
	}
	return (lift);
}

// Adds the bodies in order after the last one; returns how far they were lifted (m), -1 when one could not be added.
float	placement_add(World *world, RigidBody *bodies, int count)
{
	float	lift = lift_until_free(world, bodies, count);
	int		i;

	i = 0;
	while (i < count)
	{
		if (world_add_body(world, bodies[i]) < 0)
			return (-1.0f);
		i++;
	}
	return (lift);
}
