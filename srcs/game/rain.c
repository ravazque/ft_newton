#include "newton.h"

// The test rain: a square of small balls dropped on a spot, landing on the real surface of whatever stands there.

static int	reaches_over(Vec3 bounds_min, Vec3 bounds_max, Vec3 center, float half_width)
{
	if (bounds_min.x > center.x + half_width || bounds_max.x < center.x - half_width)
		return (0);
	return (bounds_min.z <= center.z + half_width && bounds_max.z >= center.z - half_width);
}

static float	highest_top_under(const World *world, Vec3 center, float half_width)
{
	float	top = center.y;
	Vec3	bounds_min;
	Vec3	bounds_max;
	int		i;

	i = 0;
	while (i < world->bodyCount)
	{
		collider_bounds(&world->bodies[i], &bounds_min, &bounds_max);
		if (reaches_over(bounds_min, bounds_max, center, half_width))
			top = fmaxf(top, bounds_max.y);
		i++;
	}
	return (top);
}

static RigidBody	make_drop(const ObjectDef *apple, Vec3 position)
{
	RigidBody	drop = rigidbody_make();

	drop.kind = KIND_RAIN;
	drop.position = position;
	drop.collider = collider_sphere(RAIN_RADIUS);
	drop.friction = apple->friction;
	drop.elasticity = apple->elasticity;
	drop.rollingResistance = apple->rollingResistance;
	drop.color = vec3(RAIN_R, RAIN_G, RAIN_B);
	rigidbody_set_mass(&drop, RAIN_MASS);
	return (drop);
}

// The drops take the apples' material; returns 0, or -1 on failure.
int	rain_drop(World *world, const ObjectDef *apple, Vec3 center)
{
	RigidBody	*drops = malloc(RAIN_GRID * RAIN_GRID * sizeof(RigidBody));
	float		half_width = (float)(RAIN_GRID - 1) * RAIN_SPACING * 0.5f;
	float		height = highest_top_under(world, center, half_width + RAIN_RADIUS) + RAIN_DROP;
	float		lift;
	Vec3		position;
	int			column;
	int			row;

	if (!drops)
		return (-1);
	row = 0;
	while (row < RAIN_GRID)
	{
		column = 0;
		while (column < RAIN_GRID)
		{
			position = vec3(center.x - half_width + (float)column * RAIN_SPACING, height, center.z - half_width + (float)row * RAIN_SPACING);
			drops[row * RAIN_GRID + column] = make_drop(apple, position);
			column++;
		}
		row++;
	}
	lift = placement_add(world, drops, RAIN_GRID * RAIN_GRID);
	free(drops);
	if (lift < 0.0f)
		return (-1);
	return (0);
}
