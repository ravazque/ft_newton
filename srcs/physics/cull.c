#include "newton.h"

// Dynamic bodies that flew too far away, or fell too far under a plane past its edge, leave the world.

static void	cull_far(World *world)
{
	float	limit;
	int		i;

	limit = WORLD_CULL_DISTANCE * WORLD_CULL_DISTANCE;
	i = 0;
	while (i < world->bodyCount)
	{
		if (world->bodies[i].inverseMass > 0.0f && vec3_length_sq(world->bodies[i].position) > limit)
			world_remove_body(world, i);
		else
			i++;
	}
}

// The plane is a copy: a removal may move the plane's own body to another index.
static void	cull_below(World *world, Collider plane)
{
	int	i;

	i = 0;
	while (i < world->bodyCount)
	{
		if (world->bodies[i].inverseMass > 0.0f && collider_plane_distance(&plane, world->bodies[i].position) < -FALL_CULL_DEPTH)
			world_remove_body(world, i);
		else
			i++;
	}
}

void	cull_lost_bodies(World *world)
{
	int	i;

	cull_far(world);
	i = 0;
	while (i < world->bodyCount)
	{
		if (world->bodies[i].collider.type == SHAPE_PLANE)
			cull_below(world, world->bodies[i].collider);
		i++;
	}
}
