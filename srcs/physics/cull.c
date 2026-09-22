#include "newton.h"

/*
 * Removes the dynamic bodies that can no longer interact: those that flew
 * WORLD_CULL_DISTANCE away and those that fell FALL_CULL_DEPTH below a plane
 * (only possible past its edge, since the plane holds up what is above it).
 * They stop costing broad-phase work and leave the object counter.
*/

static void	cull_far(World *w)
{
	float	limit;
	int		i;

	limit = WORLD_CULL_DISTANCE * WORLD_CULL_DISTANCE;
	i = 0;
	while (i < w->bodyCount)
	{
		if (w->bodies[i].invMass > 0.0f && vec3_length_sq(w->bodies[i].position) > limit)
			world_remove_body(w, i);
		else
			i++;
	}
}

/* The plane is copied: a removal may move the plane's own body to another slot. */
static void	cull_below(World *w, Collider plane)
{
	int	i;

	i = 0;
	while (i < w->bodyCount)
	{
		if (w->bodies[i].invMass > 0.0f && collider_plane_distance(&plane, w->bodies[i].position) < -FALL_CULL_DEPTH)
			world_remove_body(w, i);
		else
			i++;
	}
}

void	cull_lost_bodies(World *w)
{
	int	i;

	cull_far(w);
	i = 0;
	while (i < w->bodyCount)
	{
		if (w->bodies[i].collider.type == SHAPE_PLANE)
			cull_below(w, w->bodies[i].collider);
		i++;
	}
}
