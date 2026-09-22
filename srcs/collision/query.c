#include "newton.h"

/*
 * Collision queries outside the simulation step, used to place new bodies: an
 * apple is not fired into something, and a structure is lifted until none of
 * its blocks would sink into what already occupies its spot.
*/

int	bodies_in_contact(const RigidBody *a, const RigidBody *b)
{
	Contact	buf[MAX_CONTACTS_PER_PAIR];

	return (narrowphase_pair(a, b, buf) > 0);
}

/* Touching is allowed; sinking deeper than SPAWN_TOLERANCE is not. */
int	bodies_overlap(const RigidBody *a, const RigidBody *b)
{
	Contact	buf[MAX_CONTACTS_PER_PAIR];
	int		count;
	int		i;

	count = narrowphase_pair(a, b, buf);
	i = 0;
	while (i < count)
	{
		if (buf[i].penetration > SPAWN_TOLERANCE)
			return (1);
		i++;
	}
	return (0);
}

static int	bounds_overlap(Vec3 amn, Vec3 amx, Vec3 bmn, Vec3 bmx)
{
	return (amn.x <= bmx.x && bmn.x <= amx.x && amn.y <= bmx.y && bmn.y <= amx.y && amn.z <= bmx.z && bmn.z <= amx.z);   /* [F11] */
}

/* Index of a body the candidate would sink into, or -1 when its spot is free. */
int	world_first_overlap(const World *w, const RigidBody *candidate)
{
	Vec3	mn;
	Vec3	mx;
	Vec3	other_mn;
	Vec3	other_mx;
	int		i;

	collider_bounds(candidate, &mn, &mx);
	i = 0;
	while (i < w->bodyCount)
	{
		collider_bounds(&w->bodies[i], &other_mn, &other_mx);
		if (bounds_overlap(mn, mx, other_mn, other_mx) && bodies_overlap(candidate, &w->bodies[i]))
			return (i);
		i++;
	}
	return (-1);
}
