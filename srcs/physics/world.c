
#include "newton.h"

void	world_init(World *w)
{
	memset(w, 0, sizeof(*w));
	w->gravity = vec3(0.0f, GRAVITY_Y, 0.0f);
	w->solverIterations = SOLVER_ITERATIONS;
}

void	world_destroy(World *w)
{
	free(w->bodies);
	free(w->pairs);
	free(w->contacts);
	memset(w, 0, sizeof(*w));
}

int	world_add_body(World *w, RigidBody body)
{
	RigidBody	*grown;
	int			new_capacity;

	if (w->bodyCount == w->bodyCapacity)
	{
		if (w->bodyCapacity == 0)
			new_capacity = 16;
		else
			new_capacity = w->bodyCapacity * 2;
		grown = realloc(w->bodies, (size_t)new_capacity * sizeof(RigidBody));
		if (!grown)
			return (-1);
		w->bodies = grown;
		w->bodyCapacity = new_capacity;
	}
	w->bodies[w->bodyCount] = body;
	return (w->bodyCount++);
}

/* Swap-with-last removal: O(1), but the body that was last changes index. */
void	world_remove_body(World *w, int handle)
{
	if (handle < 0 || handle >= w->bodyCount)
		return ;
	w->bodies[handle] = w->bodies[w->bodyCount - 1];
	w->bodyCount--;
}

void	world_clear(World *w)
{
	w->bodyCount = 0;
}

/*
 * One fixed simulation step, three stages:
 *   1) [TODO] integration — advance every awake body: forces + gravity ->
 *      velocity -> position/orientation. integrator_integrate is yours.
 *   2) [DONE] detection — broadphase fills w->pairs with candidate pairs,
 *      narrowphase turns them into w->contacts (normal/point/penetration).
 *   3) [TODO] response — resolver_resolve consumes w->contacts (impulses,
 *      restitution, positional correction). Until you implement it, falling
 *      bodies pass through each other even though contacts are reported.
 */
void	world_step(World *w, float dt)
{
	int	i;

	i = 0;
	while (i < w->bodyCount)
	{
		if (w->bodies[i].awake)
		{
			integrator_integrate(&w->bodies[i], w->gravity, dt);
			rb_clear_accumulators(&w->bodies[i]);
		}
		i++;
	}
	broadphase_compute_pairs(w);
	narrowphase_generate_contacts(w);
	resolver_resolve(w, dt);
}
