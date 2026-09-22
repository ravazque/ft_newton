#include "newton.h"

/*
 * The world owns every body and advances them one fixed step at a time:
 * integrate -> broad-phase -> narrow-phase -> contact response -> sleep -> cull.
 * The per-step pipeline lives in srcs/collision, srcs/response and sleep.c / cull.c.
*/

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
	free(w->prevContacts);
	free(w->islandParent);
	free(w->islandTimer);
	free(w->startPositions);
	free(w->rotationDelta);
	memset(w, 0, sizeof(*w));
}

int	world_add_body(World *w, RigidBody body)
{
	RigidBody	*grown;
	int			new_capacity;

	if (w->bodyCount == w->bodyCapacity)
	{
		new_capacity = 16;
		if (w->bodyCapacity > 0)
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

/* Warm-start memory is keyed by body index: contacts of the removed body and
 * of the one that takes its slot would otherwise feed their impulses to the wrong pair. */
static void	forget_contacts(World *w, int removed, int moved)
{
	const Contact	*c;
	int				kept;
	int				i;

	kept = 0;
	i = 0;
	while (i < w->prevContactCount)
	{
		c = &w->prevContacts[i];
		if (c->a != removed && c->b != removed && c->a != moved && c->b != moved)
			w->prevContacts[kept++] = *c;
		i++;
	}
	w->prevContactCount = kept;
}

/* O(1) swap-with-last removal: the last body takes the freed index. */
void	world_remove_body(World *w, int handle)
{
	int	last;

	if (handle < 0 || handle >= w->bodyCount)
		return ;
	last = w->bodyCount - 1;
	w->bodies[handle] = w->bodies[last];
	w->bodyCount--;
	forget_contacts(w, handle, last);
}

void	world_clear(World *w)
{
	w->bodyCount = 0;
	w->prevContactCount = 0;
}

/* Used when the rules change under resting bodies (gravity, materials, reset). */
void	world_wake_all(World *w)
{
	int	i;

	i = 0;
	while (i < w->bodyCount)
	{
		w->bodies[i].awake = 1;
		w->bodies[i].sleepTimer = 0.0f;
		i++;
	}
}

/* The per-body scratch arrays only grow, and only when the body count does. */
int	world_grow_scratch(World *w)
{
	int		*parent;
	float	*timer;
	Vec3	*positions;
	Vec3	*rotations;

	if (w->bodyCount <= w->scratchCapacity)
		return (1);
	parent = realloc(w->islandParent, (size_t)w->bodyCount * sizeof(int));
	if (parent)
		w->islandParent = parent;
	timer = realloc(w->islandTimer, (size_t)w->bodyCount * sizeof(float));
	if (timer)
		w->islandTimer = timer;
	positions = realloc(w->startPositions, (size_t)w->bodyCount * sizeof(Vec3));
	if (positions)
		w->startPositions = positions;
	rotations = realloc(w->rotationDelta, (size_t)w->bodyCount * sizeof(Vec3));
	if (rotations)
		w->rotationDelta = rotations;
	if (!parent || !timer || !positions || !rotations)
		return (0);
	w->scratchCapacity = w->bodyCount;
	return (1);
}

/* One fixed step. A non-positive dt (pause, time scale 0) leaves the world untouched. */
void	world_step(World *w, float dt)
{
	int	i;

	if (dt <= 0.0f)
		return ;
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
	resolver_resolve(w);
	sleep_update(w, dt);
	cull_lost_bodies(w);
}
