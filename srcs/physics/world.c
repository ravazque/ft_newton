
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
	w->prevContactCount = 0;
}

/* Used when the rules change under resting bodies (gravity edits, reset). */
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

static int	island_root(int *parent, int i)
{
	while (parent[i] != i)
	{
		parent[i] = parent[parent[i]];
		i = parent[i];
	}
	return (i);
}

/* The per-body scratch arrays (islands, positional correction) follow the
 * body count; they are only reallocated when it grows. */
int	world_grow_scratch(World *w)
{
	int		*parent;
	float	*timer;
	Vec3	*positions;
	Vec3	*rotations;

	if (w->bodyCount <= w->scratchCapacity)
		return (1);
	parent = realloc(w->islandParent, (size_t)w->bodyCount * sizeof(int));
	timer = realloc(w->islandTimer, (size_t)w->bodyCount * sizeof(float));
	positions = realloc(w->startPositions, (size_t)w->bodyCount * sizeof(Vec3));
	rotations = realloc(w->rotationDelta, (size_t)w->bodyCount * sizeof(Vec3));
	if (parent)
		w->islandParent = parent;
	if (timer)
		w->islandTimer = timer;
	if (positions)
		w->startPositions = positions;
	if (rotations)
		w->rotationDelta = rotations;
	if (!parent || !timer || !positions || !rotations)
		return (0);
	w->scratchCapacity = w->bodyCount;
	return (1);
}

/*
 * Sleeping is decided per island: the set of dynamic bodies linked by
 * contacts (static bodies do not link anything). Every awake dynamic body
 * accumulates how long it has stayed below both speed thresholds; an island
 * goes to sleep only when ALL of its bodies have been still for SLEEP_TIME,
 * so a stack switches off as a whole. Sleeping one box at a time would drop
 * its contacts while its neighbours still move and kick the pile every time
 * they wake it again. Asleep bodies keep zero velocity and skip integration:
 * that is the "stable state" the impact must return to.
*/
static void	update_sleep(World *w, float dt)
{
	RigidBody	*b;
	int			root;
	int			i;

	if (!world_grow_scratch(w))
		return ;
	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		w->islandParent[i] = i;
		w->islandTimer[i] = SLEEP_TIME;
		if (b->invMass > 0.0f && b->awake)
		{
			if (vec3_length_sq(b->velocity) < SLEEP_LINEAR_EPS * SLEEP_LINEAR_EPS && vec3_length_sq(b->angularVelocity) < SLEEP_ANGULAR_EPS * SLEEP_ANGULAR_EPS)   /* [F24] */
				b->sleepTimer += dt;
			else
				b->sleepTimer = 0.0f;
		}
		i++;
	}
	i = 0;
	while (i < w->contactCount)
	{
		if (w->bodies[w->contacts[i].a].invMass > 0.0f && w->bodies[w->contacts[i].b].invMass > 0.0f)
			w->islandParent[island_root(w->islandParent, w->contacts[i].a)] = island_root(w->islandParent, w->contacts[i].b);
		i++;
	}
	i = 0;
	while (i < w->bodyCount)
	{
		if (w->bodies[i].invMass > 0.0f && w->bodies[i].awake)
		{
			root = island_root(w->islandParent, i);
			w->islandTimer[root] = fminf(w->islandTimer[root], w->bodies[i].sleepTimer);
		}
		i++;
	}
	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		if (b->invMass > 0.0f && b->awake && w->islandTimer[island_root(w->islandParent, i)] >= SLEEP_TIME)
		{
			b->awake = 0;
			b->velocity = vec3(0.0f, 0.0f, 0.0f);
			b->angularVelocity = vec3(0.0f, 0.0f, 0.0f);
		}
		i++;
	}
}

/* Dynamic bodies that flew out of the playable area are dropped so the object
 * count (and the broad-phase) only track what can still interact. */
static void	cull_far_bodies(World *w)
{
	int	i;

	i = 0;
	while (i < w->bodyCount)
	{
		if (w->bodies[i].invMass > 0.0f && vec3_length_sq(w->bodies[i].position) > WORLD_CULL_DISTANCE * WORLD_CULL_DISTANCE)
			world_remove_body(w, i);
		else
			i++;
	}
}

/*
 * One fixed simulation step:
 *   1) integration - every awake body: forces + gravity -> velocity ->
 *      position / orientation.
 *   2) detection - broadphase fills w->pairs with candidate pairs,
 *      narrowphase turns them into w->contacts (normal/point/penetration).
 *   3) response - resolver applies impulses (with restitution and friction)
 *      and pushes overlapping bodies apart.
 *   4) housekeeping - sleeping and culling.
 * A non-positive dt (pause, time scale 0) leaves the world untouched.
 */
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
	update_sleep(w, dt);
	cull_far_bodies(w);
}
