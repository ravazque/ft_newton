#include "newton.h"

/*
 * Sleeping, decided per contact island (dynamic bodies linked by contacts; static
 * bodies link nothing). An island sleeps only when ALL its bodies stayed below both
 * speed thresholds for SLEEP_TIME [F24]: sleeping one box of a pile alone would drop
 * the support of its neighbours and kick the pile every time they wake it again.
*/

static int	island_root(int *parent, int i)
{
	while (parent[i] != i)
	{
		parent[i] = parent[parent[i]];
		i = parent[i];
	}
	return (i);
}

static int	is_dynamic_awake(const RigidBody *b)
{
	return (b->invMass > 0.0f && b->awake);
}

/* Each awake dynamic body accumulates how long it has been still. */
static void	accumulate_rest(World *w, float dt)
{
	RigidBody	*b;
	float		lin;
	float		ang;
	int			i;

	lin = SLEEP_LINEAR_EPS * SLEEP_LINEAR_EPS;
	ang = SLEEP_ANGULAR_EPS * SLEEP_ANGULAR_EPS;
	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		w->islandParent[i] = i;
		w->islandTimer[i] = SLEEP_TIME;
		if (is_dynamic_awake(b) && vec3_length_sq(b->velocity) < lin && vec3_length_sq(b->angularVelocity) < ang)   /* [F24] */
			b->sleepTimer += dt;
		else if (is_dynamic_awake(b))
			b->sleepTimer = 0.0f;
		i++;
	}
}

/* Union-find over the contacts, then each island keeps its least rested body's time. */
static void	build_islands(World *w)
{
	const Contact	*c;
	int				root;
	int				i;

	i = 0;
	while (i < w->contactCount)
	{
		c = &w->contacts[i];
		if (w->bodies[c->a].invMass > 0.0f && w->bodies[c->b].invMass > 0.0f)
			w->islandParent[island_root(w->islandParent, c->a)] = island_root(w->islandParent, c->b);
		i++;
	}
	i = 0;
	while (i < w->bodyCount)
	{
		if (is_dynamic_awake(&w->bodies[i]))
		{
			root = island_root(w->islandParent, i);
			w->islandTimer[root] = fminf(w->islandTimer[root], w->bodies[i].sleepTimer);
		}
		i++;
	}
}

void	sleep_update(World *w, float dt)
{
	RigidBody	*b;
	int			i;

	if (!world_grow_scratch(w))
		return ;
	accumulate_rest(w, dt);
	build_islands(w);
	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		if (is_dynamic_awake(b) && w->islandTimer[island_root(w->islandParent, i)] >= SLEEP_TIME)
		{
			b->awake = 0;
			b->velocity = vec3(0.0f, 0.0f, 0.0f);
			b->angularVelocity = vec3(0.0f, 0.0f, 0.0f);
		}
		i++;
	}
}
