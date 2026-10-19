#include "newton.h"

// An island (bodies linked by contacts or joints, found by union-find) sleeps when all its bodies stayed still long enough.

static int	island_root(int *parent, int index)
{
	while (parent[index] != index)
	{
		parent[index] = parent[parent[index]];
		index = parent[index];
	}
	return (index);
}

static int	is_dynamic_awake(const RigidBody *body)
{
	return (body->inverseMass > 0.0f && body->awake);
}

static float	turned_since(Quat from, Quat to)
{
	Quat	change = quat_mul(to, quat_conjugate(from));

	return (2.0f * acosf(fminf(1.0f, fabsf(change.w))));
}

// Still: slow, or shaking in place: little net move, and past the first half of SLEEP_TIME a path SLEEP_JITTER_RATIO times longer.
static int	at_rest(RigidBody *body, float dt)
{
	float	allowance;
	float	moved;
	float	turned;
	int		settling;
	int		linear_still;
	int		angular_still;

	if (body->sleepTimer == 0.0f || body->restWindow >= 2.0f * SLEEP_TIME)
	{
		body->restPosition = body->position;
		body->restOrientation = body->orientation;
		body->restPath = 0.0f;
		body->restTurnPath = 0.0f;
		body->restWindow = 0.0f;
	}
	body->restWindow += dt;
	body->restPath += vec3_length(body->velocity) * dt;
	body->restTurnPath += vec3_length(body->angularVelocity) * dt;
	allowance = fmaxf(body->restWindow, SLEEP_TIME);
	moved = vec3_length(vec3_sub(body->position, body->restPosition));
	turned = turned_since(body->restOrientation, body->orientation);
	settling = (body->restWindow < 0.5f * SLEEP_TIME);
	body->creepTimer += dt;
	if (moved >= SLEEP_CREEP_FACTOR * SLEEP_LINEAR_EPS * allowance || turned >= SLEEP_CREEP_FACTOR * SLEEP_ANGULAR_EPS * allowance)
		body->creepTimer = 0.0f;
	linear_still = (vec3_length(body->velocity) < SLEEP_LINEAR_EPS || (moved < SLEEP_LINEAR_EPS * allowance && (settling || body->restPath > SLEEP_JITTER_RATIO * moved)));
	angular_still = (vec3_length(body->angularVelocity) < SLEEP_ANGULAR_EPS || (turned < SLEEP_ANGULAR_EPS * allowance && (settling || body->restTurnPath > SLEEP_JITTER_RATIO * turned)));
	return (linear_still && angular_still);
}

static void	accumulate_rest(World *world, float dt)
{
	RigidBody	*body;
	int			i;

	i = 0;
	while (i < world->bodyCount)
	{
		body = &world->bodies[i];
		world->islandParent[i] = i;
		world->islandTimer[i] = SLEEP_TIME;
		if (is_dynamic_awake(body) && at_rest(body, dt))   // [F24]
			body->sleepTimer += dt;
		else if (is_dynamic_awake(body))
			body->sleepTimer = 0.0f;
		i++;
	}
}

static void	join_islands(World *world, int body_a, int body_b)
{
	if (world->bodies[body_a].inverseMass > 0.0f && world->bodies[body_b].inverseMass > 0.0f)
		world->islandParent[island_root(world->islandParent, body_a)] = island_root(world->islandParent, body_b);
}

static void	build_islands(World *world)
{
	int	root;
	int	i;

	i = 0;
	while (i < world->contactCount)
	{
		join_islands(world, world->contacts[i].bodyA, world->contacts[i].bodyB);
		i++;
	}
	i = 0;
	while (i < world->jointCount)
	{
		join_islands(world, world->joints[i].bodyA, world->joints[i].bodyB);
		i++;
	}
	i = 0;
	while (i < world->bodyCount)
	{
		if (is_dynamic_awake(&world->bodies[i]))
		{
			root = island_root(world->islandParent, i);
			world->islandTimer[root] = fminf(world->islandTimer[root], fmaxf(world->bodies[i].sleepTimer, world->bodies[i].creepTimer * SLEEP_TIME / SLEEP_CREEP_TIME));
		}
		i++;
	}
}

void	sleep_update(World *world, float dt)
{
	RigidBody	*body;
	int			i;

	if (!world_grow_scratch(world))
		return ;
	accumulate_rest(world, dt);
	build_islands(world);
	i = 0;
	while (i < world->bodyCount)
	{
		body = &world->bodies[i];
		if (is_dynamic_awake(body) && world->islandTimer[island_root(world->islandParent, i)] >= SLEEP_TIME)
		{
			body->awake = 0;
			body->velocity = vec3(0.0f, 0.0f, 0.0f);
			body->angularVelocity = vec3(0.0f, 0.0f, 0.0f);
		}
		i++;
	}
}
