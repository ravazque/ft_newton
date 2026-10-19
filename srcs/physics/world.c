#include "newton.h"

// The world owns every body and joint; one step runs integrate, broad-phase, narrow-phase, response, sleep and cull.

void	world_init(World *world)
{
	memset(world, 0, sizeof(*world));
	world->gravity = vec3(0.0f, GRAVITY_Y, 0.0f);
	world->solverIterations = SOLVER_ITERATIONS;
}

void	world_destroy(World *world)
{
	free(world->bodies);
	free(world->joints);
	free(world->pairs);
	free(world->contacts);
	free(world->previousContacts);
	free(world->islandParent);
	free(world->islandTimer);
	free(world->startPositions);
	free(world->rotationDelta);
	memset(world, 0, sizeof(*world));
}

int	world_add_body(World *world, RigidBody body)
{
	RigidBody	*grown;
	int			new_capacity;

	if (world->bodyCount == world->bodyCapacity)
	{
		new_capacity = 16;
		if (world->bodyCapacity > 0)
			new_capacity = world->bodyCapacity * 2;
		grown = realloc(world->bodies, (size_t)new_capacity * sizeof(RigidBody));
		if (!grown)
			return (-1);
		world->bodies = grown;
		world->bodyCapacity = new_capacity;
	}
	world->bodies[world->bodyCount] = body;
	return (world->bodyCount++);
}

static void	forget_contacts(World *world, int removed, int moved)
{
	const Contact	*contact;
	int				kept;
	int				i;

	kept = 0;
	i = 0;
	while (i < world->previousContactCount)
	{
		contact = &world->previousContacts[i];
		if (contact->bodyA != removed && contact->bodyB != removed && contact->bodyA != moved && contact->bodyB != moved)
			world->previousContacts[kept++] = *contact;
		i++;
	}
	world->previousContactCount = kept;
}

// Swap-with-last removal: the remembered contacts and the joints keyed by the two indices follow.
void	world_remove_body(World *world, int index)
{
	int	last;

	if (index < 0 || index >= world->bodyCount)
		return ;
	last = world->bodyCount - 1;
	world->bodies[index] = world->bodies[last];
	world->bodyCount--;
	forget_contacts(world, index, last);
	joint_forget_body(world, index, last);
}

void	world_clear(World *world)
{
	world->bodyCount = 0;
	world->jointCount = 0;
	world->ragdollCount = 0;
	world->previousContactCount = 0;
}

void	world_wake_all(World *world)
{
	int	i;

	i = 0;
	while (i < world->bodyCount)
	{
		world->bodies[i].awake = 1;
		world->bodies[i].sleepTimer = 0.0f;
		world->bodies[i].creepTimer = 0.0f;
		i++;
	}
}

int	world_grow_scratch(World *world)
{
	int		*parent;
	float	*timer;
	Vec3	*positions;
	Vec3	*rotations;

	if (world->bodyCount <= world->scratchCapacity)
		return (1);
	parent = realloc(world->islandParent, (size_t)world->bodyCount * sizeof(int));
	if (parent)
		world->islandParent = parent;
	timer = realloc(world->islandTimer, (size_t)world->bodyCount * sizeof(float));
	if (timer)
		world->islandTimer = timer;
	positions = realloc(world->startPositions, (size_t)world->bodyCount * sizeof(Vec3));
	if (positions)
		world->startPositions = positions;
	rotations = realloc(world->rotationDelta, (size_t)world->bodyCount * sizeof(Vec3));
	if (rotations)
		world->rotationDelta = rotations;
	if (!parent || !timer || !positions || !rotations)
		return (0);
	world->scratchCapacity = world->bodyCount;
	return (1);
}

void	world_step(World *world, float dt)
{
	int	i;

	if (dt <= 0.0f)
		return ;
	world->dt = dt;
	i = 0;
	while (i < world->bodyCount)
	{
		if (world->bodies[i].awake)
		{
			integrator_integrate(&world->bodies[i], world->gravity, dt);
			rigidbody_clear_accumulators(&world->bodies[i]);
		}
		i++;
	}
	broadphase_compute_pairs(world);
	narrowphase_generate_contacts(world);
	resolver_resolve(world);
	sleep_update(world, dt);
	cull_lost_bodies(world);
}
