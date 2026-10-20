#include "newton.h"

// One step of response: wake, prepare joints and contacts, warm start, Gauss-Seidel passes, positional correction, joint projection, remember.

static void	wake(RigidBody *body)
{
	body->awake = 1;
	body->sleepTimer = 0.0f;
	body->creepTimer = 0.0f;
}

static void	wake_pair(RigidBody *body_a, RigidBody *body_b)
{
	if (!body_a->awake && body_b->awake && body_b->inverseMass > 0.0f)
		wake(body_a);
	if (!body_b->awake && body_a->awake && body_a->inverseMass > 0.0f)
		wake(body_b);
}

static void	wake_touching(World *world)
{
	int	i;

	i = 0;
	while (i < world->contactCount)
	{
		wake_pair(&world->bodies[world->contacts[i].bodyA], &world->bodies[world->contacts[i].bodyB]);
		i++;
	}
	i = 0;
	while (i < world->jointCount)
	{
		wake_pair(&world->bodies[world->joints[i].bodyA], &world->bodies[world->joints[i].bodyB]);
		i++;
	}
}

// Last step's contact of the same pair at nearly the same point; a resting scene keeps the order, so the scan resumes there.
static const Contact	*find_previous(const World *world, const Contact *contact, int *cursor)
{
	const Contact	*previous;
	int				count = world->previousContactCount;
	int				offset;
	int				i;

	offset = 0;
	while (offset < count)
	{
		i = (*cursor + offset) % count;
		previous = &world->previousContacts[i];
		if (previous->bodyA == contact->bodyA && previous->bodyB == contact->bodyB && vec3_length_sq(vec3_sub(previous->point, contact->point)) <= WARM_START_RADIUS * WARM_START_RADIUS)
		{
			*cursor = (i + 1) % count;
			return (previous);
		}
		offset++;
	}
	return (NULL);
}

static void	warm_start(World *world, Contact *contact, int *cursor)
{
	const Contact	*previous = find_previous(world, contact, cursor);
	Vec3			impulse;

	if (!previous)
		return ;
	contact->impulseNormal = previous->impulseNormal;
	contact->impulseTangent1 = previous->impulseTangent1;
	contact->impulseTangent2 = previous->impulseTangent2;
	impulse = vec3_add(vec3_scale(contact->normal, contact->impulseNormal), vec3_add(vec3_scale(contact->tangent1, contact->impulseTangent1), vec3_scale(contact->tangent2, contact->impulseTangent2)));
	impulse_apply(&world->bodies[contact->bodyA], &world->bodies[contact->bodyB], contact, impulse);
}

static void	remember_contacts(World *world)
{
	Contact	*grown;

	if (world->contactCount > world->previousContactCapacity)
	{
		grown = realloc(world->previousContacts, (size_t)world->contactCount * sizeof(Contact));
		if (!grown)
		{
			world->previousContactCount = 0;
			return ;
		}
		world->previousContacts = grown;
		world->previousContactCapacity = world->contactCount;
	}
	if (world->contactCount > 0)
		memcpy(world->previousContacts, world->contacts, (size_t)world->contactCount * sizeof(Contact));
	world->previousContactCount = world->contactCount;
}

static void	solve_joints(World *world)
{
	int	i;

	i = 0;
	while (i < world->jointCount)
	{
		joint_point_solve(world, &world->joints[i]);
		joint_limit_solve(world, &world->joints[i]);
		i++;
	}
}

static void	project_joints(World *world)
{
	int	sweep;
	int	i;

	sweep = 0;
	while (sweep < JOINT_PROJECTION_SWEEPS)
	{
		i = 0;
		while (i < world->jointCount)
		{
			joint_point_project(world, &world->joints[i]);
			joint_limit_project(world, &world->joints[i]);
			i++;
		}
		sweep++;
	}
}

void	resolver_resolve(World *world)
{
	int	iteration;
	int	cursor;
	int	i;

	wake_touching(world);
	i = 0;
	while (i < world->contactCount)
		impulse_prepare(world, &world->contacts[i++]);
	i = 0;
	while (i < world->jointCount)
	{
		joint_point_prepare(world, &world->joints[i]);
		joint_limit_prepare(world, &world->joints[i]);
		i++;
	}
	cursor = 0;
	i = 0;
	while (i < world->contactCount)
		warm_start(world, &world->contacts[i++], &cursor);
	iteration = 0;
	while (iteration < world->solverIterations)
	{
		solve_joints(world);
		i = 0;
		while (i < world->contactCount)
			impulse_solve(world, &world->contacts[i++]);
		iteration++;
	}
	correction_apply(world);
	project_joints(world);
	remember_contacts(world);
}
