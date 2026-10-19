#include "newton.h"

// Positional correction [F23]: overlap and joint drift removed on poses, a share per pass, so no energy is added.

// Overlap left at a contact: its depth minus how far this step's corrections moved the two points apart.
static float	current_depth(const World *world, const Contact *contact, Vec3 arm_a, Vec3 arm_b)
{
	Vec3	moved_a = vec3_add(vec3_sub(world->bodies[contact->bodyA].position, world->startPositions[contact->bodyA]), vec3_cross(world->rotationDelta[contact->bodyA], arm_a));
	Vec3	moved_b = vec3_add(vec3_sub(world->bodies[contact->bodyB].position, world->startPositions[contact->bodyB]), vec3_cross(world->rotationDelta[contact->bodyB], arm_b));

	return (contact->penetration - vec3_dot(contact->normal, vec3_sub(moved_b, moved_a)));
}

void	correction_push(World *world, int body, Vec3 arm, Vec3 push)
{
	RigidBody	*target = &world->bodies[body];
	Vec3		rotation;

	if (target->inverseMass == 0.0f)
		return ;
	target->position = vec3_add(target->position, vec3_scale(push, target->inverseMass));   // [F23] x += c n / m
	rotation = mat3_mul_vec3(target->inverseInertiaWorld, vec3_cross(arm, push));          // [F23] I^-1 (r x c n)
	world->rotationDelta[body] = vec3_add(world->rotationDelta[body], rotation);
	target->orientation = quat_integrate(target->orientation, rotation, 1.0f);
}

void	correction_turn(World *world, int body, Vec3 angular_push)
{
	RigidBody	*target = &world->bodies[body];
	Vec3		rotation;

	if (target->inverseMass == 0.0f)
		return ;
	rotation = mat3_mul_vec3(target->inverseInertiaWorld, angular_push);
	world->rotationDelta[body] = vec3_add(world->rotationDelta[body], rotation);
	target->orientation = quat_integrate(target->orientation, rotation, 1.0f);
}

static void	correct_contacts(World *world)
{
	const Contact	*contact;
	Vec3			arm_a;
	Vec3			arm_b;
	float			depth;
	float			amount;
	int				i;

	i = 0;
	while (i < world->contactCount)
	{
		contact = &world->contacts[i];
		arm_a = vec3_sub(contact->point, world->startPositions[contact->bodyA]);
		arm_b = vec3_sub(contact->point, world->startPositions[contact->bodyB]);
		depth = clampf(current_depth(world, contact, arm_a, arm_b) - PENETRATION_SLOP, 0.0f, MAX_CORRECTION);
		amount = depth * PENETRATION_PERCENT * contact->massNormal;   // [F23]
		if (amount > 0.0f)
		{
			correction_push(world, contact->bodyA, arm_a, vec3_scale(contact->normal, -amount * contact->scaleA));   // [F39]
			correction_push(world, contact->bodyB, arm_b, vec3_scale(contact->normal, amount * contact->scaleB));
		}
		i++;
	}
}

void	correction_apply(World *world)
{
	int	pass;
	int	i;

	if (!world_grow_scratch(world))
		return ;
	i = 0;
	while (i < world->bodyCount)
	{
		world->startPositions[i] = world->bodies[i].position;
		world->rotationDelta[i] = vec3(0.0f, 0.0f, 0.0f);
		i++;
	}
	pass = 0;
	while (pass < POSITION_ITERATIONS)
	{
		i = 0;
		while (i < world->jointCount)
		{
			joint_point_correct(world, &world->joints[i]);
			joint_limit_correct(world, &world->joints[i]);
			i++;
		}
		correct_contacts(world);
		pass++;
	}
}
