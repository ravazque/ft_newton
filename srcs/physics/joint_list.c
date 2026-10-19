#include "newton.h"

// The joints of the world: made from the current pose, renumbered when bodies move, and the pairs they keep apart.

static Vec3	to_body(const RigidBody *body, Vec3 world_vector)
{
	return (quat_rotate(quat_conjugate(body->orientation), world_vector));
}

// The reference is carried from axis_a to axis_b by the swing between them, so every angle starts at 0.
Joint	joint_make(const World *world, int body_a, int body_b, Vec3 anchor, Vec3 axis_a, Vec3 axis_b, Vec3 reference)
{
	const RigidBody	*first = &world->bodies[body_a];
	const RigidBody	*second = &world->bodies[body_b];
	Joint			joint;
	Vec3			reference_a;

	memset(&joint, 0, sizeof(joint));
	joint.bodyA = body_a;
	joint.bodyB = body_b;
	axis_a = vec3_normalized(axis_a);
	axis_b = vec3_normalized(axis_b);
	reference_a = vec3_normalized(vec3_sub(reference, vec3_scale(axis_a, vec3_dot(reference, axis_a))));
	joint.anchorA = to_body(first, vec3_sub(anchor, first->position));
	joint.anchorB = to_body(second, vec3_sub(anchor, second->position));
	joint.axisA = to_body(first, axis_a);
	joint.axisB = to_body(second, axis_b);
	joint.referenceA = to_body(first, reference_a);
	joint.referenceB = to_body(second, quat_rotate(quat_from_to(axis_a, axis_b), reference_a));
	return (joint);
}

int	joint_add(World *world, Joint joint)
{
	Joint	*grown;
	int		new_capacity;

	if (world->jointCount == world->jointCapacity)
	{
		new_capacity = 16;
		if (world->jointCapacity > 0)
			new_capacity = world->jointCapacity * 2;
		grown = realloc(world->joints, (size_t)new_capacity * sizeof(Joint));
		if (!grown)
			return (-1);
		world->joints = grown;
		world->jointCapacity = new_capacity;
	}
	world->joints[world->jointCount] = joint;
	return (world->jointCount++);
}

void	joint_forget_body(World *world, int removed, int moved)
{
	Joint	*joint;
	int		i;

	i = 0;
	while (i < world->jointCount)
	{
		joint = &world->joints[i];
		if (joint->bodyA == removed || joint->bodyB == removed)
		{
			*joint = world->joints[--world->jointCount];
			continue ;
		}
		if (joint->bodyA == moved)
			joint->bodyA = removed;
		if (joint->bodyB == moved)
			joint->bodyB = removed;
		i++;
	}
}

int	joint_ignores_pair(const World *world, int body_a, int body_b)
{
	const Joint	*joint;
	int			i;

	if (world->bodies[body_a].ragdollId == 0 || world->bodies[body_a].ragdollId != world->bodies[body_b].ragdollId)
		return (0);
	i = 0;
	while (i < world->jointCount)
	{
		joint = &world->joints[i];
		if (!joint->collides && ((joint->bodyA == body_a && joint->bodyB == body_b) || (joint->bodyA == body_b && joint->bodyB == body_a)))
			return (1);
		i++;
	}
	return (0);
}
