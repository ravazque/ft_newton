#include "newton.h"

// The shared point of a joint [F32]: both anchors move together, solved in three directions at once through a 3x3 mass.

static int	joint_active(const World *world, const Joint *joint)
{
	return (world->bodies[joint->bodyA].awake || world->bodies[joint->bodyB].awake);
}

static Mat3	point_mass(const RigidBody *body_a, const RigidBody *body_b, Vec3 arm_a, Vec3 arm_b)
{
	Mat3	mass = mat3_scale(mat3_identity(), body_a->inverseMass + body_b->inverseMass);
	Mat3	skew_a = mat3_skew(arm_a);
	Mat3	skew_b = mat3_skew(arm_b);

	mass = mat3_add(mass, mat3_scale(mat3_mul(mat3_mul(skew_a, body_a->inverseInertiaWorld), skew_a), -1.0f));   // [F32]
	mass = mat3_add(mass, mat3_scale(mat3_mul(mat3_mul(skew_b, body_b->inverseInertiaWorld), skew_b), -1.0f));
	return (mat3_inverse(mass));
}

static void	apply_point(RigidBody *body_a, RigidBody *body_b, const Joint *joint, Vec3 impulse)
{
	rigidbody_apply_impulse_at_point(body_a, vec3_neg(impulse), vec3_add(body_a->position, joint->armA));
	rigidbody_apply_impulse_at_point(body_b, impulse, vec3_add(body_b->position, joint->armB));
}

static void	damp_spin(RigidBody *body_a, RigidBody *body_b)
{
	Mat3	mass = mat3_inverse(mat3_add(body_a->inverseInertiaWorld, body_b->inverseInertiaWorld));
	Vec3	impulse = mat3_mul_vec3(mass, vec3_scale(vec3_sub(body_b->angularVelocity, body_a->angularVelocity), -JOINT_SPIN_DAMPING));   // [F36] joint friction

	rigidbody_apply_angular_impulse(body_a, vec3_neg(impulse));
	rigidbody_apply_angular_impulse(body_b, impulse);
}

void	joint_point_prepare(World *world, Joint *joint)
{
	RigidBody	*body_a = &world->bodies[joint->bodyA];
	RigidBody	*body_b = &world->bodies[joint->bodyB];

	if (!joint_active(world, joint))
		return ;
	joint->armA = quat_rotate(body_a->orientation, joint->anchorA);
	joint->armB = quat_rotate(body_b->orientation, joint->anchorB);
	joint->pointMass = point_mass(body_a, body_b, joint->armA, joint->armB);
	apply_point(body_a, body_b, joint, joint->pointImpulse);
	damp_spin(body_a, body_b);
}

void	joint_point_solve(World *world, Joint *joint)
{
	RigidBody	*body_a = &world->bodies[joint->bodyA];
	RigidBody	*body_b = &world->bodies[joint->bodyB];
	Vec3		velocity_a;
	Vec3		velocity_b;
	Vec3		impulse;

	if (!joint_active(world, joint))
		return ;
	velocity_a = vec3_add(body_a->velocity, vec3_cross(body_a->angularVelocity, joint->armA));   // [F17]
	velocity_b = vec3_add(body_b->velocity, vec3_cross(body_b->angularVelocity, joint->armB));
	impulse = mat3_mul_vec3(joint->pointMass, vec3_neg(vec3_sub(velocity_b, velocity_a)));        // [F32] P = -K^-1 (vb - va)
	joint->pointImpulse = vec3_add(joint->pointImpulse, impulse);
	apply_point(body_a, body_b, joint, impulse);
}

// Projection [F38]: an anchor still open by more than JOINT_PROJECTION_DISTANCE moves its child back, and stops it moving away.
void	joint_point_project(World *world, Joint *joint)
{
	RigidBody	*body_a = &world->bodies[joint->bodyA];
	RigidBody	*body_b = &world->bodies[joint->bodyB];
	Vec3		arm_a;
	Vec3		arm_b;
	Vec3		gap;
	Vec3		direction;
	float		length;
	float		separating;

	if (!joint_active(world, joint) || body_b->inverseMass == 0.0f)
		return ;
	arm_a = quat_rotate(body_a->orientation, joint->anchorA);
	arm_b = quat_rotate(body_b->orientation, joint->anchorB);
	gap = vec3_sub(vec3_add(body_b->position, arm_b), vec3_add(body_a->position, arm_a));
	length = vec3_length(gap);
	if (length <= JOINT_PROJECTION_DISTANCE)
		return ;
	direction = vec3_scale(gap, 1.0f / length);
	body_b->position = vec3_sub(body_b->position, vec3_scale(direction, length - JOINT_PROJECTION_DISTANCE));   // [F38]
	separating = vec3_dot(vec3_sub(vec3_add(body_b->velocity, vec3_cross(body_b->angularVelocity, arm_b)), vec3_add(body_a->velocity, vec3_cross(body_a->angularVelocity, arm_a))), direction);
	if (separating > 0.0f)
		body_b->velocity = vec3_sub(body_b->velocity, vec3_scale(direction, separating));   // [F38]
}

void	joint_point_correct(World *world, Joint *joint)
{
	RigidBody	*body_a = &world->bodies[joint->bodyA];
	RigidBody	*body_b = &world->bodies[joint->bodyB];
	Vec3		arm_a;
	Vec3		arm_b;
	Vec3		gap;
	Vec3		push;
	float		length;

	if (!joint_active(world, joint))
		return ;
	arm_a = quat_rotate(body_a->orientation, joint->anchorA);
	arm_b = quat_rotate(body_b->orientation, joint->anchorB);
	gap = vec3_sub(vec3_add(body_b->position, arm_b), vec3_add(body_a->position, arm_a));
	length = vec3_length(gap);
	if (length <= JOINT_SLOP)
		return ;
	if (length > JOINT_MAX_CORRECTION)
		gap = vec3_scale(gap, JOINT_MAX_CORRECTION / length);
	push = mat3_mul_vec3(point_mass(body_a, body_b, arm_a, arm_b), vec3_scale(gap, -JOINT_PERCENT));   // [F35]
	correction_push(world, joint->bodyA, arm_a, vec3_neg(push));
	correction_push(world, joint->bodyB, arm_b, push);
}
