#include "newton.h"

// A rigid body's mass properties and the ways forces and impulses change its motion.

RigidBody	rigidbody_make(void)
{
	RigidBody	body;

	memset(&body, 0, sizeof(body));
	body.kind = KIND_BLOCK;
	body.mass = 1.0f;
	body.inverseMass = 1.0f;
	body.orientation = quat_identity();
	body.elasticity = 0.4f;
	body.friction = 0.5f;
	body.color = vec3(0.7f, 0.7f, 0.7f);
	body.awake = 1;
	body.inverseInertiaLocal = mat3_identity();
	body.inverseInertiaWorld = mat3_identity();
	return (body);
}

void	rigidbody_set_mass(RigidBody *body, float mass)
{
	if (mass <= 0.0f)
	{
		rigidbody_make_static(body);
		return ;
	}
	body->mass = mass;
	body->inverseMass = 1.0f / mass;
	body->inverseInertiaLocal = inertia_local_inverse(&body->collider, mass);
	inertia_update_world(body);
}

void	rigidbody_make_static(RigidBody *body)
{
	body->mass = 0.0f;
	body->inverseMass = 0.0f;
	body->inverseInertiaLocal = mat3_zero();
	body->inverseInertiaWorld = mat3_zero();
	body->velocity = vec3(0.0f, 0.0f, 0.0f);
	body->angularVelocity = vec3(0.0f, 0.0f, 0.0f);
}

void	rigidbody_apply_force(RigidBody *body, Vec3 force)
{
	if (body->inverseMass == 0.0f)
		return ;
	body->forceAccum = vec3_add(body->forceAccum, force);
}

void	rigidbody_apply_force_at_point(RigidBody *body, Vec3 force, Vec3 world_point)
{
	Vec3	arm;

	if (body->inverseMass == 0.0f)
		return ;
	arm = vec3_sub(world_point, body->position);
	body->forceAccum = vec3_add(body->forceAccum, force);
	body->torqueAccum = vec3_add(body->torqueAccum, vec3_cross(arm, force));   // [F8] T = r x F
}

void	rigidbody_apply_impulse_at_point(RigidBody *body, Vec3 impulse, Vec3 world_point)
{
	Vec3	arm;

	if (body->inverseMass == 0.0f)
		return ;
	arm = vec3_sub(world_point, body->position);
	body->velocity = vec3_add(body->velocity, vec3_scale(impulse, body->inverseMass));                                             // [F9] v += J / m
	body->angularVelocity = vec3_add(body->angularVelocity, mat3_mul_vec3(body->inverseInertiaWorld, vec3_cross(arm, impulse)));   // [F9] w += I^-1 (r x J)
}

void	rigidbody_apply_angular_impulse(RigidBody *body, Vec3 angular_impulse)
{
	if (body->inverseMass == 0.0f)
		return ;
	body->angularVelocity = vec3_add(body->angularVelocity, mat3_mul_vec3(body->inverseInertiaWorld, angular_impulse));
}

void	rigidbody_clear_accumulators(RigidBody *body)
{
	body->forceAccum = vec3(0.0f, 0.0f, 0.0f);
	body->torqueAccum = vec3(0.0f, 0.0f, 0.0f);
}
