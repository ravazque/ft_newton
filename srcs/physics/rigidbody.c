#include "newton.h"

/* A rigid body's mass properties and the ways forces and impulses change its motion [F8] [F9]. */

RigidBody	rb_make(void)
{
	RigidBody	b;

	memset(&b, 0, sizeof(b));
	b.kind = KIND_BLOCK;
	b.mass = 1.0f;
	b.invMass = 1.0f;
	b.orientation = quat_identity();
	b.elasticity = 0.4f;
	b.friction = 0.5f;
	b.color = vec3(0.7f, 0.7f, 0.7f);
	b.awake = 1;
	b.invInertiaLocal = mat3_identity();
	b.invInertiaWorld = mat3_identity();
	return (b);
}

/* Mass and inertia follow the collider; a non-positive mass means immovable. */
void	rb_set_mass(RigidBody *b, float mass)
{
	if (mass <= 0.0f)
	{
		rb_make_static(b);
		return ;
	}
	b->mass = mass;
	b->invMass = 1.0f / mass;
	b->invInertiaLocal = inertia_local_inverse(&b->collider, mass);
	inertia_update_world(b);
}

/* Zero inverse mass and inertia turn every impulse into a no-op. */
void	rb_make_static(RigidBody *b)
{
	b->mass = 0.0f;
	b->invMass = 0.0f;
	b->invInertiaLocal = mat3_zero();
	b->invInertiaWorld = mat3_zero();
	b->velocity = vec3(0.0f, 0.0f, 0.0f);
	b->angularVelocity = vec3(0.0f, 0.0f, 0.0f);
}

void	rb_apply_force(RigidBody *b, Vec3 force)
{
	if (b->invMass == 0.0f)
		return ;
	b->forceAccum = vec3_add(b->forceAccum, force);
}

/* Off the center of mass a force also makes a torque: that is what spins a body. */
void	rb_apply_force_at_point(RigidBody *b, Vec3 force, Vec3 world_point)
{
	Vec3	arm;

	if (b->invMass == 0.0f)
		return ;
	arm = vec3_sub(world_point, b->position);
	b->forceAccum = vec3_add(b->forceAccum, force);
	b->torqueAccum = vec3_add(b->torqueAccum, vec3_cross(arm, force));   /* [F8] T = r x F */
}

void	rb_apply_impulse(RigidBody *b, Vec3 impulse)
{
	if (b->invMass == 0.0f)
		return ;
	b->velocity = vec3_add(b->velocity, vec3_scale(impulse, b->invMass));
}

void	rb_apply_impulse_at_point(RigidBody *b, Vec3 impulse, Vec3 world_point)
{
	Vec3	arm;

	if (b->invMass == 0.0f)
		return ;
	arm = vec3_sub(world_point, b->position);
	b->velocity = vec3_add(b->velocity, vec3_scale(impulse, b->invMass));												/* [F9] v += J / m */
	b->angularVelocity = vec3_add(b->angularVelocity, mat3_mul_vec3(b->invInertiaWorld, vec3_cross(arm, impulse)));		/* [F9] w += I^-1 (r x J) */
}

/* A pure torque impulse: changes the spin, never the velocity. */
void	rb_apply_angular_impulse(RigidBody *b, Vec3 angular_impulse)
{
	if (b->invMass == 0.0f)
		return ;
	b->angularVelocity = vec3_add(b->angularVelocity, mat3_mul_vec3(b->invInertiaWorld, angular_impulse));
}

void	rb_clear_accumulators(RigidBody *b)
{
	b->forceAccum = vec3(0.0f, 0.0f, 0.0f);
	b->torqueAccum = vec3(0.0f, 0.0f, 0.0f);
}
