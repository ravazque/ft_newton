
#include "newton.h"

/*
 * [TODO] The rigid body API. rb_make() and
 * rb_clear_accumulators() are provided; the force/impulse/mass functions are
 * the dynamics you must write.
*/

RigidBody	rb_make(void)
{
	RigidBody	b;

	memset(&b, 0, sizeof(b));
	b.mass = 1.0f;
	b.invMass = 1.0f;
	b.orientation = quat_identity();
	b.restitution = 0.4f;
	b.awake = 1;
	b.invInertiaLocal = mat3_identity();
	b.invInertiaWorld = mat3_identity();
	return (b);
}

void	rb_set_mass(RigidBody *b, float mass)
{
	/* TODO: set b->mass and b->invMass (= 1/mass, or 0 if mass <= 0), then
	 * derive invInertiaLocal from the collider via collider_compute_inertia. */
	(void)b;
	(void)mass;
}

void	rb_make_static(RigidBody *b)
{
	/* TODO: invMass = 0 and a zero inverse-inertia tensor => immovable body
	 * (ground, walls). */
	(void)b;
}

void	rb_apply_force(RigidBody *b, Vec3 force)
{
	/* TODO: b->forceAccum += force (at the center of mass, no torque). */
	(void)b;
	(void)force;
}

void	rb_apply_force_at_point(RigidBody *b, Vec3 force, Vec3 world_point)
{
	/* TODO: add linear force AND torque = (world_point - position) x force,
	 * so an off-center hit makes the body spin. */
	(void)b;
	(void)force;
	(void)world_point;
}

void	rb_apply_impulse(RigidBody *b, Vec3 impulse)
{
	/* TODO: b->velocity += impulse * invMass. */
	(void)b;
	(void)impulse;
}

void	rb_apply_impulse_at_point(RigidBody *b, Vec3 impulse, Vec3 world_point)
{
	/* TODO: linear impulse AND angular: angularVelocity += invInertiaWorld *
	 * ((world_point - position) x impulse). */
	(void)b;
	(void)impulse;
	(void)world_point;
}

void	rb_clear_accumulators(RigidBody *b)
{
	b->forceAccum = vec3(0.0f, 0.0f, 0.0f);
	b->torqueAccum = vec3(0.0f, 0.0f, 0.0f);
}
