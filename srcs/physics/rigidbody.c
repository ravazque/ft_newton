
#include "newton.h"

RigidBody	rb_make(void)
{
	RigidBody	b;

	memset(&b, 0, sizeof(b));
	b.kind = KIND_BLOCK;
	b.mass = 1.0f;
	b.invMass = 1.0f;
	b.orientation = quat_identity();
	b.restitution = 0.4f;
	b.friction = 0.5f;
	b.color = vec3(0.7f, 0.7f, 0.7f);
	b.awake = 1;
	b.invInertiaLocal = mat3_identity();
	b.invInertiaWorld = mat3_identity();
	return (b);
}

/* Derives every mass-related quantity from the collider. A non-positive mass
 * means "immovable". */
void	rb_set_mass(RigidBody *b, float mass)
{
	if (mass <= 0.0f)
	{
		rb_make_static(b);
		return ;
	}
	b->mass = mass;
	b->invMass = 1.0f / mass;
	b->invInertiaLocal = collider_compute_inertia(&b->collider, mass);   /* [F5] [F6] */
	rb_update_inertia_world(b);
}

/* invMass = 0 and a zero inverse inertia make every impulse a no-op: the body
 * behaves as if infinitely heavy (ground, catapult). */
void	rb_make_static(RigidBody *b)
{
	b->mass = 0.0f;
	b->invMass = 0.0f;
	b->invInertiaLocal = mat3_zero();
	b->invInertiaWorld = mat3_zero();
	b->velocity = vec3(0.0f, 0.0f, 0.0f);
	b->angularVelocity = vec3(0.0f, 0.0f, 0.0f);
}

/* I^-1_world = R * I^-1_local * R^T, refreshed whenever the orientation moves. */
void	rb_update_inertia_world(RigidBody *b)
{
	Mat3	r = mat3_from_quat(b->orientation);

	b->invInertiaWorld = mat3_mul(mat3_mul(r, b->invInertiaLocal), mat3_transpose(r));   /* [F7] I_world^-1 = R I^-1 R^T */
}

void	rb_apply_force(RigidBody *b, Vec3 force)
{
	if (b->invMass == 0.0f)
		return ;
	b->forceAccum = vec3_add(b->forceAccum, force);
}

/* A force applied away from the center of mass also produces the torque
 * r x F, which is what makes an off-center hit spin the body. */
void	rb_apply_force_at_point(RigidBody *b, Vec3 force, Vec3 world_point)
{
	if (b->invMass == 0.0f)
		return ;
	b->forceAccum = vec3_add(b->forceAccum, force);
	b->torqueAccum = vec3_add(b->torqueAccum, vec3_cross(vec3_sub(world_point, b->position), force));   /* [F8] T = r x F */
}

void	rb_apply_impulse(RigidBody *b, Vec3 impulse)
{
	if (b->invMass == 0.0f)
		return ;
	b->velocity = vec3_add(b->velocity, vec3_scale(impulse, b->invMass));
}

/* Instant velocity change: linear part scaled by 1/m, angular part by the
 * world inverse inertia applied to r x J. */
void	rb_apply_impulse_at_point(RigidBody *b, Vec3 impulse, Vec3 world_point)
{
	if (b->invMass == 0.0f)
		return ;
	b->velocity = vec3_add(b->velocity, vec3_scale(impulse, b->invMass));                                                                            /* [F9] v += J/m        */
	b->angularVelocity = vec3_add(b->angularVelocity, mat3_mul_vec3(b->invInertiaWorld, vec3_cross(vec3_sub(world_point, b->position), impulse)));   /* [F9] w += I^-1 (r x J) */
}

void	rb_clear_accumulators(RigidBody *b)
{
	b->forceAccum = vec3(0.0f, 0.0f, 0.0f);
	b->torqueAccum = vec3(0.0f, 0.0f, 0.0f);
}
