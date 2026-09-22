
#include "newton.h"

/*
 * Semi-implicit (symplectic) Euler for one body and one fixed step. The
 * velocity is updated first and the position moves with the NEW velocity;
 * it costs the same as explicit Euler but stays stable for the oscillating
 * systems (stacks, bounces) a physics engine is made of.
 *
 *   Linear:   v += (F / m + g) * dt        x += v * dt
 *   Angular:  w += (I^-1_world * tau) * dt  q  = integrate(q, w, dt)
 *
 * Angular velocity gets a mild exponential damping so spinning bodies bleed
 * energy the contact solver cannot see (rolling resistance, air), which lets
 * them fall asleep. Static bodies (invMass == 0) are never moved.
*/
void	integrator_integrate(RigidBody *b, Vec3 gravity, float dt)
{
	Vec3	acceleration;
	Vec3	angular_acceleration;

	if (b->invMass == 0.0f)
		return ;
	acceleration = vec3_add(vec3_scale(b->forceAccum, b->invMass), gravity);                          /* [F1] a = F/m + g */
	b->velocity = vec3_add(b->velocity, vec3_scale(acceleration, dt));                                /* [F2] v += a dt   */
	b->position = vec3_add(b->position, vec3_scale(b->velocity, dt));                                 /* [F2] x += v dt   */
	angular_acceleration = mat3_mul_vec3(b->invInertiaWorld, b->torqueAccum);                         /* [F1] alpha = I^-1 T */
	b->angularVelocity = vec3_add(b->angularVelocity, vec3_scale(angular_acceleration, dt));          /* [F2] w += alpha dt */
	b->angularVelocity = vec3_scale(b->angularVelocity, powf(ANGULAR_DAMPING, dt));                   /* [F4] */
	b->orientation = quat_integrate(b->orientation, b->angularVelocity, dt);                          /* [F3] */
	rb_update_inertia_world(b);                                                                       /* [F7] */
}
