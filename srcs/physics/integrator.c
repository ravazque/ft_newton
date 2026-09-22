#include "newton.h"

/* Semi-implicit Euler for one body and one fixed step: velocity first, then the position moves
 * with the NEW velocity, which keeps stacks and bounces stable [F1]-[F4]. */

void	integrator_integrate(RigidBody *b, Vec3 gravity, float dt)
{
	Vec3	acceleration;
	Vec3	angular_acceleration;

	if (b->invMass == 0.0f)
		return ;
	acceleration = vec3_add(vec3_scale(b->forceAccum, b->invMass), gravity);                  /* [F1] a = F/m + g */
	b->velocity = vec3_add(b->velocity, vec3_scale(acceleration, dt));                        /* [F2] v += a dt */
	b->position = vec3_add(b->position, vec3_scale(b->velocity, dt));                         /* [F2] x += v dt */
	angular_acceleration = mat3_mul_vec3(b->invInertiaWorld, b->torqueAccum);                 /* [F1] alpha = I^-1 T */
	b->angularVelocity = vec3_add(b->angularVelocity, vec3_scale(angular_acceleration, dt));  /* [F2] w += alpha dt */
	b->angularVelocity = vec3_scale(b->angularVelocity, powf(ANGULAR_DAMPING, dt));           /* [F4] */
	b->orientation = quat_integrate(b->orientation, b->angularVelocity, dt);                  /* [F3] */
	inertia_update_world(b);                                                                  /* [F7] */
}
