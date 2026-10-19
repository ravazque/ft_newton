#include "newton.h"

// Semi-implicit Euler: the velocity first, then the position moves with the new velocity.

void	integrator_integrate(RigidBody *body, Vec3 gravity, float dt)
{
	Vec3	acceleration;
	Vec3	angular_acceleration;

	if (body->inverseMass == 0.0f)
		return ;
	acceleration = vec3_add(vec3_scale(body->forceAccum, body->inverseMass), gravity);                  // [F1] a = F/m + g
	body->velocity = vec3_add(body->velocity, vec3_scale(acceleration, dt));                            // [F2] v += a dt
	body->position = vec3_add(body->position, vec3_scale(body->velocity, dt));                          // [F2] x += v dt
	angular_acceleration = mat3_mul_vec3(body->inverseInertiaWorld, body->torqueAccum);                 // [F1] alpha = I^-1 T
	body->angularVelocity = vec3_add(body->angularVelocity, vec3_scale(angular_acceleration, dt));      // [F2] w += alpha dt
	body->angularVelocity = vec3_scale(body->angularVelocity, powf(ANGULAR_DAMPING, dt));               // [F4]
	body->orientation = quat_integrate(body->orientation, body->angularVelocity, dt);                   // [F3]
	inertia_update_world(body);                                                                         // [F7]
}
