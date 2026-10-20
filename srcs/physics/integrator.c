#include "newton.h"

// Semi-implicit Euler: the velocity first, then the position moves with the new velocity.

// The thinnest dimension of the shape: a body that moves less than it per step always overlaps what it would cross.
static float	thickness(const Collider *collider)
{
	Vec3	extent;

	if (collider->type == SHAPE_SPHERE)
		return (2.0f * collider->radius);
	if (collider->type == SHAPE_BOX)
		extent = vec3_scale(collider->halfExtents, 2.0f);
	else if (collider->type == SHAPE_MESH)
		extent = vec3_sub(collider->mesh->boundsMax, collider->mesh->boundsMin);
	else
		return (INFINITY);
	return (fminf(extent.x, fminf(extent.y, extent.z)));
}

// a speed cap instead of a swept test, at most its thinnest dimension per step; it only bites small bodies thrown by
// heavy fast ones (a 0.07 m rain ball thrown by a 50 kg apple), the upgrade is continuous collision detection.
static void	clamp_speed(RigidBody *body, float dt)
{
	float	max_speed = thickness(&body->collider) / dt;
	float	speed_sq = vec3_length_sq(body->velocity);

	if (speed_sq > max_speed * max_speed)
		body->velocity = vec3_scale(body->velocity, max_speed / sqrtf(speed_sq));
}

void	integrator_integrate(RigidBody *body, Vec3 gravity, float dt)
{
	Vec3	acceleration;
	Vec3	angular_acceleration;

	if (body->inverseMass == 0.0f)
		return ;
	acceleration = vec3_add(vec3_scale(body->forceAccum, body->inverseMass), gravity);                  // [F1] a = F/m + g
	body->velocity = vec3_add(body->velocity, vec3_scale(acceleration, dt));                            // [F2] v += a dt
	clamp_speed(body, dt);
	body->position = vec3_add(body->position, vec3_scale(body->velocity, dt));                          // [F2] x += v dt
	angular_acceleration = mat3_mul_vec3(body->inverseInertiaWorld, body->torqueAccum);                 // [F1] alpha = I^-1 T
	body->angularVelocity = vec3_add(body->angularVelocity, vec3_scale(angular_acceleration, dt));      // [F2] w += alpha dt
	body->angularVelocity = vec3_scale(body->angularVelocity, powf(ANGULAR_DAMPING, dt));               // [F4]
	body->orientation = quat_integrate(body->orientation, body->angularVelocity, dt);                   // [F3]
	inertia_update_world(body);                                                                         // [F7]
}
