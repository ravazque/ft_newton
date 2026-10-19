#include "newton.h"

// Sequential impulses of one contact [F17]-[F22] [F26]: each accumulated total is clamped, only the change is applied.

static Vec3	point_velocity(const RigidBody *body, Vec3 arm)
{
	return (vec3_add(body->velocity, vec3_cross(body->angularVelocity, arm)));   // [F17] v + w x r
}

static Vec3	relative_velocity(const RigidBody *body_a, const RigidBody *body_b, const Contact *contact)
{
	return (vec3_sub(point_velocity(body_b, vec3_sub(contact->point, body_b->position)), point_velocity(body_a, vec3_sub(contact->point, body_a->position))));   // [F18]
}

static float	effective_mass(const RigidBody *body_a, const RigidBody *body_b, const Contact *contact, Vec3 direction)
{
	Vec3	arm_a = vec3_sub(contact->point, body_a->position);
	Vec3	arm_b = vec3_sub(contact->point, body_b->position);
	Vec3	turn_a = vec3_cross(mat3_mul_vec3(body_a->inverseInertiaWorld, vec3_cross(arm_a, direction)), arm_a);
	Vec3	turn_b = vec3_cross(mat3_mul_vec3(body_b->inverseInertiaWorld, vec3_cross(arm_b, direction)), arm_b);
	float	inverse_a = contact->scaleA * (body_a->inverseMass + vec3_dot(turn_a, direction));
	float	inverse = inverse_a + contact->scaleB * (body_b->inverseMass + vec3_dot(turn_b, direction));   // [F19] [F39]

	if (inverse <= 0.0f)
		return (0.0f);
	return (1.0f / inverse);
}

static float	angular_mass(const RigidBody *body_a, const RigidBody *body_b, const Contact *contact, Vec3 axis)
{
	float	inverse_a = contact->scaleA * vec3_dot(axis, mat3_mul_vec3(body_a->inverseInertiaWorld, axis));
	float	inverse = inverse_a + contact->scaleB * vec3_dot(axis, mat3_mul_vec3(body_b->inverseInertiaWorld, axis));   // [F26]

	if (inverse <= 0.0f)
		return (0.0f);
	return (1.0f / inverse);
}

static void	tangent_basis(Vec3 normal, Vec3 *tangent1, Vec3 *tangent2)
{
	if (fabsf(normal.x) >= 0.57735f)
		*tangent1 = vec3_normalized(vec3(normal.y, -normal.x, 0.0f));
	else
		*tangent1 = vec3_normalized(vec3(0.0f, normal.z, -normal.y));
	*tangent2 = vec3_cross(normal, *tangent1);
}

static float	rolling_arm(const RigidBody *body)
{
	if (body->collider.type != SHAPE_SPHERE)
		return (0.0f);
	return (body->rollingResistance * body->collider.radius);
}

static void	limit_mass_ratio(const World *world, const RigidBody *body_a, const RigidBody *body_b, Contact *contact)
{
	float	up_along_normal;

	contact->scaleA = 1.0f;
	contact->scaleB = 1.0f;
	if (body_a->inverseMass == 0.0f || body_b->inverseMass == 0.0f || vec3_length_sq(world->gravity) == 0.0f)
		return ;
	up_along_normal = -vec3_dot(contact->normal, vec3_normalized(world->gravity));
	if (body_a->inverseMass > MASS_RATIO_LIMIT * body_b->inverseMass && up_along_normal > SUPPORT_COSINE)
		contact->scaleA = fmaxf(MASS_RATIO_LIMIT * body_b->inverseMass / body_a->inverseMass, MASS_SCALE_FLOOR);   // [F39]
	else if (body_b->inverseMass > MASS_RATIO_LIMIT * body_a->inverseMass && up_along_normal < -SUPPORT_COSINE)
		contact->scaleB = fmaxf(MASS_RATIO_LIMIT * body_a->inverseMass / body_b->inverseMass, MASS_SCALE_FLOOR);   // [F39]
}

// The elasticity target is read before any impulse of the step, so warm starts do not look like impacts.
void	impulse_prepare(const World *world, Contact *contact)
{
	const RigidBody	*body_a = &world->bodies[contact->bodyA];
	const RigidBody	*body_b = &world->bodies[contact->bodyB];
	float			approach;

	limit_mass_ratio(world, body_a, body_b, contact);
	tangent_basis(contact->normal, &contact->tangent1, &contact->tangent2);
	contact->massNormal = effective_mass(body_a, body_b, contact, contact->normal);
	contact->massTangent1 = effective_mass(body_a, body_b, contact, contact->tangent1);
	contact->massTangent2 = effective_mass(body_a, body_b, contact, contact->tangent2);
	contact->massRoll1 = angular_mass(body_a, body_b, contact, contact->tangent1);
	contact->massRoll2 = angular_mass(body_a, body_b, contact, contact->tangent2);
	contact->massSpin = angular_mass(body_a, body_b, contact, contact->normal);
	contact->rollingArm = fmaxf(rolling_arm(body_a), rolling_arm(body_b));   // [F26]
	approach = vec3_dot(relative_velocity(body_a, body_b, contact), contact->normal);   // [F18]
	contact->restitutionBias = 0.0f;
	if (approach < -ELASTICITY_THRESHOLD)
		contact->restitutionBias = -fmaxf(body_a->elasticity, body_b->elasticity) * approach;   // [F20]
	contact->impulseNormal = 0.0f;
	contact->impulseTangent1 = 0.0f;
	contact->impulseTangent2 = 0.0f;
	contact->impulseRoll1 = 0.0f;
	contact->impulseRoll2 = 0.0f;
	contact->impulseSpin = 0.0f;
}

void	impulse_apply(RigidBody *body_a, RigidBody *body_b, const Contact *contact, Vec3 impulse)
{
	rigidbody_apply_impulse_at_point(body_a, vec3_scale(impulse, -contact->scaleA), contact->point);
	rigidbody_apply_impulse_at_point(body_b, vec3_scale(impulse, contact->scaleB), contact->point);
}

static void	solve_tangent(RigidBody **bodies, const Contact *contact, Vec3 tangent, float mass, float *accumulated, float limit)
{
	float	previous = *accumulated;
	float	lambda = -mass * vec3_dot(relative_velocity(bodies[0], bodies[1], contact), tangent);

	*accumulated = clampf(previous + lambda, -limit, limit);   // [F22] |jt| <= mu jn
	impulse_apply(bodies[0], bodies[1], contact, vec3_scale(tangent, *accumulated - previous));
}

static void	solve_rolling(RigidBody **bodies, const Contact *contact, Vec3 axis, float mass, float *accumulated, float limit)
{
	float	previous = *accumulated;
	float	lambda = -mass * vec3_dot(vec3_sub(bodies[1]->angularVelocity, bodies[0]->angularVelocity), axis);
	Vec3	spin;

	*accumulated = clampf(previous + lambda, -limit, limit);   // [F26] |jr| <= c_rr r jn
	spin = vec3_scale(axis, *accumulated - previous);
	rigidbody_apply_angular_impulse(bodies[0], vec3_scale(spin, -contact->scaleA));
	rigidbody_apply_angular_impulse(bodies[1], vec3_scale(spin, contact->scaleB));
}

void	impulse_solve(World *world, Contact *contact)
{
	RigidBody	*bodies[2] = {&world->bodies[contact->bodyA], &world->bodies[contact->bodyB]};
	float		friction = sqrtf(bodies[0]->friction * bodies[1]->friction);   // [F22] mu = sqrt(mu_a mu_b)
	float		previous;
	float		lambda;

	if (contact->rollingArm > 0.0f)
	{
		solve_rolling(bodies, contact, contact->tangent1, contact->massRoll1, &contact->impulseRoll1, contact->rollingArm * contact->impulseNormal);
		solve_rolling(bodies, contact, contact->tangent2, contact->massRoll2, &contact->impulseRoll2, contact->rollingArm * contact->impulseNormal);
		solve_rolling(bodies, contact, contact->normal, contact->massSpin, &contact->impulseSpin, contact->rollingArm * contact->impulseNormal);
	}
	solve_tangent(bodies, contact, contact->tangent1, contact->massTangent1, &contact->impulseTangent1, friction * contact->impulseNormal);
	solve_tangent(bodies, contact, contact->tangent2, contact->massTangent2, &contact->impulseTangent2, friction * contact->impulseNormal);
	lambda = contact->massNormal * (contact->restitutionBias - vec3_dot(relative_velocity(bodies[0], bodies[1], contact), contact->normal));   // [F21]
	previous = contact->impulseNormal;
	contact->impulseNormal = fmaxf(previous + lambda, 0.0f);                                                                             // [F21] push only
	impulse_apply(bodies[0], bodies[1], contact, vec3_scale(contact->normal, contact->impulseNormal - previous));                         // [F9]
}
