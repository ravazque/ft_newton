#include "newton.h"

/*
 * Sequential impulses on one contact [F17]-[F22] [F26]. Every direction is solved
 * the same way: the impulse that would cancel the relative velocity along it is
 * added to the accumulated total, the TOTAL is clamped (normal >= 0: contacts only
 * push; friction and rolling within their bounds) and only the difference is
 * applied. Clamping the total instead of each increment is what lets stacks converge.
*/

static Vec3	point_velocity(const RigidBody *b, Vec3 r)
{
	return (vec3_add(b->velocity, vec3_cross(b->angularVelocity, r)));   /* [F17] v + w x r */
}

static Vec3	relative_velocity(const RigidBody *a, const RigidBody *b, const Contact *c)
{
	return (vec3_sub(point_velocity(b, vec3_sub(c->point, b->position)), point_velocity(a, vec3_sub(c->point, a->position))));   /* [F18] */
}

/* Impulse needed per unit of velocity change along 'dir', rotation included; 0 when neither body can move. */
static float	effective_mass(const RigidBody *a, const RigidBody *b, const Contact *c, Vec3 dir)
{
	Vec3	ra = vec3_sub(c->point, a->position);
	Vec3	rb = vec3_sub(c->point, b->position);
	Vec3	ta = vec3_cross(mat3_mul_vec3(a->invInertiaWorld, vec3_cross(ra, dir)), ra);
	Vec3	tb = vec3_cross(mat3_mul_vec3(b->invInertiaWorld, vec3_cross(rb, dir)), rb);
	float	k = a->invMass + b->invMass + vec3_dot(vec3_add(ta, tb), dir);   /* [F19] */

	if (k <= 0.0f)
		return (0.0f);
	return (1.0f / k);
}

/* The same for a pure spin about 'axis'. */
static float	angular_mass(const RigidBody *a, const RigidBody *b, Vec3 axis)
{
	float	k = vec3_dot(axis, mat3_mul_vec3(a->invInertiaWorld, axis)) + vec3_dot(axis, mat3_mul_vec3(b->invInertiaWorld, axis));   /* [F26] */

	if (k <= 0.0f)
		return (0.0f);
	return (1.0f / k);
}

static void	tangent_basis(Vec3 n, Vec3 *t1, Vec3 *t2)
{
	if (fabsf(n.x) >= 0.57735f)
		*t1 = vec3_normalized(vec3(n.y, -n.x, 0.0f));
	else
		*t1 = vec3_normalized(vec3(0.0f, n.z, -n.y));
	*t2 = vec3_cross(n, *t1);
}

/* Only a sphere rolls: its coefficient times its radius bounds the resisting spin impulse per
 * unit of normal impulse. */
static float	rolling_arm(const RigidBody *b)
{
	if (b->collider.type != SHAPE_SPHERE)
		return (0.0f);
	return (b->rollingResistance * b->collider.radius);
}

/* The elasticity target is read BEFORE any impulse of the step: warm starts would otherwise
 * look like impacts. */
void	impulse_prepare(const World *w, Contact *c)
{
	const RigidBody	*a = &w->bodies[c->a];
	const RigidBody	*b = &w->bodies[c->b];
	float			vn;

	tangent_basis(c->normal, &c->t1, &c->t2);
	c->massNormal = effective_mass(a, b, c, c->normal);
	c->massT1 = effective_mass(a, b, c, c->t1);
	c->massT2 = effective_mass(a, b, c, c->t2);
	c->massR1 = angular_mass(a, b, c->t1);
	c->massR2 = angular_mass(a, b, c->t2);
	c->massRn = angular_mass(a, b, c->normal);
	c->rolling = fmaxf(rolling_arm(a), rolling_arm(b));									/* [F26] */
	vn = vec3_dot(relative_velocity(a, b, c), c->normal);								/* [F18] */
	c->bias = 0.0f;
	if (vn < -ELASTICITY_THRESHOLD)
		c->bias = -fmaxf(a->elasticity, b->elasticity) * vn;							/* [F20] */
	c->jn = 0.0f;
	c->jt1 = 0.0f;
	c->jt2 = 0.0f;
	c->jr1 = 0.0f;
	c->jr2 = 0.0f;
	c->jrn = 0.0f;
}

/* Equal and opposite impulses at the contact point. */
void	impulse_apply(RigidBody *a, RigidBody *b, const Contact *c, Vec3 impulse)
{
	rb_apply_impulse_at_point(a, vec3_neg(impulse), c->point);
	rb_apply_impulse_at_point(b, impulse, c->point);
}

static void	solve_tangent(RigidBody *a, RigidBody *b, const Contact *c, Vec3 t, float mass, float *accum, float limit)
{
	float	old = *accum;
	float	lambda = -mass * vec3_dot(relative_velocity(a, b, c), t);

	*accum = clampf(old + lambda, -limit, limit);   /* [F22] |jt| <= mu jn */
	impulse_apply(a, b, c, vec3_scale(t, *accum - old));
}

/* Rolling resistance: an opposing spin impulse about a tangent axis (rolling) or the normal
 * (spinning in place, which friction never sees), bounded by c_rr * r * jn. */
static void	solve_rolling(RigidBody *a, RigidBody *b, Vec3 axis, float mass, float *accum, float limit)
{
	float	old = *accum;
	float	lambda = -mass * vec3_dot(vec3_sub(b->angularVelocity, a->angularVelocity), axis);
	Vec3	spin;

	*accum = clampf(old + lambda, -limit, limit);   /* [F26] |jr| <= c_rr r jn */
	spin = vec3_scale(axis, *accum - old);
	rb_apply_angular_impulse(a, vec3_neg(spin));
	rb_apply_angular_impulse(b, spin);
}

void	impulse_solve(World *w, Contact *c)
{
	RigidBody	*a = &w->bodies[c->a];
	RigidBody	*b = &w->bodies[c->b];
	float		mu = sqrtf(a->friction * b->friction);   /* [F22] mu = sqrt(mu_a mu_b) */
	float		old;
	float		lambda;

	if (c->rolling > 0.0f)
	{
		solve_rolling(a, b, c->t1, c->massR1, &c->jr1, c->rolling * c->jn);
		solve_rolling(a, b, c->t2, c->massR2, &c->jr2, c->rolling * c->jn);
		solve_rolling(a, b, c->normal, c->massRn, &c->jrn, c->rolling * c->jn);
	}
	solve_tangent(a, b, c, c->t1, c->massT1, &c->jt1, mu * c->jn);
	solve_tangent(a, b, c, c->t2, c->massT2, &c->jt2, mu * c->jn);
	lambda = c->massNormal * (c->bias - vec3_dot(relative_velocity(a, b, c), c->normal));   /* [F21] */
	old = c->jn;
	c->jn = fmaxf(old + lambda, 0.0f);                                                      /* [F21] push only */
	impulse_apply(a, b, c, vec3_scale(c->normal, c->jn - old));                             /* [F9] */
}
