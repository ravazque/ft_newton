
#include "newton.h"

Quat	quat_identity(void)
{
	return ((Quat){1.0f, 0.0f, 0.0f, 0.0f});
}

Quat	quat_from_axis_angle(Vec3 axis, float radians)
{
	float	half = radians * 0.5f;
	float	s = sinf(half);

	return ((Quat){cosf(half), axis.x * s, axis.y * s, axis.z * s});
}

Quat	quat_mul(Quat a, Quat b)
{
	return ((Quat){
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w});
}

Quat	quat_normalized(Quat q)
{
	float	len;
	float	inv;

	len = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (len <= 0.0f)
		return (quat_identity());
	inv = 1.0f / len;
	return ((Quat){q.w * inv, q.x * inv, q.y * inv, q.z * inv});
}

Vec3	quat_rotate(Quat q, Vec3 v)
{
	Vec3	u = vec3(q.x, q.y, q.z);
	Vec3	t = vec3_scale(vec3_cross(u, v), 2.0f);

	return (vec3_add(vec3_add(v, vec3_scale(t, q.w)), vec3_cross(u, t)));
}

/* Shortest rotation taking unit vector 'from' onto unit vector 'to'. Opposite
 * vectors rotate 180 degrees around any perpendicular axis. */
Quat	quat_from_to(Vec3 from, Vec3 to)
{
	float	d = vec3_dot(from, to);
	Vec3	axis;

	if (d >= 0.999999f)
		return (quat_identity());
	if (d <= -0.999999f)
	{
		axis = vec3_cross(vec3(1.0f, 0.0f, 0.0f), from);
		if (vec3_length_sq(axis) < 0.000001f)
			axis = vec3_cross(vec3(0.0f, 1.0f, 0.0f), from);
		return (quat_from_axis_angle(vec3_normalized(axis), FTN_PI));
	}
	axis = vec3_cross(from, to);
	return (quat_normalized((Quat){1.0f + d, axis.x, axis.y, axis.z}));
}

/*
 * Angular half of the semi-implicit integrator. With the angular velocity as
 * a pure quaternion omega = (0, w), the orientation derivative is
 * dq/dt = 0.5 * omega * q; one explicit step is q += dq * dt, renormalized
 * so it stays a unit quaternion.
*/
Quat	quat_integrate(Quat q, Vec3 angular_velocity, float dt)
{
	Quat	omega = {0.0f, angular_velocity.x, angular_velocity.y, angular_velocity.z};
	Quat	dq = quat_mul(omega, q);   /* [F3] dq/dt = 0.5 (0, w) q */
	float	half_dt = 0.5f * dt;

	q.w += dq.w * half_dt;
	q.x += dq.x * half_dt;
	q.y += dq.y * half_dt;
	q.z += dq.z * half_dt;
	return (quat_normalized(q));
}
