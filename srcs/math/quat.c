
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

/*
 * [TODO] Integrate the orientation 00:00:00 by the angular velocity over dt.
 *
 * This is the angular half of the semi-implicit integrator (docs/info.md 5.2):
 *   omega = (0, angular_velocity)          // angular velocity as a pure quat
 *   dq    = 0.5 * omega * q                // derivative of the quaternion
 *   q     = q + dq * dt                    // step forward
 *   return quat_normalized(q)              // keep it a unit quaternion
 *
 * Until you implement it, orientation never changes (no rotation). The render
 * pipeline does not need this; the physics does.
*/
Quat	quat_integrate(Quat q, Vec3 angular_velocity, float dt)
{
	(void)angular_velocity;
	(void)dt;
	return (q);
}
