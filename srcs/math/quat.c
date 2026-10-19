#include "newton.h"

// Unit quaternions: orientations, the rotation of vectors and their integration from an angular velocity.

Quat	quat_identity(void)
{
	return ((Quat){1.0f, 0.0f, 0.0f, 0.0f});
}

Quat	quat_from_axis_angle(Vec3 axis, float radians)
{
	float	half = radians * 0.5f;
	float	sine = sinf(half);

	return ((Quat){cosf(half), axis.x * sine, axis.y * sine, axis.z * sine});
}

Quat	quat_mul(Quat a, Quat b)
{
	return ((Quat){
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w});
}

Quat	quat_normalized(Quat rotation)
{
	float	length;
	float	inverse;

	length = sqrtf(rotation.w * rotation.w + rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z);
	if (length <= 0.0f)
		return (quat_identity());
	inverse = 1.0f / length;
	return ((Quat){rotation.w * inverse, rotation.x * inverse, rotation.y * inverse, rotation.z * inverse});
}

Quat	quat_conjugate(Quat rotation)
{
	return ((Quat){rotation.w, -rotation.x, -rotation.y, -rotation.z});
}

Vec3	quat_rotate(Quat rotation, Vec3 vector)
{
	Vec3	axis = vec3(rotation.x, rotation.y, rotation.z);
	Vec3	twice_cross = vec3_scale(vec3_cross(axis, vector), 2.0f);

	return (vec3_add(vec3_add(vector, vec3_scale(twice_cross, rotation.w)), vec3_cross(axis, twice_cross)));
}

// Shortest rotation taking unit vector 'from' onto 'to'; opposite vectors turn 180 degrees about any perpendicular.
Quat	quat_from_to(Vec3 from, Vec3 to)
{
	float	cosine = vec3_dot(from, to);
	Vec3	axis;

	if (cosine >= 0.999999f)
		return (quat_identity());
	if (cosine <= -0.999999f)
	{
		axis = vec3_cross(vec3(1.0f, 0.0f, 0.0f), from);
		if (vec3_length_sq(axis) < 0.000001f)
			axis = vec3_cross(vec3(0.0f, 1.0f, 0.0f), from);
		return (quat_from_axis_angle(vec3_normalized(axis), FTN_PI));
	}
	axis = vec3_cross(from, to);
	return (quat_normalized((Quat){1.0f + cosine, axis.x, axis.y, axis.z}));
}

Quat	quat_integrate(Quat rotation, Vec3 angular_velocity, float dt)
{
	Quat	spin = {0.0f, angular_velocity.x, angular_velocity.y, angular_velocity.z};
	Quat	derivative = quat_mul(spin, rotation);   // [F3] dq/dt = 0.5 (0, w) q
	float	half_dt = 0.5f * dt;

	rotation.w += derivative.w * half_dt;
	rotation.x += derivative.x * half_dt;
	rotation.y += derivative.y * half_dt;
	rotation.z += derivative.z * half_dt;
	return (quat_normalized(rotation));
}
