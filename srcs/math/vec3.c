#include "newton.h"

// 3D vectors: positions, velocities, forces, torques and normals.

Vec3	vec3(float x, float y, float z)
{
	return ((Vec3){x, y, z});
}

Vec3	vec3_add(Vec3 a, Vec3 b)
{
	return ((Vec3){a.x + b.x, a.y + b.y, a.z + b.z});
}

Vec3	vec3_sub(Vec3 a, Vec3 b)
{
	return ((Vec3){a.x - b.x, a.y - b.y, a.z - b.z});
}

Vec3	vec3_neg(Vec3 vector)
{
	return ((Vec3){-vector.x, -vector.y, -vector.z});
}

Vec3	vec3_scale(Vec3 vector, float factor)
{
	return ((Vec3){vector.x * factor, vector.y * factor, vector.z * factor});
}

float	vec3_dot(Vec3 a, Vec3 b)
{
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}

Vec3	vec3_cross(Vec3 a, Vec3 b)
{
	return ((Vec3){
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x});
}

float	vec3_length_sq(Vec3 vector)
{
	return (vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

float	vec3_length(Vec3 vector)
{
	return (sqrtf(vec3_length_sq(vector)));
}

Vec3	vec3_normalized(Vec3 vector)
{
	float	length;

	length = vec3_length(vector);
	if (length > 0.0f)
		return (vec3_scale(vector, 1.0f / length));
	return ((Vec3){0.0f, 0.0f, 0.0f});
}

Vec3	vec3_min(Vec3 a, Vec3 b)
{
	return ((Vec3){fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z)});
}

Vec3	vec3_max(Vec3 a, Vec3 b)
{
	return ((Vec3){fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z)});
}
