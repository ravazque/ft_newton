
#include "newton.h"

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

Vec3	vec3_neg(Vec3 a)
{
	return ((Vec3){-a.x, -a.y, -a.z});
}

Vec3	vec3_scale(Vec3 a, float s)
{
	return ((Vec3){a.x * s, a.y * s, a.z * s});
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

float	vec3_length_sq(Vec3 a)
{
	return (a.x * a.x + a.y * a.y + a.z * a.z);
}

float	vec3_length(Vec3 a)
{
	return (sqrtf(vec3_length_sq(a)));
}

Vec3	vec3_normalized(Vec3 a)
{
	float	len;

	len = vec3_length(a);
	if (len > 0.0f)
		return (vec3_scale(a, 1.0f / len));
	return ((Vec3){0.0f, 0.0f, 0.0f});
}
