#include "newton.h"

// Closest points on a triangle [F29] and between two segments (Ericson 5.1.5, 5.1.9), and where a segment crosses a triangle [F37].

// 'feature' tells where the point lies: 0 inside the face, 1 to 3 on corner a, b or c, 4 to 6 on edge ab, bc or ca.
Vec3	closest_on_triangle(Vec3 point, Vec3 a, Vec3 b, Vec3 c, int *feature)
{
	Vec3	ab = vec3_sub(b, a);
	Vec3	ac = vec3_sub(c, a);
	float	d1 = vec3_dot(ab, vec3_sub(point, a));
	float	d2 = vec3_dot(ac, vec3_sub(point, a));
	float	d3 = vec3_dot(ab, vec3_sub(point, b));
	float	d4 = vec3_dot(ac, vec3_sub(point, b));
	float	d5 = vec3_dot(ab, vec3_sub(point, c));
	float	d6 = vec3_dot(ac, vec3_sub(point, c));
	float	va = d3 * d6 - d5 * d4;
	float	vb = d5 * d2 - d1 * d6;
	float	vc = d1 * d4 - d3 * d2;

	*feature = 1;
	if (d1 <= 0.0f && d2 <= 0.0f)
		return (a);
	*feature = 2;
	if (d3 >= 0.0f && d4 <= d3)
		return (b);
	*feature = 4;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
		return (vec3_add(a, vec3_scale(ab, d1 / (d1 - d3))));
	*feature = 3;
	if (d6 >= 0.0f && d5 <= d6)
		return (c);
	*feature = 6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
		return (vec3_add(a, vec3_scale(ac, d2 / (d2 - d6))));
	*feature = 5;
	if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
		return (vec3_add(b, vec3_scale(vec3_sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)))));
	*feature = 0;
	return (vec3_add(a, vec3_add(vec3_scale(ab, vb / (va + vb + vc)), vec3_scale(ac, vc / (va + vb + vc)))));   // [F29]
}

void	closest_on_segments(Vec3 start1, Vec3 end1, Vec3 start2, Vec3 end2, Vec3 *closest1, Vec3 *closest2)
{
	Vec3	direction1 = vec3_sub(end1, start1);
	Vec3	direction2 = vec3_sub(end2, start2);
	Vec3	between = vec3_sub(start1, start2);
	float	length1_sq = vec3_dot(direction1, direction1);
	float	length2_sq = vec3_dot(direction2, direction2);
	float	along2 = vec3_dot(direction2, between);
	float	along1 = vec3_dot(direction1, between);
	float	cosine = vec3_dot(direction1, direction2);
	float	denominator = length1_sq * length2_sq - cosine * cosine;
	float	s;
	float	t;

	s = 0.0f;
	if (denominator > 0.000001f)
		s = clampf((cosine * along2 - along1 * length2_sq) / denominator, 0.0f, 1.0f);
	t = (cosine * s + along2) / length2_sq;
	if (t < 0.0f || t > 1.0f)
	{
		t = clampf(t, 0.0f, 1.0f);
		s = 0.0f;
		if (length1_sq > 0.000001f)
			s = clampf((cosine * t - along1) / length1_sq, 0.0f, 1.0f);
	}
	*closest1 = vec3_add(start1, vec3_scale(direction1, s));
	*closest2 = vec3_add(start2, vec3_scale(direction2, t));
}

int	segment_crosses_triangle(Vec3 start, Vec3 end, const Vec3 *corner, float *fraction)
{
	Vec3	direction = vec3_sub(end, start);
	Vec3	edge1 = vec3_sub(corner[1], corner[0]);
	Vec3	edge2 = vec3_sub(corner[2], corner[0]);
	Vec3	across = vec3_cross(direction, edge2);
	float	determinant = vec3_dot(edge1, across);
	Vec3	offset;
	Vec3	turned;
	float	u;
	float	v;

	if (fabsf(determinant) < 1e-12f)
		return (0);
	offset = vec3_sub(start, corner[0]);
	u = vec3_dot(offset, across) / determinant;   // [F37]
	if (u < 0.0f || u > 1.0f)
		return (0);
	turned = vec3_cross(offset, edge1);
	v = vec3_dot(direction, turned) / determinant;
	if (v < 0.0f || u + v > 1.0f)
		return (0);
	*fraction = vec3_dot(edge2, turned) / determinant;
	return (*fraction >= 0.0f && *fraction <= 1.0f);
}
