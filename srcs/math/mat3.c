
#include "newton.h"

/*
 * [TODO] 3x3 matrix math, mainly for the inertia tensor and its world-space
 * rotation I_world = R * I_local * R^T. Column-major:
 * element (col c, row r) lives at m[c * 3 + r].
 *
 * identity() and diagonal() are provided (trivial constructors). The rest are
 * placeholders that you must implement before the rotational physics works.
*/

Mat3	mat3_identity(void)
{
	Mat3	r = {{0}};

	r.m[0] = 1.0f;
	r.m[4] = 1.0f;
	r.m[8] = 1.0f;
	return (r);
}

Mat3	mat3_diagonal(Vec3 d)
{
	Mat3	r = {{0}};

	r.m[0] = d.x;
	r.m[4] = d.y;
	r.m[8] = d.z;
	return (r);
}

Mat3	mat3_transpose(Mat3 a)
{
	/* TODO: swap m[c*3 + r] with m[r*3 + c]. Placeholder returns the input. */
	return (a);
}

Mat3	mat3_inverse(Mat3 a)
{
	/* TODO: 3x3 inverse via cofactors / determinant. Needed for the inertia
	 * tensor. Placeholder returns the input. */
	return (a);
}

Mat3	mat3_mul(Mat3 a, Mat3 b)
{
	/* TODO: standard 3x3 * 3x3 product. Placeholder returns identity. */
	(void)a;
	(void)b;
	return (mat3_identity());
}

Vec3	mat3_mul_vec3(Mat3 a, Vec3 v)
{
	/* TODO: 3x3 * vec3. Placeholder returns the input vector. */
	(void)a;
	return (v);
}
