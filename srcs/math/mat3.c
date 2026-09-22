#include "newton.h"

/* 3x3 matrices for the inertia tensor (I_world = R I_local R^T). Column-major; M() reads as (row, col). */

#define M(mat, row, col) ((mat).m[(col) * 3 + (row)])

Mat3	mat3_identity(void)
{
	Mat3	r = {{0}};

	r.m[0] = 1.0f;
	r.m[4] = 1.0f;
	r.m[8] = 1.0f;
	return (r);
}

Mat3	mat3_zero(void)
{
	Mat3	r = {{0}};

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

Mat3	mat3_from_quat(Quat q)
{
	Mat3	r;

	M(r, 0, 0) = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	M(r, 0, 1) = 2.0f * (q.x * q.y - q.w * q.z);
	M(r, 0, 2) = 2.0f * (q.x * q.z + q.w * q.y);
	M(r, 1, 0) = 2.0f * (q.x * q.y + q.w * q.z);
	M(r, 1, 1) = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
	M(r, 1, 2) = 2.0f * (q.y * q.z - q.w * q.x);
	M(r, 2, 0) = 2.0f * (q.x * q.z - q.w * q.y);
	M(r, 2, 1) = 2.0f * (q.y * q.z + q.w * q.x);
	M(r, 2, 2) = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
	return (r);
}

Mat3	mat3_transpose(Mat3 a)
{
	Mat3	r;
	int		row;
	int		col;

	row = 0;
	while (row < 3)
	{
		col = 0;
		while (col < 3)
		{
			M(r, row, col) = M(a, col, row);
			col++;
		}
		row++;
	}
	return (r);
}

/* Cofactor expansion; a singular matrix inverts to zero, which is what a static body needs. */
Mat3	mat3_inverse(Mat3 a)
{
	Mat3	r = {{0}};
	float	c00 = M(a, 1, 1) * M(a, 2, 2) - M(a, 1, 2) * M(a, 2, 1);
	float	c01 = M(a, 1, 2) * M(a, 2, 0) - M(a, 1, 0) * M(a, 2, 2);
	float	c02 = M(a, 1, 0) * M(a, 2, 1) - M(a, 1, 1) * M(a, 2, 0);
	float	det = M(a, 0, 0) * c00 + M(a, 0, 1) * c01 + M(a, 0, 2) * c02;
	float	inv;

	if (fabsf(det) < 1e-12f)
		return (r);
	inv = 1.0f / det;
	M(r, 0, 0) = c00 * inv;
	M(r, 1, 0) = c01 * inv;
	M(r, 2, 0) = c02 * inv;
	M(r, 0, 1) = (M(a, 0, 2) * M(a, 2, 1) - M(a, 0, 1) * M(a, 2, 2)) * inv;
	M(r, 1, 1) = (M(a, 0, 0) * M(a, 2, 2) - M(a, 0, 2) * M(a, 2, 0)) * inv;
	M(r, 2, 1) = (M(a, 0, 1) * M(a, 2, 0) - M(a, 0, 0) * M(a, 2, 1)) * inv;
	M(r, 0, 2) = (M(a, 0, 1) * M(a, 1, 2) - M(a, 0, 2) * M(a, 1, 1)) * inv;
	M(r, 1, 2) = (M(a, 0, 2) * M(a, 1, 0) - M(a, 0, 0) * M(a, 1, 2)) * inv;
	M(r, 2, 2) = (M(a, 0, 0) * M(a, 1, 1) - M(a, 0, 1) * M(a, 1, 0)) * inv;
	return (r);
}

Mat3	mat3_mul(Mat3 a, Mat3 b)
{
	Mat3	r;
	int		row;
	int		col;

	row = 0;
	while (row < 3)
	{
		col = 0;
		while (col < 3)
		{
			M(r, row, col) = M(a, row, 0) * M(b, 0, col) + M(a, row, 1) * M(b, 1, col) + M(a, row, 2) * M(b, 2, col);
			col++;
		}
		row++;
	}
	return (r);
}

Vec3	mat3_mul_vec3(Mat3 a, Vec3 v)
{
	Vec3	r;

	r.x = M(a, 0, 0) * v.x + M(a, 0, 1) * v.y + M(a, 0, 2) * v.z;
	r.y = M(a, 1, 0) * v.x + M(a, 1, 1) * v.y + M(a, 1, 2) * v.z;
	r.z = M(a, 2, 0) * v.x + M(a, 2, 1) * v.y + M(a, 2, 2) * v.z;
	return (r);
}
