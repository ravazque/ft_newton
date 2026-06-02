
#include "newton.h"

/* All matrices are column-major: element (col c, row r) lives at m[c * 4 + r]. */

Mat4	mat4_identity(void)
{
	Mat4	r = {{0}};

	r.m[0] = 1.0f;
	r.m[5] = 1.0f;
	r.m[10] = 1.0f;
	r.m[15] = 1.0f;
	return (r);
}

Mat4	mat4_translation(Vec3 t)
{
	Mat4	r = mat4_identity();

	r.m[12] = t.x;
	r.m[13] = t.y;
	r.m[14] = t.z;
	return (r);
}

Mat4	mat4_scale(Vec3 s)
{
	Mat4	r = {{0}};

	r.m[0] = s.x;
	r.m[5] = s.y;
	r.m[10] = s.z;
	r.m[15] = 1.0f;
	return (r);
}

Mat4	mat4_from_quat(Quat q)
{
	Mat4	r = {{0}};
	float	xx = q.x * q.x;
	float	yy = q.y * q.y;
	float	zz = q.z * q.z;
	float	xy = q.x * q.y;
	float	xz = q.x * q.z;
	float	yz = q.y * q.z;
	float	wx = q.w * q.x;
	float	wy = q.w * q.y;
	float	wz = q.w * q.z;

	r.m[0] = 1.0f - 2.0f * (yy + zz);
	r.m[1] = 2.0f * (xy + wz);
	r.m[2] = 2.0f * (xz - wy);
	r.m[4] = 2.0f * (xy - wz);
	r.m[5] = 1.0f - 2.0f * (xx + zz);
	r.m[6] = 2.0f * (yz + wx);
	r.m[8] = 2.0f * (xz + wy);
	r.m[9] = 2.0f * (yz - wx);
	r.m[10] = 1.0f - 2.0f * (xx + yy);
	r.m[15] = 1.0f;
	return (r);
}

Mat4	mat4_mul(Mat4 a, Mat4 b)
{
	Mat4	r = {{0}};
	int		c;
	int		row;
	int		k;

	c = 0;
	while (c < 4)
	{
		row = 0;
		while (row < 4)
		{
			k = 0;
			while (k < 4)
			{
				r.m[c * 4 + row] += a.m[k * 4 + row] * b.m[c * 4 + k];
				k++;
			}
			row++;
		}
		c++;
	}
	return (r);
}

Mat4	mat4_transform(Vec3 pos, Quat rot, Vec3 scale)
{
	return (mat4_mul(mat4_mul(mat4_translation(pos), mat4_from_quat(rot)), mat4_scale(scale)));
}

Mat4	mat4_perspective(float fovy_radians, float aspect, float near_p, float far_p)
{
	Mat4	r = {{0}};
	float	f;

	f = 1.0f / tanf(fovy_radians * 0.5f);
	r.m[0] = f / aspect;
	r.m[5] = f;
	r.m[10] = (far_p + near_p) / (near_p - far_p);
	r.m[11] = -1.0f;
	r.m[14] = (2.0f * far_p * near_p) / (near_p - far_p);
	return (r);
}

Mat4	mat4_look_at(Vec3 eye, Vec3 target, Vec3 up)
{
	Mat4	r = mat4_identity();
	Vec3	f = vec3_normalized(vec3_sub(target, eye));
	Vec3	s = vec3_normalized(vec3_cross(f, up));
	Vec3	u = vec3_cross(s, f);

	r.m[0] = s.x;
	r.m[4] = s.y;
	r.m[8] = s.z;
	r.m[1] = u.x;
	r.m[5] = u.y;
	r.m[9] = u.z;
	r.m[2] = -f.x;
	r.m[6] = -f.y;
	r.m[10] = -f.z;
	r.m[12] = -vec3_dot(s, eye);
	r.m[13] = -vec3_dot(u, eye);
	r.m[14] = vec3_dot(f, eye);
	return (r);
}
