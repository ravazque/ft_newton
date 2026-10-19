#include "newton.h"

// Column-major 4x4 matrices for rendering: model transforms, camera view and perspective projection.

Mat4	mat4_identity(void)
{
	Mat4	result = {{0}};

	result.cells[0] = 1.0f;
	result.cells[5] = 1.0f;
	result.cells[10] = 1.0f;
	result.cells[15] = 1.0f;
	return (result);
}

Mat4	mat4_translation(Vec3 offset)
{
	Mat4	result = mat4_identity();

	result.cells[12] = offset.x;
	result.cells[13] = offset.y;
	result.cells[14] = offset.z;
	return (result);
}

Mat4	mat4_scale(Vec3 factors)
{
	Mat4	result = {{0}};

	result.cells[0] = factors.x;
	result.cells[5] = factors.y;
	result.cells[10] = factors.z;
	result.cells[15] = 1.0f;
	return (result);
}

Mat4	mat4_from_quat(Quat rotation)
{
	Mat4	result = {{0}};
	float	xx = rotation.x * rotation.x;
	float	yy = rotation.y * rotation.y;
	float	zz = rotation.z * rotation.z;
	float	xy = rotation.x * rotation.y;
	float	xz = rotation.x * rotation.z;
	float	yz = rotation.y * rotation.z;
	float	wx = rotation.w * rotation.x;
	float	wy = rotation.w * rotation.y;
	float	wz = rotation.w * rotation.z;

	result.cells[0] = 1.0f - 2.0f * (yy + zz);
	result.cells[1] = 2.0f * (xy + wz);
	result.cells[2] = 2.0f * (xz - wy);
	result.cells[4] = 2.0f * (xy - wz);
	result.cells[5] = 1.0f - 2.0f * (xx + zz);
	result.cells[6] = 2.0f * (yz + wx);
	result.cells[8] = 2.0f * (xz + wy);
	result.cells[9] = 2.0f * (yz - wx);
	result.cells[10] = 1.0f - 2.0f * (xx + yy);
	result.cells[15] = 1.0f;
	return (result);
}

Mat4	mat4_mul(Mat4 a, Mat4 b)
{
	Mat4	result = {{0}};
	int		col;
	int		row;
	int		k;

	col = 0;
	while (col < 4)
	{
		row = 0;
		while (row < 4)
		{
			k = 0;
			while (k < 4)
			{
				result.cells[col * 4 + row] += a.cells[k * 4 + row] * b.cells[col * 4 + k];
				k++;
			}
			row++;
		}
		col++;
	}
	return (result);
}

Mat4	mat4_transform(Vec3 position, Quat rotation, Vec3 scale)
{
	return (mat4_mul(mat4_mul(mat4_translation(position), mat4_from_quat(rotation)), mat4_scale(scale)));
}

Mat4	mat4_perspective(float fovy_radians, float aspect, float near_plane, float far_plane)
{
	Mat4	result = {{0}};
	float	focal;

	focal = 1.0f / tanf(fovy_radians * 0.5f);
	result.cells[0] = focal / aspect;
	result.cells[5] = focal;
	result.cells[10] = (far_plane + near_plane) / (near_plane - far_plane);
	result.cells[11] = -1.0f;
	result.cells[14] = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
	return (result);
}

Mat4	mat4_look_at(Vec3 eye, Vec3 target, Vec3 up)
{
	Mat4	result = mat4_identity();
	Vec3	forward = vec3_normalized(vec3_sub(target, eye));
	Vec3	side = vec3_normalized(vec3_cross(forward, up));
	Vec3	upward = vec3_cross(side, forward);

	result.cells[0] = side.x;
	result.cells[4] = side.y;
	result.cells[8] = side.z;
	result.cells[1] = upward.x;
	result.cells[5] = upward.y;
	result.cells[9] = upward.z;
	result.cells[2] = -forward.x;
	result.cells[6] = -forward.y;
	result.cells[10] = -forward.z;
	result.cells[12] = -vec3_dot(side, eye);
	result.cells[13] = -vec3_dot(upward, eye);
	result.cells[14] = vec3_dot(forward, eye);
	return (result);
}
