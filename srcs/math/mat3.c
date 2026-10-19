#include "newton.h"

// 3x3 matrices for inertia tensors and joints; mat3_skew(v) is the cross product as a matrix: skew(v) . u = v x u.

Mat3	mat3_identity(void)
{
	Mat3	result = {{0}};

	result.cells[0] = 1.0f;
	result.cells[4] = 1.0f;
	result.cells[8] = 1.0f;
	return (result);
}

Mat3	mat3_zero(void)
{
	Mat3	result = {{0}};

	return (result);
}

Mat3	mat3_diagonal(Vec3 diagonal)
{
	Mat3	result = {{0}};

	result.cells[0] = diagonal.x;
	result.cells[4] = diagonal.y;
	result.cells[8] = diagonal.z;
	return (result);
}

Mat3	mat3_from_quat(Quat rotation)
{
	Mat3	result;

	MAT3_AT(result, 0, 0) = 1.0f - 2.0f * (rotation.y * rotation.y + rotation.z * rotation.z);
	MAT3_AT(result, 0, 1) = 2.0f * (rotation.x * rotation.y - rotation.w * rotation.z);
	MAT3_AT(result, 0, 2) = 2.0f * (rotation.x * rotation.z + rotation.w * rotation.y);
	MAT3_AT(result, 1, 0) = 2.0f * (rotation.x * rotation.y + rotation.w * rotation.z);
	MAT3_AT(result, 1, 1) = 1.0f - 2.0f * (rotation.x * rotation.x + rotation.z * rotation.z);
	MAT3_AT(result, 1, 2) = 2.0f * (rotation.y * rotation.z - rotation.w * rotation.x);
	MAT3_AT(result, 2, 0) = 2.0f * (rotation.x * rotation.z - rotation.w * rotation.y);
	MAT3_AT(result, 2, 1) = 2.0f * (rotation.y * rotation.z + rotation.w * rotation.x);
	MAT3_AT(result, 2, 2) = 1.0f - 2.0f * (rotation.x * rotation.x + rotation.y * rotation.y);
	return (result);
}

Mat3	mat3_transpose(Mat3 matrix)
{
	Mat3	result;
	int		row;
	int		col;

	row = 0;
	while (row < 3)
	{
		col = 0;
		while (col < 3)
		{
			MAT3_AT(result, row, col) = MAT3_AT(matrix, col, row);
			col++;
		}
		row++;
	}
	return (result);
}

// Cofactor expansion; a matrix singular for its own scale inverts to zero, which is what a static body needs.
Mat3	mat3_inverse(Mat3 matrix)
{
	Mat3	result = {{0}};
	float	cofactor0 = MAT3_AT(matrix, 1, 1) * MAT3_AT(matrix, 2, 2) - MAT3_AT(matrix, 1, 2) * MAT3_AT(matrix, 2, 1);
	float	cofactor1 = MAT3_AT(matrix, 1, 2) * MAT3_AT(matrix, 2, 0) - MAT3_AT(matrix, 1, 0) * MAT3_AT(matrix, 2, 2);
	float	cofactor2 = MAT3_AT(matrix, 1, 0) * MAT3_AT(matrix, 2, 1) - MAT3_AT(matrix, 1, 1) * MAT3_AT(matrix, 2, 0);
	float	determinant = MAT3_AT(matrix, 0, 0) * cofactor0 + MAT3_AT(matrix, 0, 1) * cofactor1 + MAT3_AT(matrix, 0, 2) * cofactor2;
	float	largest = 0.0f;
	float	inverse;
	int		i;

	i = 0;
	while (i < 9)
		largest = fmaxf(largest, fabsf(matrix.cells[i++]));
	if (!(fabsf(determinant) > 1e-9f * largest * largest * largest))
		return (result);
	inverse = 1.0f / determinant;
	MAT3_AT(result, 0, 0) = cofactor0 * inverse;
	MAT3_AT(result, 1, 0) = cofactor1 * inverse;
	MAT3_AT(result, 2, 0) = cofactor2 * inverse;
	MAT3_AT(result, 0, 1) = (MAT3_AT(matrix, 0, 2) * MAT3_AT(matrix, 2, 1) - MAT3_AT(matrix, 0, 1) * MAT3_AT(matrix, 2, 2)) * inverse;
	MAT3_AT(result, 1, 1) = (MAT3_AT(matrix, 0, 0) * MAT3_AT(matrix, 2, 2) - MAT3_AT(matrix, 0, 2) * MAT3_AT(matrix, 2, 0)) * inverse;
	MAT3_AT(result, 2, 1) = (MAT3_AT(matrix, 0, 1) * MAT3_AT(matrix, 2, 0) - MAT3_AT(matrix, 0, 0) * MAT3_AT(matrix, 2, 1)) * inverse;
	MAT3_AT(result, 0, 2) = (MAT3_AT(matrix, 0, 1) * MAT3_AT(matrix, 1, 2) - MAT3_AT(matrix, 0, 2) * MAT3_AT(matrix, 1, 1)) * inverse;
	MAT3_AT(result, 1, 2) = (MAT3_AT(matrix, 0, 2) * MAT3_AT(matrix, 1, 0) - MAT3_AT(matrix, 0, 0) * MAT3_AT(matrix, 1, 2)) * inverse;
	MAT3_AT(result, 2, 2) = (MAT3_AT(matrix, 0, 0) * MAT3_AT(matrix, 1, 1) - MAT3_AT(matrix, 0, 1) * MAT3_AT(matrix, 1, 0)) * inverse;
	return (result);
}

Mat3	mat3_mul(Mat3 a, Mat3 b)
{
	Mat3	result = {{0}};
	int		row;
	int		col;
	int		k;

	row = 0;
	while (row < 3)
	{
		col = 0;
		while (col < 3)
		{
			k = 0;
			while (k < 3)
			{
				MAT3_AT(result, row, col) += MAT3_AT(a, row, k) * MAT3_AT(b, k, col);
				k++;
			}
			col++;
		}
		row++;
	}
	return (result);
}

Vec3	mat3_mul_vec3(Mat3 matrix, Vec3 vector)
{
	Vec3	result;

	result.x = MAT3_AT(matrix, 0, 0) * vector.x + MAT3_AT(matrix, 0, 1) * vector.y + MAT3_AT(matrix, 0, 2) * vector.z;
	result.y = MAT3_AT(matrix, 1, 0) * vector.x + MAT3_AT(matrix, 1, 1) * vector.y + MAT3_AT(matrix, 1, 2) * vector.z;
	result.z = MAT3_AT(matrix, 2, 0) * vector.x + MAT3_AT(matrix, 2, 1) * vector.y + MAT3_AT(matrix, 2, 2) * vector.z;
	return (result);
}

Mat3	mat3_add(Mat3 a, Mat3 b)
{
	int	i;

	i = 0;
	while (i < 9)
	{
		a.cells[i] += b.cells[i];
		i++;
	}
	return (a);
}

Mat3	mat3_scale(Mat3 matrix, float factor)
{
	int	i;

	i = 0;
	while (i < 9)
	{
		matrix.cells[i] *= factor;
		i++;
	}
	return (matrix);
}

Mat3	mat3_skew(Vec3 vector)
{
	Mat3	result = {{0}};

	MAT3_AT(result, 0, 1) = -vector.z;
	MAT3_AT(result, 0, 2) = vector.y;
	MAT3_AT(result, 1, 0) = vector.z;
	MAT3_AT(result, 1, 2) = -vector.x;
	MAT3_AT(result, 2, 0) = -vector.y;
	MAT3_AT(result, 2, 1) = vector.x;
	return (result);
}
