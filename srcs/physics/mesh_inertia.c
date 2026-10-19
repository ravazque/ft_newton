#include "newton.h"

// Volume, centre of mass and inertia of the solid a closed surface encloses (density 1), by the divergence theorem.

static void	subexpressions(float w0, float w1, float w2, float *f, float *g)
{
	float	sum01 = w0 + w1;
	float	square0 = w0 * w0;
	float	partial = square0 + w1 * sum01;

	f[0] = sum01 + w2;
	f[1] = partial + w2 * f[0];
	f[2] = w0 * square0 + w1 * partial + w2 * f[1];
	g[0] = f[1] + w0 * (f[0] + w0);
	g[1] = f[1] + w1 * (f[0] + w1);
	g[2] = f[1] + w2 * (f[0] + w2);
}

// integrals: volume, first moments x y z, second moments x2 y2 z2, products xy yz zx.
static void	accumulate(const Vec3 *corner, float *integrals)
{
	Vec3	area_normal = vec3_cross(vec3_sub(corner[1], corner[0]), vec3_sub(corner[2], corner[0]));
	float	fx[3];
	float	fy[3];
	float	fz[3];
	float	gx[3];
	float	gy[3];
	float	gz[3];

	subexpressions(corner[0].x, corner[1].x, corner[2].x, fx, gx);
	subexpressions(corner[0].y, corner[1].y, corner[2].y, fy, gy);
	subexpressions(corner[0].z, corner[1].z, corner[2].z, fz, gz);
	integrals[0] += area_normal.x * fx[0];
	integrals[1] += area_normal.x * fx[1];
	integrals[2] += area_normal.y * fy[1];
	integrals[3] += area_normal.z * fz[1];
	integrals[4] += area_normal.x * fx[2];
	integrals[5] += area_normal.y * fy[2];
	integrals[6] += area_normal.z * fz[2];
	integrals[7] += area_normal.x * (corner[0].y * gx[0] + corner[1].y * gx[1] + corner[2].y * gx[2]);
	integrals[8] += area_normal.y * (corner[0].z * gy[0] + corner[1].z * gy[1] + corner[2].z * gy[2]);
	integrals[9] += area_normal.z * (corner[0].x * gz[0] + corner[1].x * gz[1] + corner[2].x * gz[2]);
}

float	mesh_mass_properties(const MeshShape *mesh, Vec3 *center, Mat3 *inertia)
{
	static const float	weights[10] = {1.0f / 6.0f, 1.0f / 24.0f, 1.0f / 24.0f, 1.0f / 24.0f,
		1.0f / 60.0f, 1.0f / 60.0f, 1.0f / 60.0f, 1.0f / 120.0f, 1.0f / 120.0f, 1.0f / 120.0f};
	float				integrals[10] = {0};
	Vec3				corner[3];
	Vec3				centroid;
	Vec3				moments;
	float				volume;
	int					i;

	i = 0;
	while (i < mesh->triangleCount)
	{
		mesh_triangle_corners(mesh, i, corner);
		accumulate(corner, integrals);
		i++;
	}
	i = 0;
	while (i < 10)
	{
		integrals[i] *= weights[i];
		i++;
	}
	volume = integrals[0];
	if (volume == 0.0f)
		return (0.0f);
	centroid = vec3(integrals[1] / volume, integrals[2] / volume, integrals[3] / volume);   // [F27] centre of mass
	*center = centroid;
	moments.x = integrals[5] + integrals[6] - volume * (centroid.y * centroid.y + centroid.z * centroid.z);   // [F27] parallel axis
	moments.y = integrals[4] + integrals[6] - volume * (centroid.z * centroid.z + centroid.x * centroid.x);
	moments.z = integrals[4] + integrals[5] - volume * (centroid.x * centroid.x + centroid.y * centroid.y);
	*inertia = mat3_diagonal(moments);
	MAT3_AT(*inertia, 1, 0) = -(integrals[7] - volume * centroid.x * centroid.y);
	MAT3_AT(*inertia, 0, 1) = MAT3_AT(*inertia, 1, 0);
	MAT3_AT(*inertia, 2, 1) = -(integrals[8] - volume * centroid.y * centroid.z);
	MAT3_AT(*inertia, 1, 2) = MAT3_AT(*inertia, 2, 1);
	MAT3_AT(*inertia, 2, 0) = -(integrals[9] - volume * centroid.z * centroid.x);
	MAT3_AT(*inertia, 0, 2) = MAT3_AT(*inertia, 2, 0);
	return (volume);
}
