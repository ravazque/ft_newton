#include "newton.h"

// Collider shapes and their world bounds; a plane is solid only over its square and PLANE_THICKNESS under it.

Collider	collider_sphere(float radius)
{
	Collider	collider;

	memset(&collider, 0, sizeof(collider));
	collider.type = SHAPE_SPHERE;
	collider.radius = radius;
	collider.halfExtents = vec3(radius, radius, radius);
	collider.normal = vec3(0.0f, 1.0f, 0.0f);
	return (collider);
}

Collider	collider_box(Vec3 half_extents)
{
	Collider	collider;

	memset(&collider, 0, sizeof(collider));
	collider.type = SHAPE_BOX;
	collider.halfExtents = half_extents;
	collider.normal = vec3(0.0f, 1.0f, 0.0f);
	return (collider);
}

Collider	collider_plane(Vec3 normal, float offset, float size)
{
	Collider	collider;

	memset(&collider, 0, sizeof(collider));
	collider.type = SHAPE_PLANE;
	collider.normal = vec3_normalized(normal);
	collider.offset = offset;
	collider.halfSize = size * 0.5f;
	return (collider);
}

Collider	collider_mesh(const MeshShape *mesh)
{
	Collider	collider;

	memset(&collider, 0, sizeof(collider));
	collider.type = SHAPE_MESH;
	collider.mesh = mesh;
	collider.halfExtents = vec3_scale(vec3_sub(mesh->boundsMax, mesh->boundsMin), 0.5f);
	collider.normal = vec3(0.0f, 1.0f, 0.0f);
	return (collider);
}

Vec3	collider_plane_origin(const Collider *collider)
{
	return (vec3_scale(collider->normal, collider->offset));
}

Quat	collider_plane_rotation(const Collider *collider)
{
	return (quat_from_to(vec3(0.0f, 1.0f, 0.0f), collider->normal));
}

float	collider_plane_distance(const Collider *collider, Vec3 point)
{
	return (vec3_dot(collider->normal, point) - collider->offset);   // [F13] signed distance
}

int	collider_plane_covers(const Collider *collider, Vec3 point)
{
	Quat	rotation = collider_plane_rotation(collider);
	Vec3	offset = vec3_sub(point, collider_plane_origin(collider));
	float	along_x = vec3_dot(offset, quat_rotate(rotation, vec3(1.0f, 0.0f, 0.0f)));
	float	along_z = vec3_dot(offset, quat_rotate(rotation, vec3(0.0f, 0.0f, 1.0f)));

	return (fabsf(along_x) <= collider->halfSize && fabsf(along_z) <= collider->halfSize && collider_plane_distance(collider, point) >= -PLANE_THICKNESS);   // [F13]
}

static void	oriented_bounds(Vec3 center, Quat rotation, Vec3 half, Vec3 *bounds_min, Vec3 *bounds_max)
{
	Vec3	axis_x = quat_rotate(rotation, vec3(1.0f, 0.0f, 0.0f));
	Vec3	axis_y = quat_rotate(rotation, vec3(0.0f, 1.0f, 0.0f));
	Vec3	axis_z = quat_rotate(rotation, vec3(0.0f, 0.0f, 1.0f));
	Vec3	extent;

	extent.x = fabsf(axis_x.x) * half.x + fabsf(axis_y.x) * half.y + fabsf(axis_z.x) * half.z;   // [F10]
	extent.y = fabsf(axis_x.y) * half.x + fabsf(axis_y.y) * half.y + fabsf(axis_z.y) * half.z;
	extent.z = fabsf(axis_x.z) * half.x + fabsf(axis_y.z) * half.y + fabsf(axis_z.z) * half.z;
	*bounds_min = vec3_sub(center, extent);
	*bounds_max = vec3_add(center, extent);
}

void	collider_bounds(const RigidBody *body, Vec3 *bounds_min, Vec3 *bounds_max)
{
	const Collider	*collider = &body->collider;
	float			half_depth = PLANE_THICKNESS * 0.5f;
	Vec3			radius;
	Vec3			mesh_center;

	if (collider->type == SHAPE_SPHERE)
	{
		radius = vec3(collider->radius, collider->radius, collider->radius);
		*bounds_min = vec3_sub(body->position, radius);
		*bounds_max = vec3_add(body->position, radius);
	}
	else if (collider->type == SHAPE_BOX)
		oriented_bounds(body->position, body->orientation, collider->halfExtents, bounds_min, bounds_max);
	else if (collider->type == SHAPE_MESH)
	{
		mesh_center = mesh_to_world(body, vec3_scale(vec3_add(collider->mesh->boundsMin, collider->mesh->boundsMax), 0.5f));
		oriented_bounds(mesh_center, body->orientation, collider->halfExtents, bounds_min, bounds_max);
	}
	else
		oriented_bounds(vec3_sub(collider_plane_origin(collider), vec3_scale(collider->normal, half_depth)), collider_plane_rotation(collider), vec3(collider->halfSize, half_depth, collider->halfSize), bounds_min, bounds_max);
}
