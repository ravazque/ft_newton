#include "newton.h"

// Overlap tests outside the step, for placing new bodies: touching is allowed, sinking deeper than SPAWN_TOLERANCE is not.

// A body swallowed whole by a mesh touches none of its triangles: its centre inside the solid [F30] still counts.
static int	center_inside_mesh(const RigidBody *mesh, const RigidBody *other)
{
	const MeshShape	*shape = mesh->collider.mesh;
	Vec3			local;
	MeshHit			hit;

	if (mesh->collider.type != SHAPE_MESH || other->collider.type == SHAPE_PLANE)
		return (0);
	local = mesh_to_local(mesh, other->position);
	if (local.x < shape->boundsMin.x || local.x > shape->boundsMax.x || local.y < shape->boundsMin.y || local.y > shape->boundsMax.y || local.z < shape->boundsMin.z || local.z > shape->boundsMax.z)
		return (0);
	return (mesh_closest_point(shape, local, shape->size, &hit) && hit.inside && hit.distance > SPAWN_TOLERANCE);
}

int	overlap_bodies(const RigidBody *body_a, const RigidBody *body_b)
{
	Contact	found[MAX_CONTACTS_PER_PAIR];
	int		count;
	int		i;

	count = narrowphase_pair(body_a, body_b, found);
	i = 0;
	while (i < count)
	{
		if (found[i].penetration > SPAWN_TOLERANCE)
			return (1);
		i++;
	}
	return (center_inside_mesh(body_a, body_b) || center_inside_mesh(body_b, body_a));
}

static int	bounds_overlap(Vec3 min_a, Vec3 max_a, Vec3 min_b, Vec3 max_b)
{
	return (min_a.x <= max_b.x && min_b.x <= max_a.x && min_a.y <= max_b.y && min_b.y <= max_a.y && min_a.z <= max_b.z && min_b.z <= max_a.z);   // [F11]
}

int	overlap_find(const World *world, const RigidBody *candidate)
{
	Vec3	candidate_min;
	Vec3	candidate_max;
	Vec3	other_min;
	Vec3	other_max;
	int		i;

	collider_bounds(candidate, &candidate_min, &candidate_max);
	i = 0;
	while (i < world->bodyCount)
	{
		collider_bounds(&world->bodies[i], &other_min, &other_max);
		if (bounds_overlap(candidate_min, candidate_max, other_min, other_max) && overlap_bodies(candidate, &world->bodies[i]))
			return (i);
		i++;
	}
	return (-1);
}
