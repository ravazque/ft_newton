#include "newton.h"

// A model on the scene: its surface from the library, mass = density x volume, standing on what occupies its spot.

int	model_spawn(World *world, const ObjectDef *def, const MeshShape *shape, Vec3 origin)
{
	RigidBody	body = rigidbody_make();
	int			index = world->bodyCount;

	body.kind = KIND_MODEL;
	body.position = vec3(origin.x, origin.y - shape->boundsMin.y, origin.z);
	body.collider = collider_mesh(shape);
	body.friction = def->friction;
	body.elasticity = def->elasticity;
	body.color = def->color;
	rigidbody_set_mass(&body, def->density * shape->volume);
	if (placement_add(world, &body, 1) < 0.0f)
		return (-1);
	return (index);
}

static int	overlaps_another(const World *world, int index)
{
	const RigidBody	*body = &world->bodies[index];
	Vec3			body_min;
	Vec3			body_max;
	Vec3			other_min;
	Vec3			other_max;
	int				i;

	collider_bounds(body, &body_min, &body_max);
	i = 0;
	while (i < world->bodyCount)
	{
		collider_bounds(&world->bodies[i], &other_min, &other_max);
		if (i != index && body_min.x <= other_max.x && other_min.x <= body_max.x && body_min.y <= other_max.y && other_min.y <= body_max.y && body_min.z <= other_max.z && other_min.z <= body_max.z && overlap_bodies(body, &world->bodies[i]))
			return (1);
		i++;
	}
	return (0);
}

// A resized model keeps its lowest point where it was; one that would then sink into something is lifted onto it.
void	model_resize_all(World *world, ModelLibrary *library, float size)
{
	float		*lowest = malloc((size_t)world->bodyCount * sizeof(float) + 1);
	Vec3		bounds_min;
	Vec3		bounds_max;
	RigidBody	moved;
	int			i;

	if (!lowest)
	{
		fprintf(stderr, "Out of memory\n");
		return ;
	}
	i = 0;
	while (i < world->bodyCount)
	{
		collider_bounds(&world->bodies[i], &bounds_min, &bounds_max);
		lowest[i] = bounds_min.y;
		i++;
	}
	model_library_resize(library, size);
	i = 0;
	while (i < world->bodyCount)
	{
		if (world->bodies[i].kind == KIND_MODEL)
		{
			world->bodies[i].collider = collider_mesh(world->bodies[i].collider.mesh);
			collider_bounds(&world->bodies[i], &bounds_min, &bounds_max);
			world->bodies[i].position.y += lowest[i] - bounds_min.y;
		}
		i++;
	}
	free(lowest);
	i = world->bodyCount - 1;
	while (i >= 0)
	{
		if (world->bodies[i].kind == KIND_MODEL && overlaps_another(world, i))
		{
			moved = world->bodies[i];
			world_remove_body(world, i);
			placement_add(world, &moved, 1);
		}
		i--;
	}
}
