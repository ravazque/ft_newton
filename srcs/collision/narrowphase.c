#include "newton.h"

// The exact test of each pair, lower shape first; a pair given the other way round gets its normals flipped back.

static int	flip_contacts(int count, Contact *contacts)
{
	int	i;

	i = 0;
	while (i < count)
	{
		contacts[i].normal = vec3_neg(contacts[i].normal);
		i++;
	}
	return (count);
}

static int	ordered_pair(const RigidBody *body_a, const RigidBody *body_b, Contact *out)
{
	ShapeType	shape_a = body_a->collider.type;
	ShapeType	shape_b = body_b->collider.type;

	if (shape_a == SHAPE_SPHERE && shape_b == SHAPE_SPHERE)
		return (contact_sphere_sphere(body_a, body_b, out));
	if (shape_a == SHAPE_SPHERE && shape_b == SHAPE_BOX)
		return (contact_sphere_box(body_a, body_b, out));
	if (shape_a == SHAPE_SPHERE && shape_b == SHAPE_MESH)
		return (contact_sphere_mesh(body_a, body_b, out));
	if (shape_a == SHAPE_SPHERE && shape_b == SHAPE_PLANE)
		return (contact_sphere_plane(body_a, body_b, out));
	if (shape_a == SHAPE_BOX && shape_b == SHAPE_BOX)
		return (contact_box_box(body_a, body_b, out));
	if (shape_a == SHAPE_BOX && shape_b == SHAPE_MESH)
		return (contact_box_mesh(body_a, body_b, out));
	if (shape_a == SHAPE_BOX && shape_b == SHAPE_PLANE)
		return (contact_box_plane(body_a, body_b, out));
	if (shape_a == SHAPE_MESH && shape_b == SHAPE_MESH)
		return (contact_mesh_mesh(body_a, body_b, out));
	if (shape_a == SHAPE_MESH && shape_b == SHAPE_PLANE)
		return (contact_mesh_plane(body_a, body_b, out));
	return (0);
}

int	narrowphase_pair(const RigidBody *body_a, const RigidBody *body_b, Contact *out)
{
	if (body_a->collider.type > body_b->collider.type)
		return (flip_contacts(ordered_pair(body_b, body_a, out), out));
	return (ordered_pair(body_a, body_b, out));
}

static int	push_contacts(World *world, const Pair *pair, const Contact *found, int count)
{
	Contact	*grown;
	int		new_capacity;
	int		i;

	while (world->contactCount + count > world->contactCapacity)
	{
		new_capacity = 128;
		if (world->contactCapacity > 0)
			new_capacity = world->contactCapacity * 2;
		grown = realloc(world->contacts, (size_t)new_capacity * sizeof(Contact));
		if (!grown)
			return (0);
		world->contacts = grown;
		world->contactCapacity = new_capacity;
	}
	i = 0;
	while (i < count)
	{
		world->contacts[world->contactCount] = found[i];
		world->contacts[world->contactCount].bodyA = pair->bodyA;
		world->contacts[world->contactCount].bodyB = pair->bodyB;
		world->contactCount++;
		i++;
	}
	return (1);
}

void	narrowphase_generate_contacts(World *world)
{
	Contact	found[MAX_CONTACTS_PER_PAIR];
	int		count;
	int		i;

	world->contactCount = 0;
	i = 0;
	while (i < world->pairCount)
	{
		count = narrowphase_pair(&world->bodies[world->pairs[i].bodyA], &world->bodies[world->pairs[i].bodyB], found);
		if (count > 0)
			push_contacts(world, &world->pairs[i], found, count);
		i++;
	}
}
