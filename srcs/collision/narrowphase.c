#include "newton.h"

/*
 * Narrow-phase: the exact test for each broad-phase pair, dispatched on the two
 * shapes. Every generator writes its normal from its first body toward its
 * second, so swapped arguments get their normals flipped back to "a toward b".
*/

static int	flip_contacts(int count, Contact *c)
{
	int	i;

	i = 0;
	while (i < count)
	{
		c[i].normal = vec3_neg(c[i].normal);
		i++;
	}
	return (count);
}

int	narrowphase_pair(const RigidBody *a, const RigidBody *b, Contact *out)
{
	ShapeType	ta = a->collider.type;
	ShapeType	tb = b->collider.type;

	if (ta == SHAPE_SPHERE && tb == SHAPE_SPHERE)
		return (contact_sphere_sphere(a, b, out));
	if (ta == SHAPE_SPHERE && tb == SHAPE_PLANE)
		return (contact_sphere_plane(a, b, out));
	if (ta == SHAPE_PLANE && tb == SHAPE_SPHERE)
		return (flip_contacts(contact_sphere_plane(b, a, out), out));
	if (ta == SHAPE_BOX && tb == SHAPE_PLANE)
		return (contact_box_plane(a, b, out));
	if (ta == SHAPE_PLANE && tb == SHAPE_BOX)
		return (flip_contacts(contact_box_plane(b, a, out), out));
	if (ta == SHAPE_SPHERE && tb == SHAPE_BOX)
		return (contact_sphere_box(a, b, out));
	if (ta == SHAPE_BOX && tb == SHAPE_SPHERE)
		return (flip_contacts(contact_sphere_box(b, a, out), out));
	if (ta == SHAPE_BOX && tb == SHAPE_BOX)
		return (contact_box_box(a, b, out));
	return (0);
}

static int	push_contacts(World *w, const Pair *p, const Contact *buf, int count)
{
	Contact	*grown;
	int		new_capacity;
	int		i;

	while (w->contactCount + count > w->contactCapacity)
	{
		new_capacity = 128;
		if (w->contactCapacity > 0)
			new_capacity = w->contactCapacity * 2;
		grown = realloc(w->contacts, (size_t)new_capacity * sizeof(Contact));
		if (!grown)
			return (0);
		w->contacts = grown;
		w->contactCapacity = new_capacity;
	}
	i = 0;
	while (i < count)
	{
		w->contacts[w->contactCount] = buf[i];
		w->contacts[w->contactCount].a = p->a;
		w->contacts[w->contactCount].b = p->b;
		w->contactCount++;
		i++;
	}
	return (1);
}

void	narrowphase_generate_contacts(World *w)
{
	Contact	buf[MAX_CONTACTS_PER_PAIR];
	int		found;
	int		i;

	w->contactCount = 0;
	i = 0;
	while (i < w->pairCount)
	{
		found = narrowphase_pair(&w->bodies[w->pairs[i].a], &w->bodies[w->pairs[i].b], buf);
		if (found > 0)
			push_contacts(w, &w->pairs[i], buf, found);
		i++;
	}
}
