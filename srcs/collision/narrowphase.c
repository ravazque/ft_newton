
#include "newton.h"

/*
 * Narrow-phase: the exact second pass. For each broad-phase pair it runs the
 * precise test for that shape combination and appends the resulting contacts
 * (normal + point + penetration) to w->contacts. The resolver consumes them.
 * Box tests live in narrowphase_box.c.
 */

int	contact_sphere_sphere(const RigidBody *a, const RigidBody *b, Contact *out)
{
	Vec3	d;
	float	dist;
	float	sum_r;

	d = vec3_sub(b->position, a->position);
	sum_r = a->collider.radius + b->collider.radius;
	dist = vec3_length(d);
	if (dist > sum_r)
		return (0);
	if (dist > 0.0001f)
		out->normal = vec3_scale(d, 1.0f / dist);
	else
		out->normal = vec3(0.0f, 1.0f, 0.0f);
	out->penetration = sum_r - dist;
	out->point = vec3_add(a->position,
			vec3_scale(out->normal, a->collider.radius - out->penetration * 0.5f));
	return (1);
}

/* The plane is defined by its collider: normal . x = offset (it does not
 * depend on the plane body's position). The sphere touches it when the signed
 * distance from its center to the plane is <= its radius. */
int	contact_sphere_plane(const RigidBody *sphere, const RigidBody *plane, Contact *out)
{
	float	dist;

	dist = vec3_dot(plane->collider.normal, sphere->position)
		- plane->collider.offset;
	if (dist > sphere->collider.radius)
		return (0);
	out->normal = vec3_neg(plane->collider.normal);
	out->penetration = sphere->collider.radius - dist;
	out->point = vec3_add(sphere->position,
			vec3_scale(out->normal, sphere->collider.radius));
	return (1);
}

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

/* Runs the right generator for (a, b), flipping the normal when the arguments
 * had to be swapped so it always points from a toward b. */
static int	dispatch_pair(const RigidBody *a, const RigidBody *b, Contact *out)
{
	ShapeType	ta;
	ShapeType	tb;

	ta = a->collider.type;
	tb = b->collider.type;
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
	int	i;

	while (w->contactCount + count > w->contactCapacity)
	{
		int		newCap;
		Contact	*grown;

		newCap = 128;
		if (w->contactCapacity > 0)
			newCap = w->contactCapacity * 2;
		grown = realloc(w->contacts, (size_t)newCap * sizeof(Contact));
		if (!grown)
			return (0);
		w->contacts = grown;
		w->contactCapacity = newCap;
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
	int		i;
	int		found;

	w->contactCount = 0;
	i = 0;
	while (i < w->pairCount)
	{
		found = dispatch_pair(&w->bodies[w->pairs[i].a],
				&w->bodies[w->pairs[i].b], buf);
		if (found > 0)
			push_contacts(w, &w->pairs[i], buf, found);
		i++;
	}
}

/*
 * The single contact query: 1 if the two bodies are touching/overlapping,
 * 0 otherwise. It only inspects each body's collider, so it works for any
 * pair you pass.
 */
int	bodies_in_contact(const RigidBody *a, const RigidBody *b)
{
	Contact	buf[MAX_CONTACTS_PER_PAIR];

	return (dispatch_pair(a, b, buf) > 0);
}
