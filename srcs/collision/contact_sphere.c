#include "newton.h"

/* Contacts of a sphere against a sphere, a plane and a box: one point each, normal from the
 * sphere toward the other body [F12]-[F14]. */

int	contact_sphere_sphere(const RigidBody *a, const RigidBody *b, Contact *out)
{
	Vec3	d = vec3_sub(b->position, a->position);   /* [F12] */
	float	sum_r = a->collider.radius + b->collider.radius;
	float	dist = vec3_length(d);

	if (dist > sum_r)
		return (0);
	out->normal = vec3(0.0f, 1.0f, 0.0f);
	if (dist > 0.0001f)
		out->normal = vec3_scale(d, 1.0f / dist);
	out->penetration = sum_r - dist;
	out->point = vec3_add(a->position, vec3_scale(out->normal, a->collider.radius - out->penetration * 0.5f));
	return (1);
}

/* Touching when the center is closer than the radius to the surface, above the ground square. */
int	contact_sphere_plane(const RigidBody *sphere, const RigidBody *plane, Contact *out)
{
	float	dist = collider_plane_distance(&plane->collider, sphere->position);   /* [F13] */

	if (dist > sphere->collider.radius || !collider_plane_covers(&plane->collider, sphere->position))
		return (0);
	out->normal = vec3_neg(plane->collider.normal);
	out->penetration = sphere->collider.radius - dist;
	out->point = vec3_add(sphere->position, vec3_scale(out->normal, sphere->collider.radius));
	return (1);
}

/* Center inside the box: push out through the nearest face. */
static int	sphere_inside_box(const RigidBody *sphere, const Obb *o, const float *t, Contact *out)
{
	Vec3	dir;
	int		k;
	int		i;

	k = 0;
	i = 1;
	while (i < 3)
	{
		if (o->h[i] - fabsf(t[i]) < o->h[k] - fabsf(t[k]))
			k = i;
		i++;
	}
	dir = o->ax[k];
	if (t[k] < 0.0f)
		dir = vec3_neg(dir);
	out->normal = vec3_neg(dir);
	out->point = sphere->position;
	out->penetration = o->h[k] - fabsf(t[k]) + sphere->collider.radius;
	return (1);
}

/* Closest point on the box, clamped in the box's own frame: exact for faces, edges and corners alike. */
int	contact_sphere_box(const RigidBody *sphere, const RigidBody *box, Contact *out)
{
	Obb		o = obb_from_body(box);
	Vec3	d = vec3_sub(sphere->position, o.c);
	Vec3	q = o.c;
	float	t[3];
	int		inside;
	int		i;

	inside = 1;
	i = 0;
	while (i < 3)
	{
		t[i] = vec3_dot(d, o.ax[i]);
		if (fabsf(t[i]) > o.h[i])
			inside = 0;
		t[i] = clampf(t[i], -o.h[i], o.h[i]);                   /* [F14] */
		q = vec3_add(q, vec3_scale(o.ax[i], t[i]));
		i++;
	}
	if (inside)
		return (sphere_inside_box(sphere, &o, t, out));
	d = vec3_sub(sphere->position, q);
	if (vec3_length_sq(d) > sphere->collider.radius * sphere->collider.radius)
		return (0);
	out->penetration = sphere->collider.radius - vec3_length(d);
	out->normal = vec3_neg(vec3_normalized(d));
	out->point = q;
	return (1);
}
