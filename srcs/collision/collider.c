
#include "newton.h"

Collider	collider_sphere(float radius)
{
	Collider	c;

	memset(&c, 0, sizeof(c));
	c.type = SHAPE_SPHERE;
	c.radius = radius;
	c.halfExtents = vec3(radius, radius, radius);
	c.normal = vec3(0.0f, 1.0f, 0.0f);
	return (c);
}

Collider	collider_box(Vec3 half_extents)
{
	Collider	c;

	memset(&c, 0, sizeof(c));
	c.type = SHAPE_BOX;
	c.halfExtents = half_extents;
	c.normal = vec3(0.0f, 1.0f, 0.0f);
	return (c);
}

Collider	collider_plane(Vec3 normal, float offset)
{
	Collider	c;

	memset(&c, 0, sizeof(c));
	c.type = SHAPE_PLANE;
	c.normal = vec3_normalized(normal);
	c.offset = offset;
	return (c);
}

/*
 * Local-space INVERSE inertia tensor. Both dynamic shapes have a
 * diagonal tensor in their own frame:
 *   sphere: I = 2/5 * m * r^2 on every axis
 *   box:    I = m/12 * (H^2 + D^2, ...) with full extents = m/3 * (hy^2 + hz^2, ...)
 * A plane never moves, so its inverse tensor is zero (like its inverse mass).
*/
Mat3	collider_compute_inertia(const Collider *c, float mass)
{
	Vec3	h;
	float	i;

	if (mass <= 0.0f || c->type == SHAPE_PLANE)
		return (mat3_zero());
	if (c->type == SHAPE_SPHERE)
	{
		i = 0.4f * mass * c->radius * c->radius;   /* [F5] I = 2/5 m R^2 */
		return (mat3_inverse(mat3_diagonal(vec3(i, i, i))));
	}
	h = c->halfExtents;
	return (mat3_inverse(mat3_diagonal(vec3(mass / 3.0f * (h.y * h.y + h.z * h.z), mass / 3.0f * (h.x * h.x + h.z * h.z), mass / 3.0f * (h.x * h.x + h.y * h.y)))));   /* [F6] */
}

/* A plane is defined by normal . x = offset, independent of its body's
 * position; these two helpers give the renderer a transform that matches. */
Vec3	collider_plane_origin(const Collider *c)
{
	return (vec3_scale(c->normal, c->offset));
}

Quat	collider_plane_rotation(const Collider *c)
{
	return (quat_from_to(vec3(0.0f, 1.0f, 0.0f), c->normal));
}
