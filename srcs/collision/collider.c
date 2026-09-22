#include "newton.h"

/*
 * Collider shapes and their world bounds. A plane is the surface normal . x =
 * offset limited to a square of side 'size' (the ground drawn on screen), solid
 * down to PLANE_THICKNESS under it: past its edge nothing holds a body up [F13].
*/

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

Collider	collider_plane(Vec3 normal, float offset, float size)
{
	Collider	c;

	memset(&c, 0, sizeof(c));
	c.type = SHAPE_PLANE;
	c.normal = vec3_normalized(normal);
	c.offset = offset;
	c.halfSize = size * 0.5f;
	return (c);
}

/* Center of the plane's square; the plane's body position plays no part. */
Vec3	collider_plane_origin(const Collider *c)
{
	return (vec3_scale(c->normal, c->offset));
}

/* Turns local +Y onto the normal: the square spans local X and Z. */
Quat	collider_plane_rotation(const Collider *c)
{
	return (quat_from_to(vec3(0.0f, 1.0f, 0.0f), c->normal));
}

float	collider_plane_distance(const Collider *c, Vec3 point)
{
	return (vec3_dot(c->normal, point) - c->offset);   /* [F13] signed distance */
}

/* Whether the plane's solid slab lies under this point: inside the square and not deeper than
 * its thickness. */
int	collider_plane_covers(const Collider *c, Vec3 point)
{
	Quat	rotation = collider_plane_rotation(c);
	Vec3	d = vec3_sub(point, collider_plane_origin(c));
	float	u = vec3_dot(d, quat_rotate(rotation, vec3(1.0f, 0.0f, 0.0f)));
	float	v = vec3_dot(d, quat_rotate(rotation, vec3(0.0f, 0.0f, 1.0f)));

	return (fabsf(u) <= c->halfSize && fabsf(v) <= c->halfSize && collider_plane_distance(c, point) >= -PLANE_THICKNESS);   /* [F13] */
}

/* AABB of an oriented box: the rotated half extents, |R| . h, axis by axis. */
static void	oriented_bounds(Vec3 center, Quat rotation, Vec3 h, Vec3 *mn, Vec3 *mx)
{
	Vec3	a0 = quat_rotate(rotation, vec3(1.0f, 0.0f, 0.0f));
	Vec3	a1 = quat_rotate(rotation, vec3(0.0f, 1.0f, 0.0f));
	Vec3	a2 = quat_rotate(rotation, vec3(0.0f, 0.0f, 1.0f));
	Vec3	ex;

	ex.x = fabsf(a0.x) * h.x + fabsf(a1.x) * h.y + fabsf(a2.x) * h.z;   /* [F10] */
	ex.y = fabsf(a0.y) * h.x + fabsf(a1.y) * h.y + fabsf(a2.y) * h.z;
	ex.z = fabsf(a0.z) * h.x + fabsf(a1.z) * h.y + fabsf(a2.z) * h.z;
	*mn = vec3_sub(center, ex);
	*mx = vec3_add(center, ex);
}

/* World AABB of a body; a plane's is its square extruded PLANE_THICKNESS down along the normal. */
void	collider_bounds(const RigidBody *b, Vec3 *mn, Vec3 *mx)
{
	const Collider	*c = &b->collider;
	float			half_depth = PLANE_THICKNESS * 0.5f;
	Vec3			r;

	if (c->type == SHAPE_SPHERE)
	{
		r = vec3(c->radius, c->radius, c->radius);
		*mn = vec3_sub(b->position, r);
		*mx = vec3_add(b->position, r);
	}
	else if (c->type == SHAPE_BOX)
		oriented_bounds(b->position, b->orientation, c->halfExtents, mn, mx);
	else
		oriented_bounds(vec3_sub(collider_plane_origin(c), vec3_scale(c->normal, half_depth)), collider_plane_rotation(c), vec3(c->halfSize, half_depth, c->halfSize), mn, mx);
}
