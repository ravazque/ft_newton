
#include "newton.h"

/*
 * Collider constructors are provided; collider_compute_inertia is [TODO]
 * (the inertia tensor that the rotational physics depends on).
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

Collider	collider_plane(Vec3 normal, float offset)
{
	Collider	c;

	memset(&c, 0, sizeof(c));
	c.type = SHAPE_PLANE;
	c.normal = vec3_normalized(normal);
	c.offset = offset;
	return (c);
}

Mat3	collider_compute_inertia(const Collider *c, float mass)
{
	/* TODO: local (diagonal) inverse inertia tensor per shape:
	 *   Box:    I = 1/12 * m * (hy^2+hz^2, hx^2+hz^2, hx^2+hy^2) (full extents)
	 *   Sphere: I = 2/5 * m * r^2 on each axis
	 *   Plane:  zero (static, infinite mass)
	 * Return the INVERSE tensor (mat3_inverse of the diagonal), or zero for
	 * static shapes.
	*/
	(void)c;
	(void)mass;
	return (mat3_identity());
}
