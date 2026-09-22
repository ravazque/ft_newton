#include "newton.h"

/* A box body seen as an oriented box (center, three world axes, half extents) and the
 * geometric queries the box contacts share. */

Obb	obb_from_body(const RigidBody *b)
{
	Obb	o;

	o.c = b->position;
	o.ax[0] = quat_rotate(b->orientation, vec3(1.0f, 0.0f, 0.0f));
	o.ax[1] = quat_rotate(b->orientation, vec3(0.0f, 1.0f, 0.0f));
	o.ax[2] = quat_rotate(b->orientation, vec3(0.0f, 0.0f, 1.0f));
	o.h[0] = b->collider.halfExtents.x;
	o.h[1] = b->collider.halfExtents.y;
	o.h[2] = b->collider.halfExtents.z;
	return (o);
}

/* Half length of the box's shadow on a unit axis. */
float	obb_radius(const Obb *o, Vec3 n)
{
	return (o->h[0] * fabsf(vec3_dot(o->ax[0], n)) + o->h[1] * fabsf(vec3_dot(o->ax[1], n)) + o->h[2] * fabsf(vec3_dot(o->ax[2], n)));   /* [F15] */
}

/* Corner i: bit k of i picks the + or - side along axis k. */
Vec3	obb_corner(const Obb *o, int i)
{
	Vec3	v = o->c;
	float	sign;
	int		axis;

	axis = 0;
	while (axis < 3)
	{
		sign = -1.0f;
		if (i & (1 << axis))
			sign = 1.0f;
		v = vec3_add(v, vec3_scale(o->ax[axis], sign * o->h[axis]));
		axis++;
	}
	return (v);
}

/* The edge along ax[dir_idx] that reaches farthest along n. */
void	obb_support_edge(const Obb *o, int dir_idx, Vec3 n, Vec3 *p0, Vec3 *p1)
{
	Vec3	p = o->c;
	float	sign;
	int		i;

	i = 0;
	while (i < 3)
	{
		sign = -1.0f;
		if (vec3_dot(o->ax[i], n) >= 0.0f)
			sign = 1.0f;
		if (i != dir_idx)
			p = vec3_add(p, vec3_scale(o->ax[i], sign * o->h[i]));
		i++;
	}
	*p0 = vec3_sub(p, vec3_scale(o->ax[dir_idx], o->h[dir_idx]));
	*p1 = vec3_add(p, vec3_scale(o->ax[dir_idx], o->h[dir_idx]));
}
