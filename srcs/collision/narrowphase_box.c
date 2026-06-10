
#include "newton.h"

#include <float.h>

/*
 * Box narrow-phase tests. A box body is treated as an OBB: center + 3 world
 * axes (the orientation quaternion applied to X/Y/Z) + half extents.
 * box-box runs the Separating Axis Theorem over the 15 candidate axes; when
 * the boxes overlap, the manifold is built by clipping the incident face
 * against the reference face (or a single point for edge-edge cases).
 */

typedef struct Obb
{
	Vec3	c;
	Vec3	ax[3];
	float	h[3];
}	Obb;

typedef struct SatResult
{
	float	pen;
	Vec3	n;			/* unit, points from A toward B          */
	int		faceOwner;	/* 0 = face of A, 1 = face of B, 2 = edge */
	int		faceIdx;
	int		edgeA;
	int		edgeB;
}	SatResult;

static Obb	obb_from_body(const RigidBody *b)
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

/* Projection radius of the box onto a unit axis. */
static float	obb_radius(const Obb *o, Vec3 n)
{
	return (o->h[0] * fabsf(vec3_dot(o->ax[0], n))
		+ o->h[1] * fabsf(vec3_dot(o->ax[1], n))
		+ o->h[2] * fabsf(vec3_dot(o->ax[2], n)));
}

/* Keeps the manifold capped at MAX_CONTACTS_PER_PAIR, dropping the
 * shallowest contact when full. */
static void	manifold_push(Contact *out, int *count, Contact c)
{
	int	i;
	int	shallowest;

	if (*count < MAX_CONTACTS_PER_PAIR)
	{
		out[*count] = c;
		*count += 1;
		return ;
	}
	shallowest = 0;
	i = 1;
	while (i < MAX_CONTACTS_PER_PAIR)
	{
		if (out[i].penetration < out[shallowest].penetration)
			shallowest = i;
		i++;
	}
	if (c.penetration > out[shallowest].penetration)
		out[shallowest] = c;
}

int	contact_box_plane(const RigidBody *box, const RigidBody *plane, Contact *out)
{
	Obb		o;
	Vec3	n;
	int		count;
	int		i;

	o = obb_from_body(box);
	n = plane->collider.normal;
	count = 0;
	i = 0;
	while (i < 8)
	{
		Vec3	v;
		float	dist;
		Contact	c;

		v = vec3_add(o.c, vec3_scale(o.ax[0], (i & 1) ? o.h[0] : -o.h[0]));
		v = vec3_add(v, vec3_scale(o.ax[1], (i & 2) ? o.h[1] : -o.h[1]));
		v = vec3_add(v, vec3_scale(o.ax[2], (i & 4) ? o.h[2] : -o.h[2]));
		dist = vec3_dot(n, v) - plane->collider.offset;
		if (dist < 0.0f)
		{
			c.normal = vec3_neg(n);
			c.point = v;
			c.penetration = -dist;
			manifold_push(out, &count, c);
		}
		i++;
	}
	return (count);
}

int	contact_sphere_box(const RigidBody *sphere, const RigidBody *box, Contact *out)
{
	Obb		o;
	Vec3	d;
	Vec3	q;
	float	t[3];
	int		i;
	int		inside;

	o = obb_from_body(box);
	d = vec3_sub(sphere->position, o.c);
	q = o.c;
	inside = 1;
	i = 0;
	while (i < 3)
	{
		t[i] = vec3_dot(d, o.ax[i]);
		if (t[i] > o.h[i])
		{
			t[i] = o.h[i];
			inside = 0;
		}
		if (t[i] < -o.h[i])
		{
			t[i] = -o.h[i];
			inside = 0;
		}
		q = vec3_add(q, vec3_scale(o.ax[i], t[i]));
		i++;
	}
	if (inside)
	{
		int		k;
		float	depth;
		Vec3	dir;

		k = 0;
		i = 1;
		while (i < 3)
		{
			if (o.h[i] - fabsf(t[i]) < o.h[k] - fabsf(t[k]))
				k = i;
			i++;
		}
		depth = o.h[k] - fabsf(t[k]);
		dir = o.ax[k];
		if (t[k] < 0.0f)
			dir = vec3_neg(dir);
		out->normal = vec3_neg(dir);
		out->point = sphere->position;
		out->penetration = depth + sphere->collider.radius;
		return (1);
	}
	d = vec3_sub(sphere->position, q);
	if (vec3_length_sq(d) > sphere->collider.radius * sphere->collider.radius)
		return (0);
	out->penetration = sphere->collider.radius - vec3_length(d);
	out->normal = vec3_neg(vec3_normalized(d));
	out->point = q;
	return (1);
}

/* SAT over the 15 axes. Returns 0 when a separating axis exists. Face axes
 * are preferred over edge axes (bias) for manifold stability. */
static int	sat_boxes(const Obb *a, const Obb *b, SatResult *res)
{
	Vec3	d;
	float	facePen[2];
	Vec3	faceN[2];
	int		faceIdx[2];
	int		i;
	int		j;

	d = vec3_sub(b->c, a->c);
	facePen[0] = FLT_MAX;
	facePen[1] = FLT_MAX;
	faceIdx[0] = 0;
	faceIdx[1] = 0;
	faceN[0] = vec3(1.0f, 0.0f, 0.0f);
	faceN[1] = vec3(1.0f, 0.0f, 0.0f);
	i = 0;
	while (i < 6)
	{
		const Obb	*owner = (i < 3) ? a : b;
		Vec3		n = owner->ax[i % 3];
		float		pen;

		pen = obb_radius(a, n) + obb_radius(b, n) - fabsf(vec3_dot(d, n));
		if (pen < 0.0f)
			return (0);
		if (pen < facePen[i / 3])
		{
			facePen[i / 3] = pen;
			faceIdx[i / 3] = i % 3;
			faceN[i / 3] = (vec3_dot(d, n) < 0.0f) ? vec3_neg(n) : n;
		}
		i++;
	}
	res->pen = facePen[0];
	res->n = faceN[0];
	res->faceOwner = 0;
	res->faceIdx = faceIdx[0];
	if (facePen[1] < 0.95f * res->pen - 0.005f)
	{
		res->pen = facePen[1];
		res->n = faceN[1];
		res->faceOwner = 1;
		res->faceIdx = faceIdx[1];
	}
	i = 0;
	while (i < 3)
	{
		j = 0;
		while (j < 3)
		{
			Vec3	cr = vec3_cross(a->ax[i], b->ax[j]);
			float	len2 = vec3_length_sq(cr);
			Vec3	n;
			float	pen;

			if (len2 > 0.000001f)
			{
				n = vec3_scale(cr, 1.0f / sqrtf(len2));
				pen = obb_radius(a, n) + obb_radius(b, n)
					- fabsf(vec3_dot(d, n));
				if (pen < 0.0f)
					return (0);
				if (pen < 0.95f * res->pen - 0.005f)
				{
					res->pen = pen;
					res->n = (vec3_dot(d, n) < 0.0f) ? vec3_neg(n) : n;
					res->faceOwner = 2;
					res->edgeA = i;
					res->edgeB = j;
				}
			}
			j++;
		}
		i++;
	}
	return (1);
}

/* Sutherland-Hodgman: clips 'poly' in place against dot(n, x) <= off. */
static int	clip_plane(Vec3 *poly, int count, Vec3 n, float off)
{
	Vec3	outp[12];
	int		outc;
	int		i;

	outc = 0;
	i = 0;
	while (i < count)
	{
		Vec3	cur = poly[i];
		Vec3	nxt = poly[(i + 1) % count];
		float	dc = vec3_dot(n, cur) - off;
		float	dn = vec3_dot(n, nxt) - off;

		if (dc <= 0.0f)
			outp[outc++] = cur;
		if ((dc < 0.0f && dn > 0.0f) || (dc > 0.0f && dn < 0.0f))
			outp[outc++] = vec3_add(cur,
					vec3_scale(vec3_sub(nxt, cur), dc / (dc - dn)));
		i++;
	}
	memcpy(poly, outp, (size_t)outc * sizeof(Vec3));
	return (outc);
}

/* Builds the face manifold: takes the incident face of 'inc' (the one most
 * opposed to refN), clips it against the 4 side planes of the reference face
 * of 'ref', and keeps every clipped point below the reference face. */
static int	face_manifold(const Obb *ref, const Obb *inc, SatResult *sat,
				Contact *out)
{
	Vec3	refN;
	Vec3	poly[12];
	int		count;
	int		m;
	int		i;

	refN = (sat->faceOwner == 0) ? sat->n : vec3_neg(sat->n);
	m = 0;
	i = 1;
	while (i < 3)
	{
		if (fabsf(vec3_dot(inc->ax[i], refN))
			> fabsf(vec3_dot(inc->ax[m], refN)))
			m = i;
		i++;
	}
	{
		Vec3	f = inc->ax[m];
		int		p = (m + 1) % 3;
		int		q = (m + 2) % 3;
		Vec3	fc;

		if (vec3_dot(f, refN) > 0.0f)
			f = vec3_neg(f);
		fc = vec3_add(inc->c, vec3_scale(f, inc->h[m]));
		poly[0] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], inc->h[p]),
					vec3_scale(inc->ax[q], inc->h[q])));
		poly[1] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], -inc->h[p]),
					vec3_scale(inc->ax[q], inc->h[q])));
		poly[2] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], -inc->h[p]),
					vec3_scale(inc->ax[q], -inc->h[q])));
		poly[3] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], inc->h[p]),
					vec3_scale(inc->ax[q], -inc->h[q])));
	}
	count = 4;
	i = 0;
	while (i < 3 && count > 0)
	{
		if (i != sat->faceIdx)
		{
			Vec3	u = ref->ax[i];
			float	uc = vec3_dot(u, ref->c);

			count = clip_plane(poly, count, u, uc + ref->h[i]);
			if (count > 0)
				count = clip_plane(poly, count, vec3_neg(u),
						-(uc - ref->h[i]));
		}
		i++;
	}
	{
		float	faceOff = vec3_dot(refN, ref->c) + ref->h[sat->faceIdx];
		int		kept = 0;

		i = 0;
		while (i < count)
		{
			float	depth = vec3_dot(refN, poly[i]) - faceOff;

			if (depth < 0.0f)
			{
				Contact	c;

				c.normal = sat->n;
				c.point = poly[i];
				c.penetration = -depth;
				manifold_push(out, &kept, c);
			}
			i++;
		}
		return (kept);
	}
}

/* Supporting edge of the box along 'n' with direction ax[dirIdx]. */
static void	support_edge(const Obb *o, int dirIdx, Vec3 n, Vec3 *p0, Vec3 *p1)
{
	Vec3	p;
	int		i;

	p = o->c;
	i = 0;
	while (i < 3)
	{
		if (i != dirIdx)
		{
			float	s = (vec3_dot(o->ax[i], n) >= 0.0f) ? 1.0f : -1.0f;

			p = vec3_add(p, vec3_scale(o->ax[i], s * o->h[i]));
		}
		i++;
	}
	*p0 = vec3_sub(p, vec3_scale(o->ax[dirIdx], o->h[dirIdx]));
	*p1 = vec3_add(p, vec3_scale(o->ax[dirIdx], o->h[dirIdx]));
}

static float	clampf(float v, float lo, float hi)
{
	if (v < lo)
		return (lo);
	if (v > hi)
		return (hi);
	return (v);
}

/* Closest points between two segments (Ericson, Real-Time Collision
 * Detection 5.1.9, simplified: box edges never degenerate to points). */
static void	closest_on_edges(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2,
				Vec3 *c1, Vec3 *c2)
{
	Vec3	d1 = vec3_sub(q1, p1);
	Vec3	d2 = vec3_sub(q2, p2);
	Vec3	r = vec3_sub(p1, p2);
	float	a = vec3_dot(d1, d1);
	float	e = vec3_dot(d2, d2);
	float	f = vec3_dot(d2, r);
	float	c = vec3_dot(d1, r);
	float	b = vec3_dot(d1, d2);
	float	den = a * e - b * b;
	float	s;
	float	t;

	s = 0.0f;
	if (den > 0.000001f)
		s = clampf((b * f - c * e) / den, 0.0f, 1.0f);
	t = (b * s + f) / e;
	if (t < 0.0f || t > 1.0f)
	{
		t = clampf(t, 0.0f, 1.0f);
		s = clampf((b * t - c) / a, 0.0f, 1.0f);
	}
	*c1 = vec3_add(p1, vec3_scale(d1, s));
	*c2 = vec3_add(p2, vec3_scale(d2, t));
}

int	contact_box_box(const RigidBody *ba, const RigidBody *bb, Contact *out)
{
	Obb			a;
	Obb			b;
	SatResult	sat;

	a = obb_from_body(ba);
	b = obb_from_body(bb);
	if (!sat_boxes(&a, &b, &sat))
		return (0);
	if (sat.faceOwner == 2)
	{
		Vec3	pa0;
		Vec3	pa1;
		Vec3	pb0;
		Vec3	pb1;
		Vec3	ca;
		Vec3	cb;

		support_edge(&a, sat.edgeA, sat.n, &pa0, &pa1);
		support_edge(&b, sat.edgeB, vec3_neg(sat.n), &pb0, &pb1);
		closest_on_edges(pa0, pa1, pb0, pb1, &ca, &cb);
		out->normal = sat.n;
		out->point = vec3_scale(vec3_add(ca, cb), 0.5f);
		out->penetration = sat.pen;
		return (1);
	}
	if (sat.faceOwner == 0)
		return (face_manifold(&a, &b, &sat, out));
	return (face_manifold(&b, &a, &sat, out));
}
