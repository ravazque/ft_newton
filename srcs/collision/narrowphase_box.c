
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
	return (o->h[0] * fabsf(vec3_dot(o->ax[0], n)) + o->h[1] * fabsf(vec3_dot(o->ax[1], n)) + o->h[2] * fabsf(vec3_dot(o->ax[2], n)));
}

/* Reduces a set of coplanar candidate points to at most MAX_CONTACTS_PER_PAIR
 * that stay well spread: the deepest point, the one farthest from it, the one
 * making the largest triangle with them, and the one on the other side of
 * the first two making the largest quad. Keeping simply the deepest points
 * would cluster the manifold on one side and make a resting box rock, since
 * the kept subset (and its torque) would change from step to step. */
static int	pick_point(const Contact *cand, int n, const int *taken, Vec3 axis, Vec3 p0, Vec3 p1, int mode, int side)
{
	float	best_score = -1.0f;
	float	score;
	int		best = -1;
	int		i;

	i = 0;
	while (i < n)
	{
		if (!taken[i])
		{
			if (mode == 0)
				score = cand[i].penetration;
			else if (mode == 1)
				score = vec3_length_sq(vec3_sub(cand[i].point, p0));
			else
				score = (float)side * vec3_dot(axis, vec3_cross(vec3_sub(p1, p0), vec3_sub(cand[i].point, p0)));
			if (score > best_score)
			{
				best_score = score;
				best = i;
			}
		}
		i++;
	}
	if (mode == 2 && best_score <= 0.000001f)
		return (-1);
	return (best);
}

static int	reduce_manifold(const Contact *cand, int n, Vec3 axis, Contact *out)
{
	int		taken[12] = {0};
	int		chosen[4];
	int		count;
	int		side;
	int		k;

	if (n <= MAX_CONTACTS_PER_PAIR)
	{
		memcpy(out, cand, (size_t)n * sizeof(Contact));
		return (n);
	}
	chosen[0] = pick_point(cand, n, taken, axis, cand[0].point, cand[0].point, 0, 1);
	taken[chosen[0]] = 1;
	chosen[1] = pick_point(cand, n, taken, axis, cand[chosen[0]].point, cand[chosen[0]].point, 1, 1);
	taken[chosen[1]] = 1;
	count = 2;
	side = 1;
	while (count < 4 && side >= -1)
	{
		k = pick_point(cand, n, taken, axis, cand[chosen[0]].point, cand[chosen[1]].point, 2, side);
		if (k >= 0)
		{
			taken[k] = 1;
			chosen[count++] = k;
		}
		side -= 2;
	}
	k = 0;
	while (k < count)
	{
		out[k] = cand[chosen[k]];
		k++;
	}
	return (count);
}

/* Corner i of the box: each bit of i picks the +/- side on one axis. */
static Vec3	obb_corner(const Obb *o, int i)
{
	Vec3	v = o->c;
	int		axis;
	float	sign;

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

int	contact_box_plane(const RigidBody *box, const RigidBody *plane, Contact *out)
{
	Obb		o;
	Vec3	n;
	Contact	cand[8];
	float	dist;
	int		count;
	int		i;

	o = obb_from_body(box);
	n = plane->collider.normal;
	count = 0;
	i = 0;
	while (i < 8)
	{
		cand[count].point = obb_corner(&o, i);
		dist = vec3_dot(n, cand[count].point) - plane->collider.offset;
		if (dist < 0.0f)
		{
			cand[count].normal = vec3_neg(n);
			cand[count].penetration = -dist;
			count++;
		}
		i++;
	}
	return (reduce_manifold(cand, count, n, out));
}

int	contact_sphere_box(const RigidBody *sphere, const RigidBody *box, Contact *out)
{
	Obb		o;
	Vec3	d;
	Vec3	q;
	float	t[3];
	int		i;
	int		inside;

	o = obb_from_body(box);          /* [F14] closest point on the box, clamped in its own frame */
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

/* The candidate axis oriented from A toward B. */
static Vec3	axis_toward_b(Vec3 n, Vec3 d)
{
	if (vec3_dot(d, n) < 0.0f)
		return (vec3_neg(n));
	return (n);
}

/* SAT over the 15 axes. Returns 0 when a separating axis exists. Face axes
 * are preferred over edge axes (bias) for manifold stability. */
static int	sat_boxes(const Obb *a, const Obb *b, SatResult *res)
{
	const Obb	*owner;
	Vec3		d;
	Vec3		n;
	float		pen;
	float		facePen[2];
	Vec3		faceN[2];
	int			faceIdx[2];
	int			i;
	int			j;

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
		owner = b;
		if (i < 3)
			owner = a;
		n = owner->ax[i % 3];
		pen = obb_radius(a, n) + obb_radius(b, n) - fabsf(vec3_dot(d, n));   /* [F15] -separation on a face axis */
		if (pen < 0.0f)
			return (0);
		if (pen < facePen[i / 3])
		{
			facePen[i / 3] = pen;
			faceIdx[i / 3] = i % 3;
			faceN[i / 3] = axis_toward_b(n, d);
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

			if (len2 > 0.000001f)
			{
				n = vec3_scale(cr, 1.0f / sqrtf(len2));
				pen = obb_radius(a, n) + obb_radius(b, n) - fabsf(vec3_dot(d, n));   /* [F15] edge-edge axis */
				if (pen < 0.0f)
					return (0);
				if (pen < 0.95f * res->pen - 0.005f)
				{
					res->pen = pen;
					res->n = axis_toward_b(n, d);
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
/* [F16] Sutherland-Hodgman: keeps the part of the polygon behind a plane. */
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
			outp[outc++] = vec3_add(cur, vec3_scale(vec3_sub(nxt, cur), dc / (dc - dn)));
		i++;
	}
	memcpy(poly, outp, (size_t)outc * sizeof(Vec3));
	return (outc);
}

/* Builds the face manifold: takes the incident face of 'inc' (the one most
 * opposed to refN), clips it against the 4 side planes of the reference face
 * of 'ref', and keeps every clipped point below the reference face. */
static int	face_manifold(const Obb *ref, const Obb *inc, SatResult *sat, Contact *out)
{
	Vec3	refN;
	Vec3	poly[12];
	int		count;
	int		m;
	int		i;

	refN = sat->n;
	if (sat->faceOwner != 0)
		refN = vec3_neg(sat->n);
	m = 0;
	i = 1;
	while (i < 3)
	{
		if (fabsf(vec3_dot(inc->ax[i], refN)) > fabsf(vec3_dot(inc->ax[m], refN)))
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
		poly[0] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], inc->h[p]), vec3_scale(inc->ax[q], inc->h[q])));
		poly[1] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], -inc->h[p]), vec3_scale(inc->ax[q], inc->h[q])));
		poly[2] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], -inc->h[p]), vec3_scale(inc->ax[q], -inc->h[q])));
		poly[3] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], inc->h[p]), vec3_scale(inc->ax[q], -inc->h[q])));
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
				count = clip_plane(poly, count, vec3_neg(u), -(uc - ref->h[i]));
		}
		i++;
	}
	{
		float	faceOff = vec3_dot(refN, ref->c) + ref->h[sat->faceIdx];
		Contact	cand[12];
		int		kept = 0;

		i = 0;
		while (i < count)
		{
			float	depth = vec3_dot(refN, poly[i]) - faceOff;

			if (depth < 0.0f)
			{
				cand[kept].normal = sat->n;
				cand[kept].point = poly[i];
				cand[kept].penetration = -depth;
				kept++;
			}
			i++;
		}
		return (reduce_manifold(cand, kept, refN, out));
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
			float	s = -1.0f;

			if (vec3_dot(o->ax[i], n) >= 0.0f)
				s = 1.0f;
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
static void	closest_on_edges(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2, Vec3 *c1, Vec3 *c2)
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
