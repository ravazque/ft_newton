#include "newton.h"

/*
 * Box contacts. Against a plane: every corner below the ground square. Against
 * another box: the separating axis test (sat.c) gives the normal; a face normal
 * clips the incident face of the other box against the reference face
 * (manifold.c) for up to four points, an edge normal gives the single closest
 * point between the two edges.
*/

int	contact_box_plane(const RigidBody *box, const RigidBody *plane, Contact *out)
{
	Obb		o = obb_from_body(box);
	Contact	cand[8];
	float	dist;
	int		count;
	int		i;

	count = 0;
	i = 0;
	while (i < 8)
	{
		cand[count].point = obb_corner(&o, i);
		dist = collider_plane_distance(&plane->collider, cand[count].point);   /* [F13] per corner */
		if (dist < 0.0f && collider_plane_covers(&plane->collider, cand[count].point))
		{
			cand[count].normal = vec3_neg(plane->collider.normal);
			cand[count].penetration = -dist;
			count++;
		}
		i++;
	}
	return (manifold_reduce(cand, count, plane->collider.normal, out));
}

/* The face of 'inc' most opposed to the reference normal, as a 4-point polygon. */
static void	incident_face(const Obb *inc, Vec3 ref_n, Vec3 *poly)
{
	Vec3	f;
	Vec3	fc;
	int		m;
	int		p;
	int		q;

	m = 0;
	if (fabsf(vec3_dot(inc->ax[1], ref_n)) > fabsf(vec3_dot(inc->ax[m], ref_n)))
		m = 1;
	if (fabsf(vec3_dot(inc->ax[2], ref_n)) > fabsf(vec3_dot(inc->ax[m], ref_n)))
		m = 2;
	p = (m + 1) % 3;
	q = (m + 2) % 3;
	f = inc->ax[m];
	if (vec3_dot(f, ref_n) > 0.0f)
		f = vec3_neg(f);
	fc = vec3_add(inc->c, vec3_scale(f, inc->h[m]));
	poly[0] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], inc->h[p]), vec3_scale(inc->ax[q], inc->h[q])));
	poly[1] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], -inc->h[p]), vec3_scale(inc->ax[q], inc->h[q])));
	poly[2] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], -inc->h[p]), vec3_scale(inc->ax[q], -inc->h[q])));
	poly[3] = vec3_add(fc, vec3_add(vec3_scale(inc->ax[p], inc->h[p]), vec3_scale(inc->ax[q], -inc->h[q])));
}

/* Clips the incident face against the four side planes of the reference face, keeps what lies
 * below it [F16]. */
static int	face_manifold(const Obb *ref, const Obb *inc, const SatResult *sat, Contact *out)
{
	Vec3	ref_n = sat->n;
	Vec3	poly[12];
	Contact	cand[12];
	float	uc;
	float	depth;
	int		count;
	int		kept;
	int		i;

	if (sat->faceOwner != 0)
		ref_n = vec3_neg(sat->n);
	incident_face(inc, ref_n, poly);
	count = 4;
	i = 0;
	while (i < 3 && count > 0)
	{
		uc = vec3_dot(ref->ax[i], ref->c);
		if (i != sat->faceIdx)
			count = manifold_clip(poly, count, ref->ax[i], uc + ref->h[i]);
		if (i != sat->faceIdx && count > 0)
			count = manifold_clip(poly, count, vec3_neg(ref->ax[i]), -(uc - ref->h[i]));
		i++;
	}
	kept = 0;
	i = 0;
	while (i < count)
	{
		depth = vec3_dot(ref_n, poly[i]) - (vec3_dot(ref_n, ref->c) + ref->h[sat->faceIdx]);
		if (depth < 0.0f)
		{
			cand[kept].normal = sat->n;
			cand[kept].point = poly[i];
			cand[kept].penetration = -depth;
			kept++;
		}
		i++;
	}
	return (manifold_reduce(cand, kept, ref_n, out));
}

/* Closest points between two segments (Ericson, Real-Time Collision Detection 5.1.9); box
 * edges never degenerate. */
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

static int	edge_contact(const Obb *a, const Obb *b, const SatResult *sat, Contact *out)
{
	Vec3	pa0;
	Vec3	pa1;
	Vec3	pb0;
	Vec3	pb1;
	Vec3	ca;
	Vec3	cb;

	obb_support_edge(a, sat->edgeA, sat->n, &pa0, &pa1);
	obb_support_edge(b, sat->edgeB, vec3_neg(sat->n), &pb0, &pb1);
	closest_on_edges(pa0, pa1, pb0, pb1, &ca, &cb);
	out->normal = sat->n;
	out->point = vec3_scale(vec3_add(ca, cb), 0.5f);
	out->penetration = sat->pen;
	return (1);
}

int	contact_box_box(const RigidBody *ba, const RigidBody *bb, Contact *out)
{
	Obb			a = obb_from_body(ba);
	Obb			b = obb_from_body(bb);
	SatResult	sat;

	if (!sat_boxes(&a, &b, &sat))
		return (0);
	if (sat.faceOwner == 2)
		return (edge_contact(&a, &b, &sat, out));
	if (sat.faceOwner == 0)
		return (face_manifold(&a, &b, &sat, out));
	return (face_manifold(&b, &a, &sat, out));
}
