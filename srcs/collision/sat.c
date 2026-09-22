#include "newton.h"

/*
 * Separating axis test for two oriented boxes over the 15 candidate axes (3 faces
 * of each box, 9 edge cross products) [F15]. One axis where the shadows do not
 * overlap proves there is no contact; otherwise the axis of least overlap is the
 * contact normal. A face of A wins unless another axis is clearly better, which
 * keeps resting manifolds from flickering between candidates.
*/

static Vec3	toward_b(Vec3 n, Vec3 d)
{
	if (vec3_dot(d, n) < 0.0f)
		return (vec3_neg(n));
	return (n);
}

static float	overlap_on(const Obb *a, const Obb *b, Vec3 d, Vec3 n)
{
	return (obb_radius(a, n) + obb_radius(b, n) - fabsf(vec3_dot(d, n)));   /* [F15] */
}

static int	clearly_better(float pen, float best)
{
	return (pen < SAT_EDGE_BIAS_REL * best - SAT_EDGE_BIAS_ABS);
}

/* Least overlap among the face axes of 'owner'; 0 when one of them separates the boxes. */
static int	best_face(const Obb *a, const Obb *b, const Obb *owner, Vec3 d, float *pen, int *idx)
{
	float	p;
	int		i;

	*pen = INFINITY;
	*idx = 0;
	i = 0;
	while (i < 3)
	{
		p = overlap_on(a, b, d, owner->ax[i]);
		if (p < 0.0f)
			return (0);
		if (p < *pen)
		{
			*pen = p;
			*idx = i;
		}
		i++;
	}
	return (1);
}

static int	face_axes(const Obb *a, const Obb *b, Vec3 d, SatResult *res)
{
	float	pen_b;
	int		idx_b;

	if (!best_face(a, b, a, d, &res->pen, &res->faceIdx) || !best_face(a, b, b, d, &pen_b, &idx_b))
		return (0);
	res->n = toward_b(a->ax[res->faceIdx], d);
	res->faceOwner = 0;
	if (clearly_better(pen_b, res->pen))
	{
		res->pen = pen_b;
		res->faceIdx = idx_b;
		res->n = toward_b(b->ax[idx_b], d);
		res->faceOwner = 1;
	}
	return (1);
}

/* The 9 edge-edge axes; parallel edges give no axis and are skipped. */
static int	edge_axes(const Obb *a, const Obb *b, Vec3 d, SatResult *res)
{
	Vec3	n;
	float	len2;
	float	pen;
	int		k;

	k = 0;
	while (k < 9)
	{
		n = vec3_cross(a->ax[k / 3], b->ax[k % 3]);
		len2 = vec3_length_sq(n);
		if (len2 > 0.000001f)
		{
			n = vec3_scale(n, 1.0f / sqrtf(len2));
			pen = overlap_on(a, b, d, n);
			if (pen < 0.0f)
				return (0);
			if (clearly_better(pen, res->pen))
			{
				res->pen = pen;
				res->n = toward_b(n, d);
				res->faceOwner = 2;
				res->edgeA = k / 3;
				res->edgeB = k % 3;
			}
		}
		k++;
	}
	return (1);
}

int	sat_boxes(const Obb *a, const Obb *b, SatResult *res)
{
	Vec3	d = vec3_sub(b->c, a->c);

	return (face_axes(a, b, d, res) && edge_axes(a, b, d, res));
}
