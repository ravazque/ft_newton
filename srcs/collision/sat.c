#include "newton.h"

// Separating axis test of two boxes on 15 axes [F15]; a face of A wins unless another axis is clearly better.

static Vec3	toward_b(Vec3 axis, Vec3 a_to_b)
{
	if (vec3_dot(a_to_b, axis) < 0.0f)
		return (vec3_neg(axis));
	return (axis);
}

static float	overlap_on(const Obb *box_a, const Obb *box_b, Vec3 a_to_b, Vec3 axis)
{
	return (obb_radius(box_a, axis) + obb_radius(box_b, axis) - fabsf(vec3_dot(a_to_b, axis)));   // [F15]
}

int	sat_clearly_better(float penetration, float best)
{
	return (penetration < SAT_EDGE_BIAS_REL * best - SAT_EDGE_BIAS_ABS);
}

static int	best_face(const Obb *box_a, const Obb *box_b, const Obb *owner, Vec3 a_to_b, float *penetration, int *face)
{
	float	overlap;
	int		i;

	*penetration = INFINITY;
	*face = 0;
	i = 0;
	while (i < 3)
	{
		overlap = overlap_on(box_a, box_b, a_to_b, owner->axis[i]);
		if (overlap < 0.0f)
			return (0);
		if (overlap < *penetration)
		{
			*penetration = overlap;
			*face = i;
		}
		i++;
	}
	return (1);
}

static int	face_axes(const Obb *box_a, const Obb *box_b, Vec3 a_to_b, SatResult *result)
{
	float	penetration_b;
	int		face_b;

	if (!best_face(box_a, box_b, box_a, a_to_b, &result->penetration, &result->faceIndex) || !best_face(box_a, box_b, box_b, a_to_b, &penetration_b, &face_b))
		return (0);
	result->normal = toward_b(box_a->axis[result->faceIndex], a_to_b);
	result->faceOwner = 0;
	if (sat_clearly_better(penetration_b, result->penetration))
	{
		result->penetration = penetration_b;
		result->faceIndex = face_b;
		result->normal = toward_b(box_b->axis[face_b], a_to_b);
		result->faceOwner = 1;
	}
	return (1);
}

// Parallel edges give no axis and are skipped.
static int	edge_axes(const Obb *box_a, const Obb *box_b, Vec3 a_to_b, SatResult *result)
{
	Vec3	axis;
	float	length_sq;
	float	penetration;
	int		k;

	k = 0;
	while (k < 9)
	{
		axis = vec3_cross(box_a->axis[k / 3], box_b->axis[k % 3]);
		length_sq = vec3_length_sq(axis);
		if (length_sq > 0.000001f)
		{
			axis = vec3_scale(axis, 1.0f / sqrtf(length_sq));
			penetration = overlap_on(box_a, box_b, a_to_b, axis);
			if (penetration < 0.0f)
				return (0);
			if (sat_clearly_better(penetration, result->penetration))
			{
				result->penetration = penetration;
				result->normal = toward_b(axis, a_to_b);
				result->faceOwner = 2;
				result->edgeA = k / 3;
				result->edgeB = k % 3;
			}
		}
		k++;
	}
	return (1);
}

int	sat_boxes(const Obb *box_a, const Obb *box_b, SatResult *result)
{
	Vec3	a_to_b = vec3_sub(box_b->center, box_a->center);

	return (face_axes(box_a, box_b, a_to_b, result) && edge_axes(box_a, box_b, a_to_b, result));
}
