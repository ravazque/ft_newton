#include "newton.h"

// A box against a plane (its corners under the square) and against a box (separating axes, then clipping or edges).

int	contact_box_plane(const RigidBody *box_body, const RigidBody *plane, Contact *out)
{
	Obb		box = obb_from_body(box_body);
	Contact	candidates[8];
	float	distance;
	int		count;
	int		i;

	count = 0;
	i = 0;
	while (i < 8)
	{
		candidates[count].point = obb_corner(&box, i);
		distance = collider_plane_distance(&plane->collider, candidates[count].point);   // [F13] per corner
		if (distance < 0.0f && collider_plane_covers(&plane->collider, candidates[count].point))
		{
			candidates[count].normal = vec3_neg(plane->collider.normal);
			candidates[count].penetration = -distance;
			count++;
		}
		i++;
	}
	return (manifold_reduce(candidates, count, plane->collider.normal, out));
}

// The incident face clipped by the four side planes of the reference face; what stays below it is the contact patch.
static int	face_manifold(const Obb *reference, const Obb *incident, const SatResult *sat, Contact *out)
{
	Vec3	reference_normal = sat->normal;
	Vec3	polygon[12];
	Contact	candidates[12];
	float	center_along;
	float	depth;
	int		count;
	int		kept;
	int		i;

	if (sat->faceOwner != 0)
		reference_normal = vec3_neg(sat->normal);
	obb_incident_face(incident, reference_normal, polygon);
	count = 4;
	i = 0;
	while (i < 3 && count > 0)
	{
		center_along = vec3_dot(reference->axis[i], reference->center);
		if (i != sat->faceIndex)
			count = manifold_clip(polygon, count, reference->axis[i], center_along + reference->half[i]);   // [F16]
		if (i != sat->faceIndex && count > 0)
			count = manifold_clip(polygon, count, vec3_neg(reference->axis[i]), -(center_along - reference->half[i]));
		i++;
	}
	kept = 0;
	i = 0;
	while (i < count)
	{
		depth = vec3_dot(reference_normal, polygon[i]) - (vec3_dot(reference_normal, reference->center) + reference->half[sat->faceIndex]);
		if (depth < 0.0f)
		{
			candidates[kept].normal = sat->normal;
			candidates[kept].point = polygon[i];
			candidates[kept].penetration = -depth;
			kept++;
		}
		i++;
	}
	return (manifold_reduce(candidates, kept, reference_normal, out));
}

static int	edge_contact(const Obb *box_a, const Obb *box_b, const SatResult *sat, Contact *out)
{
	Vec3	edge_a[2];
	Vec3	edge_b[2];
	Vec3	closest_a;
	Vec3	closest_b;

	obb_support_edge(box_a, sat->edgeA, sat->normal, &edge_a[0], &edge_a[1]);
	obb_support_edge(box_b, sat->edgeB, vec3_neg(sat->normal), &edge_b[0], &edge_b[1]);
	closest_on_segments(edge_a[0], edge_a[1], edge_b[0], edge_b[1], &closest_a, &closest_b);
	out->normal = sat->normal;
	out->point = vec3_scale(vec3_add(closest_a, closest_b), 0.5f);
	out->penetration = sat->penetration;
	return (1);
}

int	contact_box_box(const RigidBody *box_a, const RigidBody *box_b, Contact *out)
{
	Obb			first = obb_from_body(box_a);
	Obb			second = obb_from_body(box_b);
	SatResult	sat;

	if (!sat_boxes(&first, &second, &sat))
		return (0);
	if (sat.faceOwner == 2)
		return (edge_contact(&first, &second, &sat, out));
	if (sat.faceOwner == 0)
		return (face_manifold(&first, &second, &sat, out));
	return (face_manifold(&second, &first, &sat, out));
}
