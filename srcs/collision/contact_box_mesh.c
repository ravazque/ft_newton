#include "newton.h"

// A box against each nearby triangle of a mesh on 13 axes [F31]; the one-sided triangle normal wins unless clearly worse.

typedef struct TriangleSat
{
	float	penetration;
	Vec3	normal;
	int		kind;
	int		boxAxis;
	int		triangleEdge;
}	TriangleSat;

typedef struct BoxQuery
{
	const MeshShape		*mesh;
	Obb					box;
	ContactCandidates	set;
}	BoxQuery;

// kind of a TriangleSat: which feature gives the axis.
enum
{
	AXIS_TRIANGLE_FACE,
	AXIS_BOX_FACE,
	AXIS_EDGES
};

static int	overlap_on(const Obb *box, const Vec3 *triangle, Vec3 axis, float *penetration, Vec3 *normal)
{
	float	center = vec3_dot(box->center, axis);
	float	radius = obb_radius(box, axis);
	float	low = fminf(vec3_dot(triangle[0], axis), fminf(vec3_dot(triangle[1], axis), vec3_dot(triangle[2], axis)));
	float	high = fmaxf(vec3_dot(triangle[0], axis), fmaxf(vec3_dot(triangle[1], axis), vec3_dot(triangle[2], axis)));

	if (center + radius < low || high < center - radius)
		return (0);
	*penetration = center + radius - low;   // [F31]
	*normal = axis;
	if (high - (center - radius) < *penetration)
	{
		*penetration = high - (center - radius);
		*normal = vec3_neg(axis);
	}
	return (1);
}

static int	triangle_axis(const Obb *box, const Vec3 *triangle, Vec3 face_normal, TriangleSat *sat)
{
	float	center = vec3_dot(box->center, face_normal);
	float	radius = obb_radius(box, face_normal);
	float	plane = vec3_dot(triangle[0], face_normal);

	if (center - radius > plane || center + radius < plane)
		return (0);
	sat->penetration = plane - (center - radius);   // [F31]
	sat->normal = vec3_neg(face_normal);
	sat->kind = AXIS_TRIANGLE_FACE;
	return (1);
}

static void	keep_axis(TriangleSat *sat, float penetration, Vec3 normal, int kind, int box_axis, int triangle_edge)
{
	if (!sat_clearly_better(penetration, sat->penetration))
		return ;
	sat->penetration = penetration;
	sat->normal = normal;
	sat->kind = kind;
	sat->boxAxis = box_axis;
	sat->triangleEdge = triangle_edge;
}

static int	other_axes(const Obb *box, const Vec3 *triangle, TriangleSat *sat)
{
	Vec3	axis;
	Vec3	normal;
	float	penetration;
	int		k;

	k = 0;
	while (k < 3)
	{
		if (!overlap_on(box, triangle, box->axis[k], &penetration, &normal))
			return (0);
		keep_axis(sat, penetration, normal, AXIS_BOX_FACE, k, 0);
		k++;
	}
	k = 0;
	while (k < 9)
	{
		axis = vec3_cross(box->axis[k / 3], vec3_normalized(vec3_sub(triangle[(k % 3 + 1) % 3], triangle[k % 3])));
		if (vec3_length_sq(axis) > 0.000001f && !overlap_on(box, triangle, vec3_normalized(axis), &penetration, &normal))
			return (0);
		if (vec3_length_sq(axis) > 0.000001f)
			keep_axis(sat, penetration, normal, AXIS_EDGES, k / 3, k % 3);
		k++;
	}
	return (1);
}

static void	triangle_face_contacts(BoxQuery *query, const Vec3 *triangle, Vec3 face_normal)
{
	Vec3	polygon[12];
	Vec3	side;
	float	depth;
	int		count;
	int		k;

	obb_incident_face(&query->box, face_normal, polygon);
	count = 4;
	k = 0;
	while (k < 3 && count > 0)
	{
		side = vec3_normalized(vec3_cross(vec3_sub(triangle[(k + 1) % 3], triangle[k]), face_normal));
		count = manifold_clip(polygon, count, side, vec3_dot(side, triangle[k]));   // [F16]
		k++;
	}
	k = 0;
	while (k < count)
	{
		depth = vec3_dot(face_normal, triangle[0]) - vec3_dot(face_normal, polygon[k]);
		if (depth > 0.0f)
			manifold_add_candidate(&query->set, polygon[k], vec3_neg(face_normal), depth);
		k++;
	}
}

static void	box_face_contacts(BoxQuery *query, const Vec3 *triangle, const TriangleSat *sat)
{
	const Obb	*box = &query->box;
	Vec3		polygon[12];
	float		center_along;
	float		depth;
	int			count;
	int			i;

	memcpy(polygon, triangle, 3 * sizeof(Vec3));
	count = 3;
	i = 0;
	while (i < 3 && count > 0)
	{
		center_along = vec3_dot(box->axis[i], box->center);
		if (i != sat->boxAxis)
			count = manifold_clip(polygon, count, box->axis[i], center_along + box->half[i]);
		if (i != sat->boxAxis && count > 0)
			count = manifold_clip(polygon, count, vec3_neg(box->axis[i]), -(center_along - box->half[i]));
		i++;
	}
	i = 0;
	while (i < count)
	{
		depth = vec3_dot(sat->normal, box->center) + box->half[sat->boxAxis] - vec3_dot(sat->normal, polygon[i]);
		if (depth > 0.0f)
			manifold_add_candidate(&query->set, polygon[i], sat->normal, depth);
		i++;
	}
}

static void	box_visit(void *context, int triangle_index)
{
	BoxQuery	*query = context;
	TriangleSat	sat;
	Vec3		triangle[3];
	Vec3		box_edge[2];
	Vec3		closest[2];

	mesh_triangle_corners(query->mesh, triangle_index, triangle);
	if (!triangle_axis(&query->box, triangle, query->mesh->faceNormals[triangle_index], &sat) || !other_axes(&query->box, triangle, &sat))
		return ;
	if (sat.kind == AXIS_TRIANGLE_FACE)
		triangle_face_contacts(query, triangle, query->mesh->faceNormals[triangle_index]);
	else if (sat.kind == AXIS_BOX_FACE)
		box_face_contacts(query, triangle, &sat);
	else
	{
		obb_support_edge(&query->box, sat.boxAxis, sat.normal, &box_edge[0], &box_edge[1]);
		closest_on_segments(box_edge[0], box_edge[1], triangle[sat.triangleEdge], triangle[(sat.triangleEdge + 1) % 3], &closest[0], &closest[1]);
		manifold_add_candidate(&query->set, vec3_scale(vec3_add(closest[0], closest[1]), 0.5f), sat.normal, sat.penetration);
	}
}

int	contact_box_mesh(const RigidBody *box, const RigidBody *mesh, Contact *out)
{
	BoxQuery	query;
	Quat		inverse = quat_conjugate(mesh->orientation);
	Vec3		extent = vec3(0.0f, 0.0f, 0.0f);
	int			i;

	query.mesh = mesh->collider.mesh;
	query.box = obb_from_body(box);
	query.box.center = mesh_to_local(mesh, query.box.center);
	query.set.count = 0;
	i = 0;
	while (i < 3)
	{
		query.box.axis[i] = quat_rotate(inverse, query.box.axis[i]);
		extent = vec3_add(extent, vec3_scale(vec3(fabsf(query.box.axis[i].x), fabsf(query.box.axis[i].y), fabsf(query.box.axis[i].z)), query.box.half[i]));   // [F10]
		i++;
	}
	bvh_query(query.mesh, vec3_sub(query.box.center, extent), vec3_add(query.box.center, extent), box_visit, &query);
	i = 0;
	while (i < query.set.count)
	{
		query.set.contacts[i].point = mesh_to_world(mesh, query.set.contacts[i].point);
		query.set.contacts[i].normal = quat_rotate(mesh->orientation, query.set.contacts[i].normal);
		i++;
	}
	return (manifold_reduce_candidates(&query.set, out));
}
