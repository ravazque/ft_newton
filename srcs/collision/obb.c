#include "newton.h"

// A box body as an oriented box, and the geometric queries the box contacts share.

Obb	obb_from_body(const RigidBody *body)
{
	Obb	box;

	box.center = body->position;
	box.axis[0] = quat_rotate(body->orientation, vec3(1.0f, 0.0f, 0.0f));
	box.axis[1] = quat_rotate(body->orientation, vec3(0.0f, 1.0f, 0.0f));
	box.axis[2] = quat_rotate(body->orientation, vec3(0.0f, 0.0f, 1.0f));
	box.half[0] = body->collider.halfExtents.x;
	box.half[1] = body->collider.halfExtents.y;
	box.half[2] = body->collider.halfExtents.z;
	return (box);
}

float	obb_radius(const Obb *box, Vec3 direction)
{
	return (box->half[0] * fabsf(vec3_dot(box->axis[0], direction)) + box->half[1] * fabsf(vec3_dot(box->axis[1], direction)) + box->half[2] * fabsf(vec3_dot(box->axis[2], direction)));   // [F15]
}

// Bit k of 'corner' picks the + or - side along axis k.
Vec3	obb_corner(const Obb *box, int corner)
{
	Vec3	point = box->center;
	float	sign;
	int		axis;

	axis = 0;
	while (axis < 3)
	{
		sign = -1.0f;
		if (corner & (1 << axis))
			sign = 1.0f;
		point = vec3_add(point, vec3_scale(box->axis[axis], sign * box->half[axis]));
		axis++;
	}
	return (point);
}

void	obb_support_edge(const Obb *box, int axis_index, Vec3 direction, Vec3 *start, Vec3 *end)
{
	Vec3	middle = box->center;
	float	sign;
	int		i;

	i = 0;
	while (i < 3)
	{
		sign = -1.0f;
		if (vec3_dot(box->axis[i], direction) >= 0.0f)
			sign = 1.0f;
		if (i != axis_index)
			middle = vec3_add(middle, vec3_scale(box->axis[i], sign * box->half[i]));
		i++;
	}
	*start = vec3_sub(middle, vec3_scale(box->axis[axis_index], box->half[axis_index]));
	*end = vec3_add(middle, vec3_scale(box->axis[axis_index], box->half[axis_index]));
}

void	obb_incident_face(const Obb *box, Vec3 reference_normal, Vec3 *polygon)
{
	Vec3	face_normal;
	Vec3	face_center;
	int		facing;
	int		side_u;
	int		side_v;

	facing = 0;
	if (fabsf(vec3_dot(box->axis[1], reference_normal)) > fabsf(vec3_dot(box->axis[facing], reference_normal)))
		facing = 1;
	if (fabsf(vec3_dot(box->axis[2], reference_normal)) > fabsf(vec3_dot(box->axis[facing], reference_normal)))
		facing = 2;
	side_u = (facing + 1) % 3;
	side_v = (facing + 2) % 3;
	face_normal = box->axis[facing];
	if (vec3_dot(face_normal, reference_normal) > 0.0f)
		face_normal = vec3_neg(face_normal);
	face_center = vec3_add(box->center, vec3_scale(face_normal, box->half[facing]));
	polygon[0] = vec3_add(face_center, vec3_add(vec3_scale(box->axis[side_u], box->half[side_u]), vec3_scale(box->axis[side_v], box->half[side_v])));
	polygon[1] = vec3_add(face_center, vec3_add(vec3_scale(box->axis[side_u], -box->half[side_u]), vec3_scale(box->axis[side_v], box->half[side_v])));
	polygon[2] = vec3_add(face_center, vec3_add(vec3_scale(box->axis[side_u], -box->half[side_u]), vec3_scale(box->axis[side_v], -box->half[side_v])));
	polygon[3] = vec3_add(face_center, vec3_add(vec3_scale(box->axis[side_u], box->half[side_u]), vec3_scale(box->axis[side_v], -box->half[side_v])));
}
