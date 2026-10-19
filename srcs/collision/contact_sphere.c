#include "newton.h"

// A sphere against a sphere, a plane and a box: one contact each, normal from the sphere toward the other body.

int	contact_sphere_sphere(const RigidBody *sphere_a, const RigidBody *sphere_b, Contact *out)
{
	Vec3	offset = vec3_sub(sphere_b->position, sphere_a->position);   // [F12]
	float	radius_sum = sphere_a->collider.radius + sphere_b->collider.radius;
	float	distance = vec3_length(offset);

	if (distance > radius_sum)
		return (0);
	out->normal = vec3(0.0f, 1.0f, 0.0f);
	if (distance > 0.0001f)
		out->normal = vec3_scale(offset, 1.0f / distance);
	out->penetration = radius_sum - distance;
	out->point = vec3_add(sphere_a->position, vec3_scale(out->normal, sphere_a->collider.radius - out->penetration * 0.5f));
	return (1);
}

int	contact_sphere_plane(const RigidBody *sphere, const RigidBody *plane, Contact *out)
{
	float	distance = collider_plane_distance(&plane->collider, sphere->position);   // [F13]

	if (distance > sphere->collider.radius || !collider_plane_covers(&plane->collider, sphere->position))
		return (0);
	out->normal = vec3_neg(plane->collider.normal);
	out->penetration = sphere->collider.radius - distance;
	out->point = vec3_add(sphere->position, vec3_scale(out->normal, sphere->collider.radius));
	return (1);
}

// A centre inside the box goes out through the nearest face.
static int	sphere_inside_box(const RigidBody *sphere, const Obb *box, const float *local, Contact *out)
{
	Vec3	direction;
	int		nearest;
	int		i;

	nearest = 0;
	i = 1;
	while (i < 3)
	{
		if (box->half[i] - fabsf(local[i]) < box->half[nearest] - fabsf(local[nearest]))
			nearest = i;
		i++;
	}
	direction = box->axis[nearest];
	if (local[nearest] < 0.0f)
		direction = vec3_neg(direction);
	out->normal = vec3_neg(direction);
	out->point = sphere->position;
	out->penetration = box->half[nearest] - fabsf(local[nearest]) + sphere->collider.radius;
	return (1);
}

int	contact_sphere_box(const RigidBody *sphere, const RigidBody *box_body, Contact *out)
{
	Obb		box = obb_from_body(box_body);
	Vec3	offset = vec3_sub(sphere->position, box.center);
	Vec3	closest = box.center;
	float	local[3];
	int		inside;
	int		i;

	inside = 1;
	i = 0;
	while (i < 3)
	{
		local[i] = vec3_dot(offset, box.axis[i]);
		if (fabsf(local[i]) > box.half[i])
			inside = 0;
		local[i] = clampf(local[i], -box.half[i], box.half[i]);   // [F14]
		closest = vec3_add(closest, vec3_scale(box.axis[i], local[i]));
		i++;
	}
	if (inside)
		return (sphere_inside_box(sphere, &box, local, out));
	offset = vec3_sub(sphere->position, closest);
	if (vec3_length_sq(offset) > sphere->collider.radius * sphere->collider.radius)
		return (0);
	out->penetration = sphere->collider.radius - vec3_length(offset);
	out->normal = vec3_neg(vec3_normalized(offset));
	out->point = closest;
	return (1);
}
