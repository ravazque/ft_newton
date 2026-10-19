#include "newton.h"

// Inverse inertia tensors: built in body space from the collider, rotated into world space when the body turns.

Mat3	inertia_local_inverse(const Collider *collider, float mass)
{
	Vec3	half;
	Vec3	squares;
	float	moment;

	if (mass <= 0.0f || collider->type == SHAPE_PLANE)
		return (mat3_zero());
	if (collider->type == SHAPE_SPHERE)
	{
		moment = 0.4f * mass * collider->radius * collider->radius;   // [F5] I = 2/5 m R^2
		return (mat3_inverse(mat3_diagonal(vec3(moment, moment, moment))));
	}
	if (collider->type == SHAPE_MESH)
		return (mat3_inverse(mat3_scale(collider->mesh->inertiaPerKg, mass)));   // [F27]
	half = collider->halfExtents;
	squares = vec3(half.y * half.y + half.z * half.z, half.x * half.x + half.z * half.z, half.x * half.x + half.y * half.y);
	return (mat3_inverse(mat3_diagonal(vec3_scale(squares, mass / 3.0f))));   // [F6]
}

void	inertia_update_world(RigidBody *body)
{
	Mat3	rotation = mat3_from_quat(body->orientation);

	body->inverseInertiaWorld = mat3_mul(mat3_mul(rotation, body->inverseInertiaLocal), mat3_transpose(rotation));   // [F7] I^-1 = R I^-1 R^T
}
