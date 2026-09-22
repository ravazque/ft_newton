#include "newton.h"

/* Inertia tensors, the rotational counterpart of mass: built inverted in body space from the
 * collider, rotated into world space whenever the body turns [F5]-[F7]. */

Mat3	inertia_local_inverse(const Collider *c, float mass)
{
	Vec3	h;
	float	i;

	if (mass <= 0.0f || c->type == SHAPE_PLANE)
		return (mat3_zero());
	if (c->type == SHAPE_SPHERE)
	{
		i = 0.4f * mass * c->radius * c->radius;   /* [F5] I = 2/5 m R^2 */
		return (mat3_inverse(mat3_diagonal(vec3(i, i, i))));
	}
	h = c->halfExtents;
	return (mat3_inverse(mat3_diagonal(vec3_scale(vec3(h.y * h.y + h.z * h.z, h.x * h.x + h.z * h.z, h.x * h.x + h.y * h.y), mass / 3.0f))));   /* [F6] */
}

void	inertia_update_world(RigidBody *b)
{
	Mat3	r = mat3_from_quat(b->orientation);

	b->invInertiaWorld = mat3_mul(mat3_mul(r, b->invInertiaLocal), mat3_transpose(r));   /* [F7] I_world^-1 = R I^-1 R^T */
}
