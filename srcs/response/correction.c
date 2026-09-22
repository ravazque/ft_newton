#include "newton.h"

/*
 * Positional correction [F23]: after the velocity solver, the overlap left beyond
 * PENETRATION_SLOP is removed by moving and slightly rotating the bodies through
 * the same effective masses, a fraction per pass. Several passes let a push travel
 * through a stack within one step (ground -> bottom box -> next box...).
*/

/* Overlap left at a contact: its depth minus how far this step's corrections moved the two points apart. */
static float	current_depth(const World *w, const Contact *c, Vec3 ra, Vec3 rb)
{
	Vec3	moved_a = vec3_add(vec3_sub(w->bodies[c->a].position, w->startPositions[c->a]), vec3_cross(w->rotationDelta[c->a], ra));
	Vec3	moved_b = vec3_add(vec3_sub(w->bodies[c->b].position, w->startPositions[c->b]), vec3_cross(w->rotationDelta[c->b], rb));

	return (c->penetration - vec3_dot(c->normal, vec3_sub(moved_b, moved_a)));
}

/* A positional "impulse": translation by 1/m and a small rotation by I^-1, applied to the pose. */
static void	push_body(World *w, int index, Vec3 r, Vec3 push)
{
	RigidBody	*b = &w->bodies[index];
	Vec3		rotation;

	if (b->invMass == 0.0f)
		return ;
	b->position = vec3_add(b->position, vec3_scale(push, b->invMass));   /* [F23] x += c n / m */
	rotation = mat3_mul_vec3(b->invInertiaWorld, vec3_cross(r, push));   /* [F23] I^-1 (r x c n) */
	w->rotationDelta[index] = vec3_add(w->rotationDelta[index], rotation);
	b->orientation = quat_integrate(b->orientation, rotation, 1.0f);
}

static void	correction_pass(World *w)
{
	const Contact	*c;
	Vec3			ra;
	Vec3			rb;
	float			depth;
	float			amount;
	int				i;

	i = 0;
	while (i < w->contactCount)
	{
		c = &w->contacts[i];
		ra = vec3_sub(c->point, w->startPositions[c->a]);
		rb = vec3_sub(c->point, w->startPositions[c->b]);
		depth = clampf(current_depth(w, c, ra, rb) - PENETRATION_SLOP, 0.0f, MAX_CORRECTION);
		amount = depth * PENETRATION_PERCENT * c->massNormal;   /* [F23] */
		if (amount > 0.0f)
		{
			push_body(w, c->a, ra, vec3_scale(c->normal, -amount));
			push_body(w, c->b, rb, vec3_scale(c->normal, amount));
		}
		i++;
	}
}

void	correction_apply(World *w)
{
	int	pass;
	int	i;

	if (!world_grow_scratch(w))
		return ;
	i = 0;
	while (i < w->bodyCount)
	{
		w->startPositions[i] = w->bodies[i].position;
		w->rotationDelta[i] = vec3(0.0f, 0.0f, 0.0f);
		i++;
	}
	pass = 0;
	while (pass < POSITION_ITERATIONS)
	{
		correction_pass(w);
		pass++;
	}
}
