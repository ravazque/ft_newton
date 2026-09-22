
#include "newton.h"

/*
 * Collision response: sequential impulses with accumulated clamping.
 *
 * The narrow-phase already filled w->contacts (normal from a toward b, world
 * point, penetration). For every step:
 *   1) wake  - a sleeping body touched by an awake dynamic one wakes up.
 *   2) prepare - per contact: friction basis, effective masses (how much a
 *      unit impulse changes the relative velocity, including rotation) and
 *      the elasticity target. Only impacts faster than
 *      ELASTICITY_THRESHOLD bounce; resting contacts aim for zero relative
 *      speed, which is what kills the endless micro-bounce. A contact that
 *      already existed last step starts from last step's impulse (warm
 *      starting): resting stacks then converge across steps instead of
 *      re-solving from zero every time, which is what keeps a tall tower
 *      from creeping.
 *   3) iterate - Gauss-Seidel over all contacts, solverIterations times.
 *      Each pass computes the impulse that would cancel the current relative
 *      velocity, adds it to the accumulated total, clamps the TOTAL (normal
 *      >= 0: contacts only push; friction <= mu * normal: Coulomb) and applies
 *      only the difference. Clamping the total instead of each increment is
 *      what makes stacks converge instead of drifting.
 *   4) correct - overlapping pairs are pushed apart along the normal by a
 *      fraction of the penetration beyond a small slop, weighted by inverse
 *      mass, so stacks do not sink into each other while impulses converge.
*/

static Vec3	point_velocity(const RigidBody *b, Vec3 r)
{
	return (vec3_add(b->velocity, vec3_cross(b->angularVelocity, r)));   /* [F17] v_point = v + w x r */
}

/* Velocity of b's material point at the contact relative to a's. */
static Vec3	relative_velocity(const RigidBody *a, const RigidBody *b, const Contact *c)
{
	return (vec3_sub(point_velocity(b, vec3_sub(c->point, b->position)), point_velocity(a, vec3_sub(c->point, a->position))));   /* [F18] */
}

/* Effective mass along 'dir': 1 / (1/ma + 1/mb + angular terms). Zero when
 * neither body can move along it. */
static float	effective_mass(const RigidBody *a, const RigidBody *b, const Contact *c, Vec3 dir)
{
	Vec3	ra = vec3_sub(c->point, a->position);
	Vec3	rb = vec3_sub(c->point, b->position);
	Vec3	ta = vec3_cross(mat3_mul_vec3(a->invInertiaWorld, vec3_cross(ra, dir)), ra);
	Vec3	tb = vec3_cross(mat3_mul_vec3(b->invInertiaWorld, vec3_cross(rb, dir)), rb);
	float	k = a->invMass + b->invMass + vec3_dot(vec3_add(ta, tb), dir);   /* [F19] 1/mD */

	if (k <= 0.0f)
		return (0.0f);
	return (1.0f / k);
}

/* Any orthonormal pair perpendicular to n: the two friction directions. */
static void	tangent_basis(Vec3 n, Vec3 *t1, Vec3 *t2)
{
	if (fabsf(n.x) >= 0.57735f)
		*t1 = vec3_normalized(vec3(n.y, -n.x, 0.0f));
	else
		*t1 = vec3_normalized(vec3(0.0f, n.z, -n.y));
	*t2 = vec3_cross(n, *t1);
}

static void	wake_touching(World *w)
{
	RigidBody	*a;
	RigidBody	*b;
	int			i;

	i = 0;
	while (i < w->contactCount)
	{
		a = &w->bodies[w->contacts[i].a];
		b = &w->bodies[w->contacts[i].b];
		if (!a->awake && b->awake && b->invMass > 0.0f)
		{
			a->awake = 1;
			a->sleepTimer = 0.0f;
		}
		if (!b->awake && a->awake && a->invMass > 0.0f)
		{
			b->awake = 1;
			b->sleepTimer = 0.0f;
		}
		i++;
	}
}

/* Equal and opposite impulses at the contact point. */
static void	apply_pair_impulse(RigidBody *a, RigidBody *b, const Contact *c, Vec3 impulse)
{
	rb_apply_impulse_at_point(a, vec3_neg(impulse), c->point);
	rb_apply_impulse_at_point(b, impulse, c->point);
}

/* Finds last step's contact for the same pair at (almost) the same point.
 * Contacts keep their order between steps in a resting scene, so the scan
 * resumes from the previous hit and is O(1) amortized. */
static const Contact	*find_previous(const World *w, const Contact *c, int *cursor)
{
	const Contact	*p;
	int				n = w->prevContactCount;
	int				k;
	int				i;

	k = 0;
	while (k < n)
	{
		i = (*cursor + k) % n;
		p = &w->prevContacts[i];
		if (p->a == c->a && p->b == c->b && vec3_length_sq(vec3_sub(p->point, c->point)) <= WARM_START_RADIUS * WARM_START_RADIUS)
		{
			*cursor = (i + 1) % n;
			return (p);
		}
		k++;
	}
	return (NULL);
}

/* Basis, effective masses and elasticity target. The target comes from the
 * relative velocity BEFORE any impulse of this step is applied: warm starts
 * of neighbouring contacts move the bodies first and would otherwise be read
 * as a violent impact, turning a resting stack into a bounce. */
static void	prepare_contact(const World *w, Contact *c)
{
	const RigidBody	*a = &w->bodies[c->a];
	const RigidBody	*b = &w->bodies[c->b];
	float			vn;

	tangent_basis(c->normal, &c->t1, &c->t2);
	c->massNormal = effective_mass(a, b, c, c->normal);
	c->massT1 = effective_mass(a, b, c, c->t1);
	c->massT2 = effective_mass(a, b, c, c->t2);
	vn = vec3_dot(relative_velocity(a, b, c), c->normal);                     /* [F18] */
	c->bias = 0.0f;
	if (vn < -ELASTICITY_THRESHOLD)
		c->bias = -fmaxf(a->elasticity, b->elasticity) * vn;                /* [F20] */
	c->jn = 0.0f;
	c->jt1 = 0.0f;
	c->jt2 = 0.0f;
}

/* Starts the contact from last step's accumulated impulses when it existed. */
static void	warm_start(World *w, Contact *c, int *cursor)
{
	const Contact	*prev = find_previous(w, c, cursor);

	if (!prev)
		return ;
	c->jn = prev->jn;
	c->jt1 = prev->jt1;
	c->jt2 = prev->jt2;
	apply_pair_impulse(&w->bodies[c->a], &w->bodies[c->b], c, vec3_add(vec3_scale(c->normal, c->jn), vec3_add(vec3_scale(c->t1, c->jt1), vec3_scale(c->t2, c->jt2))));
}

/* Solves one friction direction with the Coulomb bound |jt| <= mu * jn. */
static void	solve_tangent(RigidBody *a, RigidBody *b, const Contact *c, Vec3 t, float mass, float *accum, float max_friction)
{
	float	old = *accum;
	float	lambda = -mass * vec3_dot(relative_velocity(a, b, c), t);

	*accum = fminf(fmaxf(old + lambda, -max_friction), max_friction);   /* [F22] |jt| <= mu jn */
	apply_pair_impulse(a, b, c, vec3_scale(t, *accum - old));
}

static void	solve_contact(World *w, Contact *c)
{
	RigidBody	*a = &w->bodies[c->a];
	RigidBody	*b = &w->bodies[c->b];
	float		mu = sqrtf(a->friction * b->friction);   /* [F22] mu = sqrt(mu_a mu_b) */
	float		old;
	float		lambda;

	solve_tangent(a, b, c, c->t1, c->massT1, &c->jt1, mu * c->jn);
	solve_tangent(a, b, c, c->t2, c->massT2, &c->jt2, mu * c->jn);
	lambda = c->massNormal * (c->bias - vec3_dot(relative_velocity(a, b, c), c->normal));   /* [F21] lambda = -(v_rel_n + bias) mN */
	old = c->jn;
	c->jn = fmaxf(old + lambda, 0.0f);                                                      /* [F21] accumulated clamp, push only */
	apply_pair_impulse(a, b, c, vec3_scale(c->normal, c->jn - old));                        /* [F9]  */
}

/* Overlap at this contact right now: the narrow-phase depth minus what the
 * corrections already applied this step moved the two contact points apart
 * (translation plus the small rotation acting on the lever arm). */
static float	current_depth(const World *w, const Contact *c, Vec3 ra, Vec3 rb)
{
	Vec3	moved_a = vec3_add(vec3_sub(w->bodies[c->a].position, w->startPositions[c->a]), vec3_cross(w->rotationDelta[c->a], ra));
	Vec3	moved_b = vec3_add(vec3_sub(w->bodies[c->b].position, w->startPositions[c->b]), vec3_cross(w->rotationDelta[c->b], rb));

	return (c->penetration - vec3_dot(c->normal, vec3_sub(moved_b, moved_a)));
}

/* Moves one body by a positional "impulse" at the contact: translation by
 * 1/m and a small rotation by the inverse inertia, exactly like a velocity
 * impulse but applied to the pose. */
static void	push_body(World *w, int index, Vec3 r, Vec3 push)
{
	RigidBody	*b = &w->bodies[index];
	Vec3		rotation;

	if (b->invMass == 0.0f)
		return ;
	b->position = vec3_add(b->position, vec3_scale(push, b->invMass));   /* [F23] x += c n / m       */
	rotation = mat3_mul_vec3(b->invInertiaWorld, vec3_cross(r, push));   /* [F23] and I^-1 (r x c n) */
	w->rotationDelta[index] = vec3_add(w->rotationDelta[index], rotation);
	b->orientation = quat_integrate(b->orientation, rotation, 1.0f);
}

/* One pass over every contact. Each point pushes the two bodies apart by a
 * fraction of its own overlap beyond the slop, through the same effective
 * mass the velocity solver uses, so a box resting on one deep corner is
 * straightened rather than lifted whole. */
static void	correction_pass(World *w)
{
	const Contact	*c;
	Vec3			ra;
	Vec3			rb;
	float			amount;
	int				i;

	i = 0;
	while (i < w->contactCount)
	{
		c = &w->contacts[i];
		ra = vec3_sub(c->point, w->startPositions[c->a]);
		rb = vec3_sub(c->point, w->startPositions[c->b]);
		amount = fminf(fmaxf(current_depth(w, c, ra, rb) - PENETRATION_SLOP, 0.0f), MAX_CORRECTION) * PENETRATION_PERCENT * c->massNormal;   /* [F23] */
		if (amount > 0.0f)
		{
			push_body(w, c->a, ra, vec3_scale(c->normal, -amount));
			push_body(w, c->b, rb, vec3_scale(c->normal, amount));
		}
		i++;
	}
}

/* Several passes let a correction travel through a stack within one step
 * (the ground pushes the bottom box, which then pushes the next...), so a
 * column does not slowly sink into itself. */
static void	correct_positions(World *w)
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

/* Keeps a copy of this step's contacts (with their final impulses) for the
 * next step's warm start. */
static void	remember_contacts(World *w)
{
	Contact	*grown;

	if (w->contactCount > w->prevContactCapacity)
	{
		grown = realloc(w->prevContacts, (size_t)w->contactCount * sizeof(Contact));
		if (!grown)
		{
			w->prevContactCount = 0;
			return ;
		}
		w->prevContacts = grown;
		w->prevContactCapacity = w->contactCount;
	}
	memcpy(w->prevContacts, w->contacts, (size_t)w->contactCount * sizeof(Contact));
	w->prevContactCount = w->contactCount;
}

void	resolver_resolve(World *w)
{
	int	iteration;
	int	cursor;
	int	i;

	wake_touching(w);
	i = 0;
	while (i < w->contactCount)
	{
		prepare_contact(w, &w->contacts[i]);
		i++;
	}
	cursor = 0;
	i = 0;
	while (i < w->contactCount)
	{
		warm_start(w, &w->contacts[i], &cursor);
		i++;
	}
	iteration = 0;
	while (iteration < w->solverIterations)
	{
		i = 0;
		while (i < w->contactCount)
		{
			solve_contact(w, &w->contacts[i]);
			i++;
		}
		iteration++;
	}
	correct_positions(w);
	remember_contacts(w);
}
