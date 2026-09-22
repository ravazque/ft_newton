#include "newton.h"

/*
 * Contact response for one step: wake the sleepers an awake body touches, prepare
 * every contact, warm start it from last step's impulses, run solverIterations
 * Gauss-Seidel passes of impulse_solve (impulse.c), remove the leftover overlap
 * (correction.c) and keep this step's contacts for the next warm start.
*/

static void	wake(RigidBody *b)
{
	b->awake = 1;
	b->sleepTimer = 0.0f;
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
			wake(a);
		if (!b->awake && a->awake && a->invMass > 0.0f)
			wake(b);
		i++;
	}
}

/* Last step's contact for the same pair at (almost) the same point. Contacts keep
 * their order in a resting scene, so the scan resumes from the previous hit. */
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

/* A resting stack converges across steps instead of re-solving from zero every time. */
static void	warm_start(World *w, Contact *c, int *cursor)
{
	const Contact	*prev = find_previous(w, c, cursor);
	Vec3			impulse;

	if (!prev)
		return ;
	c->jn = prev->jn;
	c->jt1 = prev->jt1;
	c->jt2 = prev->jt2;
	impulse = vec3_add(vec3_scale(c->normal, c->jn), vec3_add(vec3_scale(c->t1, c->jt1), vec3_scale(c->t2, c->jt2)));
	impulse_apply(&w->bodies[c->a], &w->bodies[c->b], c, impulse);
}

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
		impulse_prepare(w, &w->contacts[i++]);
	cursor = 0;
	i = 0;
	while (i < w->contactCount)
		warm_start(w, &w->contacts[i++], &cursor);
	iteration = 0;
	while (iteration < w->solverIterations)
	{
		i = 0;
		while (i < w->contactCount)
			impulse_solve(w, &w->contacts[i++]);
		iteration++;
	}
	correction_apply(w);
	remember_contacts(w);
}
