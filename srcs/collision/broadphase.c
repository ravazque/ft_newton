
#include "newton.h"

#include <float.h>

/*
 * Broad-phase: the cheap first pass. Every body gets a world-space AABB and a
 * sweep-and-prune runs along the X axis: entries are sorted by their minimum
 * X, so a pair can only overlap while the next entry starts before the
 * current one ends. Pairs that also overlap on Y and Z go to w->pairs for the
 * narrow-phase. O(n log n + k) instead of the brute force O(n^2) the subject
 * forbids at scale.
 */

typedef struct SweepEntry
{
	int		index;
	Vec3	mn;
	Vec3	mx;
}	SweepEntry;

static void	body_aabb(const RigidBody *b, Vec3 *mn, Vec3 *mx)
{
	if (b->collider.type == SHAPE_SPHERE)
	{
		float	r = b->collider.radius;

		*mn = vec3_sub(b->position, vec3(r, r, r));
		*mx = vec3_add(b->position, vec3(r, r, r));
	}
	else if (b->collider.type == SHAPE_BOX)
	{
		Vec3	h = b->collider.halfExtents;
		Vec3	a0 = quat_rotate(b->orientation, vec3(1.0f, 0.0f, 0.0f));
		Vec3	a1 = quat_rotate(b->orientation, vec3(0.0f, 1.0f, 0.0f));
		Vec3	a2 = quat_rotate(b->orientation, vec3(0.0f, 0.0f, 1.0f));
		Vec3	ex;

		ex.x = fabsf(a0.x) * h.x + fabsf(a1.x) * h.y + fabsf(a2.x) * h.z;
		ex.y = fabsf(a0.y) * h.x + fabsf(a1.y) * h.y + fabsf(a2.y) * h.z;
		ex.z = fabsf(a0.z) * h.x + fabsf(a1.z) * h.y + fabsf(a2.z) * h.z;
		*mn = vec3_sub(b->position, ex);
		*mx = vec3_add(b->position, ex);
	}
	else
	{
		*mn = vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		*mx = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
}

static int	cmp_entries(const void *pa, const void *pb)
{
	const SweepEntry	*a = pa;
	const SweepEntry	*b = pb;

	if (a->mn.x < b->mn.x)
		return (-1);
	return (a->mn.x > b->mn.x);
}

static int	push_pair(World *w, int a, int b)
{
	if (w->pairCount == w->pairCapacity)
	{
		int		newCap;
		Pair	*grown;

		newCap = 64;
		if (w->pairCapacity > 0)
			newCap = w->pairCapacity * 2;
		grown = realloc(w->pairs, (size_t)newCap * sizeof(Pair));
		if (!grown)
			return (0);
		w->pairs = grown;
		w->pairCapacity = newCap;
	}
	w->pairs[w->pairCount].a = a;
	w->pairs[w->pairCount].b = b;
	w->pairCount++;
	return (1);
}

static int	overlap_yz(const SweepEntry *a, const SweepEntry *b)
{
	return (a->mn.y <= b->mx.y && b->mn.y <= a->mx.y
		&& a->mn.z <= b->mx.z && b->mn.z <= a->mx.z);
}

void	broadphase_compute_pairs(World *w)
{
	SweepEntry	*e;
	int			i;
	int			j;

	w->pairCount = 0;
	if (w->bodyCount < 2)
		return ;
	e = malloc((size_t)w->bodyCount * sizeof(SweepEntry));
	if (!e)
		return ;
	i = 0;
	while (i < w->bodyCount)
	{
		e[i].index = i;
		body_aabb(&w->bodies[i], &e[i].mn, &e[i].mx);
		i++;
	}
	qsort(e, (size_t)w->bodyCount, sizeof(SweepEntry), cmp_entries);
	i = 0;
	while (i < w->bodyCount)
	{
		j = i + 1;
		while (j < w->bodyCount && e[j].mn.x <= e[i].mx.x)
		{
			if (overlap_yz(&e[i], &e[j])
				&& !(w->bodies[e[i].index].collider.type == SHAPE_PLANE
					&& w->bodies[e[j].index].collider.type == SHAPE_PLANE))
				push_pair(w, e[i].index, e[j].index);
			j++;
		}
		i++;
	}
	free(e);
}
