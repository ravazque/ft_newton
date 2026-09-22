#include "newton.h"

/*
 * Broad-phase: every body gets a world AABB (collider_bounds) and a sweep-and-prune runs along X.
 * Entries sorted by their minimum X can only overlap while the next one starts
 * before the current one ends; those that also overlap on Y and Z become
 * candidate pairs. O(n log n + k) instead of testing all n^2 pairs [F10] [F11].
*/

typedef struct SweepEntry
{
	int		index;
	Vec3	mn;
	Vec3	mx;
}	SweepEntry;

static int	cmp_entries(const void *pa, const void *pb)
{
	const SweepEntry	*a = pa;
	const SweepEntry	*b = pb;

	if (a->mn.x < b->mn.x)
		return (-1);
	return (a->mn.x > b->mn.x);
}

/* Pairs are stored as (lower, higher) index so a pair keeps its normal sign and warm-start
 * identity between steps. */
static int	push_pair(World *w, int a, int b)
{
	Pair	*grown;
	int		new_capacity;

	if (w->pairCount == w->pairCapacity)
	{
		new_capacity = 64;
		if (w->pairCapacity > 0)
			new_capacity = w->pairCapacity * 2;
		grown = realloc(w->pairs, (size_t)new_capacity * sizeof(Pair));
		if (!grown)
			return (0);
		w->pairs = grown;
		w->pairCapacity = new_capacity;
	}
	w->pairs[w->pairCount].a = a;
	w->pairs[w->pairCount].b = b;
	if (a > b)
	{
		w->pairs[w->pairCount].a = b;
		w->pairs[w->pairCount].b = a;
	}
	w->pairCount++;
	return (1);
}

/* Sorted pairs make every step sweep the contacts in the same order, which lets a resting pile converge. */
static int	cmp_pairs(const void *pa, const void *pb)
{
	const Pair	*a = pa;
	const Pair	*b = pb;

	if (a->a != b->a)
		return (a->a - b->a);
	return (a->b - b->b);
}

static int	overlap_yz(const SweepEntry *a, const SweepEntry *b)
{
	return (a->mn.y <= b->mx.y && b->mn.y <= a->mx.y && a->mn.z <= b->mx.z && b->mn.z <= a->mx.z);   /* [F11] */
}

/* Only pairs where something can still move: at least one awake dynamic body. */
static int	pair_is_active(const RigidBody *a, const RigidBody *b)
{
	return ((a->awake && a->invMass > 0.0f) || (b->awake && b->invMass > 0.0f));
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
		collider_bounds(&w->bodies[i], &e[i].mn, &e[i].mx);
		i++;
	}
	qsort(e, (size_t)w->bodyCount, sizeof(SweepEntry), cmp_entries);
	i = 0;
	while (i < w->bodyCount)
	{
		j = i + 1;
		while (j < w->bodyCount && e[j].mn.x <= e[i].mx.x)
		{
			if (overlap_yz(&e[i], &e[j]) && pair_is_active(&w->bodies[e[i].index], &w->bodies[e[j].index]))
				push_pair(w, e[i].index, e[j].index);
			j++;
		}
		i++;
	}
	free(e);
	qsort(w->pairs, (size_t)w->pairCount, sizeof(Pair), cmp_pairs);
}
