#include "newton.h"

// Sweep and prune: boxes sorted by their minimum X, neighbours that overlap on X tested on Y and Z [F10] [F11].

typedef struct SweepEntry
{
	int		body;
	Vec3	boundsMin;
	Vec3	boundsMax;
}	SweepEntry;

static int	compare_entries(const void *first, const void *second)
{
	const SweepEntry	*a = first;
	const SweepEntry	*b = second;

	if (a->boundsMin.x < b->boundsMin.x)
		return (-1);
	return (a->boundsMin.x > b->boundsMin.x);
}

// Lower index first, so a pair keeps its normal sign and its warm-start memory between steps.
static int	push_pair(World *world, int body_a, int body_b)
{
	Pair	*grown;
	int		new_capacity;

	if (world->pairCount == world->pairCapacity)
	{
		new_capacity = 64;
		if (world->pairCapacity > 0)
			new_capacity = world->pairCapacity * 2;
		grown = realloc(world->pairs, (size_t)new_capacity * sizeof(Pair));
		if (!grown)
			return (0);
		world->pairs = grown;
		world->pairCapacity = new_capacity;
	}
	world->pairs[world->pairCount].bodyA = body_a;
	world->pairs[world->pairCount].bodyB = body_b;
	if (body_a > body_b)
	{
		world->pairs[world->pairCount].bodyA = body_b;
		world->pairs[world->pairCount].bodyB = body_a;
	}
	world->pairCount++;
	return (1);
}

static int	compare_pairs(const void *first, const void *second)
{
	const Pair	*a = first;
	const Pair	*b = second;

	if (a->bodyA != b->bodyA)
		return (a->bodyA - b->bodyA);
	return (a->bodyB - b->bodyB);
}

static int	overlap_yz(const SweepEntry *a, const SweepEntry *b)
{
	return (a->boundsMin.y <= b->boundsMax.y && b->boundsMin.y <= a->boundsMax.y && a->boundsMin.z <= b->boundsMax.z && b->boundsMin.z <= a->boundsMax.z);   // [F11]
}

static int	pair_is_active(const World *world, int body_a, int body_b)
{
	const RigidBody	*first = &world->bodies[body_a];
	const RigidBody	*second = &world->bodies[body_b];

	if (!(first->awake && first->inverseMass > 0.0f) && !(second->awake && second->inverseMass > 0.0f))
		return (0);
	return (!joint_ignores_pair(world, body_a, body_b));
}

void	broadphase_compute_pairs(World *world)
{
	SweepEntry	*entries;
	int			i;
	int			j;

	world->pairCount = 0;
	if (world->bodyCount < 2)
		return ;
	entries = malloc((size_t)world->bodyCount * sizeof(SweepEntry));
	if (!entries)
		return ;
	i = 0;
	while (i < world->bodyCount)
	{
		entries[i].body = i;
		collider_bounds(&world->bodies[i], &entries[i].boundsMin, &entries[i].boundsMax);
		i++;
	}
	qsort(entries, (size_t)world->bodyCount, sizeof(SweepEntry), compare_entries);
	i = 0;
	while (i < world->bodyCount)
	{
		j = i + 1;
		while (j < world->bodyCount && entries[j].boundsMin.x <= entries[i].boundsMax.x)
		{
			if (overlap_yz(&entries[i], &entries[j]) && pair_is_active(world, entries[i].body, entries[j].body))
				push_pair(world, entries[i].body, entries[j].body);
			j++;
		}
		i++;
	}
	free(entries);
	qsort(world->pairs, (size_t)world->pairCount, sizeof(Pair), compare_pairs);
}
