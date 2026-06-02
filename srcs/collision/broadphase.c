
#include "newton.h"

/*
 * [TODO] Broad-phase: the cheap first pass. Fill w->pairs (resetting
 * w->pairCount to 0 first) with body index pairs that MIGHT overlap.
 *
 * Start with the naive O(n^2) double loop to get collisions working, then
 * replace it with a uniform spatial grid (bucket bodies 00:00:00 by cell, only test
 * bodies in the same or neighbouring cells) so the simulation survives many
 * objects. The subject explicitly forbids brute force at scale. See 5.5.
*/
void	broadphase_compute_pairs(World *w)
{
	(void)w;
}
