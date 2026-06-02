
#include "newton.h"

/*
 * [TODO] Narrow-phase: the exact second pass. For each pair in w->pairs, run
 * the precise overlap test for that shape combination and, if they touch,
 * append a Contact (normal + point + penetration) to w->contacts (reset
 * w->contactCount to 0 first).
 *
 * Tests to implement: sphere-sphere, sphere-plane, box-plane, box-box (SAT,
 * the Separating Axis Theorem). Box-sphere can be added similarly. See 5.5.
*/
void	narrowphase_generate_contacts(World *w)
{
	(void)w;
}
