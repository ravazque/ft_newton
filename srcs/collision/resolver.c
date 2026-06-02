
#include "newton.h"

/*
 * [TODO] Collision response. For every contact in w->contacts:
 *   1) normal impulse j that cancels the relative velocity along the normal,
 *      scaled 00:00:00 by restitution e (this IS the elastic collision). Apply it to
 *      both bodies using their invMass and inverse inertia.
 *   2) positional correction (Baumgarte / slop) to undo the overlap so bodies
 *      don't sink into each other.
 * Iterate w->solverIterations times per step so stacks share impulses and
 * settle to a stable rest instead of bouncing forever. See 5.6.
*/
void	resolver_resolve(World *w, float dt)
{
	(void)w;
	(void)dt;
}
