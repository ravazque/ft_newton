
#include "newton.h"

/*
 * [TODO] Collision response — the physics of the impact. Detection already
 * filled w->contacts: each contact gives you the two body indices, the
 * contact normal (unit vector pointing from a toward b), the contact point in
 * world space and the penetration depth.
 *
 * For every contact:
 *   1) Relative velocity at the contact point, projected on the normal. If
 *      the bodies are already separating (positive along the normal from a's
 *      point of view), skip the contact.
 *   2) Normal impulse j = -(1 + e) * velAlongNormal / (sum of invMass +
 *      angular terms), with e = combined restitution of both bodies. This IS
 *      the elastic collision. Apply -j*normal to a and +j*normal to b with
 *      rb_apply_impulse_at_point so off-center hits also make bodies spin.
 *   3) Positional correction (Baumgarte): push both bodies apart along the
 *      normal, proportionally to the penetration (minus a small slop, e.g.
 *      0.005) and to their inverse masses, so stacks do not sink into each
 *      other while impulses converge.
 *
 * Run the whole contact list w->solverIterations times per step so stacked
 * contacts share impulses and settle to a stable rest instead of bouncing
 * forever.
 */
void	resolver_resolve(World *w, float dt)
{
	(void)w;
	(void)dt;
}
