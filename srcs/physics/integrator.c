
#include "newton.h"

/*
 * [TODO] Semi-implicit (symplectic) Euler for one body, one fixed step.
 * See docs/info.md 5.2.
 *
 *   Linear:
 *     acc       = forceAccum * invMass + gravity
 *     velocity += acc * dt              // velocity FIRST
 *     position += velocity * dt         // then move with the NEW velocity
 *
 *   Angular:
 *     ang_acc          = invInertiaWorld * torqueAccum
 *     angularVelocity += ang_acc * dt
 *     orientation      = quat_integrate(orientation, angularVelocity, dt)
 *
 * Skip integration if !b->awake. Remember to refresh invInertiaWorld from the
 * orientation (R * invInertiaLocal * R^T) each step.
*/
void	integrator_integrate(RigidBody *b, Vec3 gravity, float dt)
{
	(void)b;
	(void)gravity;
	(void)dt;
}
