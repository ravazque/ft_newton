
#include "newton.h"

/*
 * [TODO] Semi-implicit (symplectic) Euler for one body, one fixed step.
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
 * Skip static bodies (invMass == 0): gravity must not move the ground.
 * Refresh invInertiaWorld from the orientation (R * invInertiaLocal * R^T)
 * each step so torques keep working while the body rotates.
 */
void	integrator_integrate(RigidBody *b, Vec3 gravity, float dt)
{
	(void)b;
	(void)gravity;
	(void)dt;
}
