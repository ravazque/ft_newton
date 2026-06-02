
#include "newton.h"

/*
 * [TODO] An "apple" is just a spherical rigid body. This helper centralizes its
 * construction so the catapult (and any spawner) stays consistent.
 *
 * Sketch:
 *   RigidBody b = rb_make();
 *   b.position = position;
 *   b.velocity = velocity;
 *   b.collider = collider_sphere(radius);
 *   rb_set_mass(&b, mass);     // also sets the inertia tensor
 *   return b;
*/
RigidBody	projectile_make_apple(Vec3 position, Vec3 velocity, float mass, float radius)
{
	RigidBody	b;

	b = rb_make();
	b.position = position;
	b.collider = collider_sphere(radius);
	(void)velocity;
	(void)mass;
	return (b);
}
