
#include "newton.h"

/* An "apple" is just a spherical rigid body; centralizing its construction
 * keeps the catapult and any other spawner consistent. */
RigidBody	projectile_make_apple(Vec3 position, Vec3 velocity, float mass, float radius)
{
	RigidBody	b;

	b = rb_make();
	b.position = position;
	b.velocity = velocity;
	b.collider = collider_sphere(radius);
	b.restitution = 0.45f;
	rb_set_mass(&b, mass);
	return (b);
}
