#include "newton.h"

/* The projectile: an apples.csv body with the launch velocity and the mass the Q / E keys set. */
RigidBody	projectile_make_apple(const ObjectDef *apple, Vec3 position, Vec3 velocity, float mass)
{
	RigidBody	b;

	b = objectdef_make_body(apple, position);
	b.velocity = velocity;
	rb_set_mass(&b, mass);
	return (b);
}
