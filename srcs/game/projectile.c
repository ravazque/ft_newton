
#include "newton.h"

/* A bird is the bird.csv body launched with a velocity; the mass override
 * is what the Q / E controls tune. */
RigidBody	projectile_make_bird(const ObjectDef *bird, Vec3 position, Vec3 velocity, float mass)
{
	RigidBody	b;

	b = objectdef_make_body(bird, position);
	b.velocity = velocity;
	rb_set_mass(&b, mass);
	return (b);
}
