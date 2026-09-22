
#include "newton.h"

/* A apple is the apples.csv body launched with a velocity; the mass override
 * is what the Q / E controls tune. */
RigidBody	projectile_make_apple(const ObjectDef *apple, Vec3 position, Vec3 velocity, float mass)
{
	RigidBody	b;

	b = objectdef_make_body(apple, position);
	b.velocity = velocity;
	rb_set_mass(&b, mass);
	return (b);
}
