#include "newton.h"

// The projectile: an apples.csv body with the launch velocity and the mass the Q and E keys set.

RigidBody	projectile_make_apple(const ObjectDef *apple, Vec3 position, Vec3 velocity, float mass)
{
	RigidBody	body = object_def_make_body(apple, position);

	body.velocity = velocity;
	rigidbody_set_mass(&body, mass);
	return (body);
}
