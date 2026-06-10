
#include "newton.h"

Catapult	catapult_default(void)
{
	Catapult	c;

	c.position = vec3(-12.0f, 1.0f, 0.0f);
	c.launchDirection = vec3(0.7f, 0.7f, 0.0f);
	c.launchSpeed = 22.0f;
	c.projectileMass = 1.0f;
	c.projectileRadius = 0.5f;
	return (c);
}

/* Builds an apple with the catapult's CURRENT parameters (speed, direction
 * and mass stay live-tunable) and adds it to the world. */
int	catapult_fire(Catapult *c, World *w)
{
	Vec3	dir;

	dir = vec3_normalized(c->launchDirection);
	if (vec3_length_sq(dir) == 0.0f)
		dir = vec3(1.0f, 0.0f, 0.0f);
	return (world_add_body(w, projectile_make_apple(c->position,
				vec3_scale(dir, c->launchSpeed), c->projectileMass,
				c->projectileRadius)));
}
