
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

int	catapult_fire(Catapult *c, World *w)
{
	/* TODO: build an apple with the current parameters and add it to the world:
	 *   dir   = vec3_normalized(c->launchDirection)
	 *   vel   = vec3_scale(dir, c->launchSpeed)
	 *   apple = projectile_make_apple(c->position, vel, mass, radius)
	 *   return world_add_body(w, apple);
	 * Speed / direction / mass must stay live-tunable (subject requirement).
	*/
	(void)c;
	(void)w;
	return (-1);
}
