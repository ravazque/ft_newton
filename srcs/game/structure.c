
#include "newton.h"

/*
 * [TODO] Build the "unstable composed structures" the apples knock down: stacks
 * and walls of box rigid bodies. This is also the easy way to spawn MANY
 * objects on demand (a hard requirement) and the stress case the broad-phase
 * must survive.
 *
 * For each box: rb_make(), set collider_box(half), set position from the grid,
 * rb_set_mass(...), then world_add_body(w, box).
*/

void	structure_spawn_wall(World *w, Vec3 origin, int columns, int rows)
{
	(void)w;
	(void)origin;
	(void)columns;
	(void)rows;
}

void	structure_spawn_pyramid(World *w, Vec3 origin, int base_count)
{
	(void)w;
	(void)origin;
	(void)base_count;
}

void	structure_spawn_tower(World *w, Vec3 origin, int height)
{
	(void)w;
	(void)origin;
	(void)height;
}
