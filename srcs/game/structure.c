
#include "newton.h"

/*
 * Spawners for the "unstable composed structures" the apples knock down, and
 * the easy way to add MANY bodies on demand. Boxes are 1x1x1, stacked from
 * origin.y upward on the XY gameplay plane (z = 0 by default).
 */

#define BOX_SPACING 1.001f

static void	spawn_box(World *w, Vec3 position)
{
	RigidBody	box;

	box = rb_make();
	box.position = position;
	box.collider = collider_box(vec3(0.5f, 0.5f, 0.5f));
	box.restitution = 0.2f;
	rb_set_mass(&box, 1.0f);
	world_add_body(w, box);
}

void	structure_spawn_wall(World *w, Vec3 origin, int columns, int rows)
{
	int	c;
	int	r;

	r = 0;
	while (r < rows)
	{
		c = 0;
		while (c < columns)
		{
			spawn_box(w, vec3(
					origin.x + ((float)c - (float)(columns - 1) * 0.5f) * BOX_SPACING,
					origin.y + 0.5f + (float)r * BOX_SPACING,
					origin.z));
			c++;
		}
		r++;
	}
}

void	structure_spawn_pyramid(World *w, Vec3 origin, int base_count)
{
	int	row;
	int	c;
	int	n;

	row = 0;
	while (row < base_count)
	{
		n = base_count - row;
		c = 0;
		while (c < n)
		{
			spawn_box(w, vec3(
					origin.x + ((float)c - (float)(n - 1) * 0.5f) * BOX_SPACING,
					origin.y + 0.5f + (float)row * BOX_SPACING,
					origin.z));
			c++;
		}
		row++;
	}
}

void	structure_spawn_tower(World *w, Vec3 origin, int height)
{
	int	i;

	i = 0;
	while (i < height)
	{
		spawn_box(w, vec3(origin.x, origin.y + 0.5f + (float)i * BOX_SPACING,
				origin.z));
		i++;
	}
}
