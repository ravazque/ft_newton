
#include "newton.h"

/*
 * Spawners for the unstable structures the birds knock down, and
 * the easy way to add MANY bodies on demand. Blocks come from block.csv and
 * are stacked from origin.y upward on the XY gameplay plane, with a hair of
 * spacing so neighbours start separated instead of interpenetrating.
*/

#define BLOCK_GAP 1.002f

static void	spawn_block(World *w, const ObjectDef *block, Vec3 position)
{
	world_add_body(w, objectdef_make_body(block, position));
}

void	structure_spawn_wall(World *w, const ObjectDef *block, Vec3 origin, int columns, int rows)
{
	float	sx = block->size.x * BLOCK_GAP;
	float	sy = block->size.y * BLOCK_GAP;
	int		c;
	int		r;

	r = 0;
	while (r < rows)
	{
		c = 0;
		while (c < columns)
		{
			spawn_block(w, block, vec3(origin.x + ((float)c - (float)(columns - 1) * 0.5f) * sx, origin.y + block->size.y * 0.5f + (float)r * sy, origin.z));
			c++;
		}
		r++;
	}
}

void	structure_spawn_pyramid(World *w, const ObjectDef *block, Vec3 origin, int base_count)
{
	float	sx = block->size.x * BLOCK_GAP;
	float	sy = block->size.y * BLOCK_GAP;
	int		row;
	int		c;
	int		n;

	row = 0;
	while (row < base_count)
	{
		n = base_count - row;
		c = 0;
		while (c < n)
		{
			spawn_block(w, block, vec3(origin.x + ((float)c - (float)(n - 1) * 0.5f) * sx, origin.y + block->size.y * 0.5f + (float)row * sy, origin.z));
			c++;
		}
		row++;
	}
}

void	structure_spawn_tower(World *w, const ObjectDef *block, Vec3 origin, int height)
{
	float	sy = block->size.y * BLOCK_GAP;
	int		i;

	i = 0;
	while (i < height)
	{
		spawn_block(w, block, vec3(origin.x, origin.y + block->size.y * 0.5f + (float)i * sy, origin.z));
		i++;
	}
}
