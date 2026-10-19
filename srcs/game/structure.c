#include "newton.h"

// Walls, pyramids and towers of block.csv boxes on the XY plane, lifted onto whatever occupies their spot.

static float	place_blocks(World *world, const ObjectDef *block, const Vec3 *positions, int count)
{
	RigidBody	*blocks = malloc((size_t)count * sizeof(RigidBody));
	float		lift;
	int			i;

	if (!blocks)
		return (-1.0f);
	i = 0;
	while (i < count)
	{
		blocks[i] = object_def_make_body(block, positions[i]);
		i++;
	}
	lift = placement_add(world, blocks, count);
	free(blocks);
	return (lift);
}

// Centre of the block in 'column' of a row of 'count' centred on origin.x, 'row' rows up.
static Vec3	block_slot(const ObjectDef *block, Vec3 origin, int column, int count, int row)
{
	float	x = origin.x + ((float)column - (float)(count - 1) * 0.5f) * block->size.x * BLOCK_GAP;
	float	y = origin.y + block->size.y * 0.5f + (float)row * block->size.y * BLOCK_GAP;

	return (vec3(x, y, origin.z));
}

float	structure_spawn_wall(World *world, const ObjectDef *block, Vec3 origin, int columns, int rows)
{
	Vec3	*positions = malloc((size_t)(columns * rows) * sizeof(Vec3));
	float	lift;
	int		i;

	if (!positions)
		return (-1.0f);
	i = 0;
	while (i < columns * rows)
	{
		positions[i] = block_slot(block, origin, i % columns, columns, i / columns);
		i++;
	}
	lift = place_blocks(world, block, positions, columns * rows);
	free(positions);
	return (lift);
}

float	structure_spawn_pyramid(World *world, const ObjectDef *block, Vec3 origin, int base_count)
{
	Vec3	*positions = malloc((size_t)(base_count * (base_count + 1) / 2) * sizeof(Vec3));
	float	lift;
	int		count;
	int		row;
	int		column;

	if (!positions)
		return (-1.0f);
	count = 0;
	row = 0;
	while (row < base_count)
	{
		column = 0;
		while (column < base_count - row)
		{
			positions[count] = block_slot(block, origin, column, base_count - row, row);
			count++;
			column++;
		}
		row++;
	}
	lift = place_blocks(world, block, positions, count);
	free(positions);
	return (lift);
}

float	structure_spawn_tower(World *world, const ObjectDef *block, Vec3 origin, int height)
{
	return (structure_spawn_wall(world, block, origin, 1, height));
}
