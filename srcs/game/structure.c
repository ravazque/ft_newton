#include "newton.h"

/*
 * Walls, pyramids and towers of block.csv boxes on the XY gameplay plane: the
 * unstable targets, and the way to spawn many bodies on demand. A new structure
 * never starts inside anything: when its spot is taken it is lifted until no
 * block would sink into what is there, so it lands on top of it.
*/

/* The most one block must rise to clear the top of what it would sink into (0 when it is free). */
static float	needed_lift(const World *w, const RigidBody *block)
{
	Vec3	mn;
	Vec3	mx;
	Vec3	block_mn;
	Vec3	block_mx;
	int		hit;

	hit = world_first_overlap(w, block);
	if (hit < 0)
		return (0.0f);
	collider_bounds(&w->bodies[hit], &mn, &mx);
	collider_bounds(block, &block_mn, &block_mx);
	return (mx.y - block_mn.y + SPAWN_CLEARANCE);
}

/* Lifts every block by the same amount until none sinks into anything: the structure lands on
 * what was there. */
static float	lift_until_free(const World *w, RigidBody *blocks, int n)
{
	float	lift = 0.0f;
	float	needed = 1.0f;
	int		guard;
	int		i;

	guard = 0;
	while (needed > 0.0f && guard <= w->bodyCount)
	{
		needed = 0.0f;
		i = 0;
		while (i < n)
			needed = fmaxf(needed, needed_lift(w, &blocks[i++]));
		i = 0;
		while (i < n)
			blocks[i++].position.y += needed;
		lift += needed;
		guard++;
	}
	return (lift);
}

/* Builds the blocks at 'positions', lifts them clear and adds them; returns the lift (m), -1
 * on allocation failure. */
static float	place(World *w, const ObjectDef *block, const Vec3 *positions, int n)
{
	RigidBody	*blocks = malloc((size_t)n * sizeof(RigidBody));
	float		lift;
	int			i;

	if (!blocks)
		return (-1.0f);
	i = 0;
	while (i < n)
	{
		blocks[i] = objectdef_make_body(block, positions[i]);
		i++;
	}
	lift = lift_until_free(w, blocks, n);
	i = 0;
	while (i < n)
		world_add_body(w, blocks[i++]);
	free(blocks);
	return (lift);
}

/* Center of the block in 'column' of a row of 'count' centred on origin.x, 'row' rows up. */
static Vec3	slot(const ObjectDef *block, Vec3 origin, int column, int count, int row)
{
	float	x = origin.x + ((float)column - (float)(count - 1) * 0.5f) * block->size.x * BLOCK_GAP;
	float	y = origin.y + block->size.y * 0.5f + (float)row * block->size.y * BLOCK_GAP;

	return (vec3(x, y, origin.z));
}

float	structure_spawn_wall(World *w, const ObjectDef *block, Vec3 origin, int columns, int rows)
{
	Vec3	*positions = malloc((size_t)(columns * rows) * sizeof(Vec3));
	float	lift;
	int		i;

	if (!positions)
		return (-1.0f);
	i = 0;
	while (i < columns * rows)
	{
		positions[i] = slot(block, origin, i % columns, columns, i / columns);
		i++;
	}
	lift = place(w, block, positions, columns * rows);
	free(positions);
	return (lift);
}

float	structure_spawn_pyramid(World *w, const ObjectDef *block, Vec3 origin, int base_count)
{
	Vec3	*positions = malloc((size_t)(base_count * (base_count + 1) / 2) * sizeof(Vec3));
	float	lift;
	int		n;
	int		row;
	int		c;

	if (!positions)
		return (-1.0f);
	n = 0;
	row = 0;
	while (row < base_count)
	{
		c = 0;
		while (c < base_count - row)
		{
			positions[n++] = slot(block, origin, c, base_count - row, row);
			c++;
		}
		row++;
	}
	lift = place(w, block, positions, n);
	free(positions);
	return (lift);
}

float	structure_spawn_tower(World *w, const ObjectDef *block, Vec3 origin, int height)
{
	return (structure_spawn_wall(w, block, origin, 1, height));
}
