#include "newton.h"

// Polygons to triangles by ear clipping on the plane of the Newell normal; each triangle keeps its face's winding.

static int	emit_triangle(ObjMesh *obj, int a, int b, int c)
{
	Vec3	edge_ab = vec3_sub(obj->positions[b], obj->positions[a]);
	Vec3	edge_ac = vec3_sub(obj->positions[c], obj->positions[a]);
	int		*grown;

	if (!(vec3_length(vec3_cross(edge_ab, edge_ac)) > obj->minArea))
		return (1);
	if (obj->triangleCount >= INT_MAX / 3 - 1)
		return (fprintf(stderr, "Too many triangles\n"), 0);
	grown = array_reserve(obj->corners, &obj->cornerCapacity, 3 * (obj->triangleCount + 1), sizeof(int));
	if (!grown)
		return (0);
	obj->corners = grown;
	obj->corners[3 * obj->triangleCount + 0] = a;
	obj->corners[3 * obj->triangleCount + 1] = b;
	obj->corners[3 * obj->triangleCount + 2] = c;
	obj->triangleCount++;
	return (1);
}

static int	fan(ObjMesh *obj, const ObjParser *parser, int count)
{
	const int	*corner = parser->polygon;
	const int	*left = parser->remaining;
	int			i;

	i = 1;
	while (i + 1 < count)
	{
		if (!emit_triangle(obj, corner[left[0]], corner[left[i]], corner[left[i + 1]]))
			return (0);
		i++;
	}
	return (1);
}

static Vec3	newell_normal(const ObjMesh *obj, const int *polygon, int count)
{
	Vec3	sum = vec3(0.0f, 0.0f, 0.0f);
	Vec3	current;
	Vec3	next;
	int		i;

	i = 0;
	while (i < count)
	{
		current = obj->positions[polygon[i]];
		next = obj->positions[polygon[(i + 1) % count]];
		sum.x += (current.y - next.y) * (current.z + next.z);
		sum.y += (current.z - next.z) * (current.x + next.x);
		sum.z += (current.x - next.x) * (current.y + next.y);
		i++;
	}
	return (sum);
}

static float	coordinate(Vec3 point, int axis)
{
	if (axis == 0)
		return (point.x);
	if (axis == 1)
		return (point.y);
	return (point.z);
}

// The normal's dominant axis is dropped; the other two swap when it points away, so outlines turn counter-clockwise.
static void	project_polygon(const ObjMesh *obj, ObjParser *parser, int count, Vec3 normal)
{
	int	dropped;
	int	axis_u;
	int	axis_v;
	int	i;

	dropped = 2;
	if (fabsf(normal.x) >= fabsf(normal.y) && fabsf(normal.x) >= fabsf(normal.z))
		dropped = 0;
	else if (fabsf(normal.y) >= fabsf(normal.z))
		dropped = 1;
	axis_u = (dropped + 1) % 3;
	axis_v = (dropped + 2) % 3;
	if (coordinate(normal, dropped) < 0.0f)
	{
		axis_u = axis_v;
		axis_v = (dropped + 1) % 3;
	}
	i = 0;
	while (i < count)
	{
		parser->projected[2 * i] = coordinate(obj->positions[parser->polygon[i]], axis_u);
		parser->projected[2 * i + 1] = coordinate(obj->positions[parser->polygon[i]], axis_v);
		i++;
	}
}

static float	turn(const float *a, const float *b, const float *c)
{
	return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]));
}

static int	same_point(const float *a, const float *b)
{
	return (a[0] == b[0] && a[1] == b[1]);
}

static int	is_ear(const ObjParser *parser, int count, int corner)
{
	const float	*previous = parser->projected + 2 * parser->remaining[(corner + count - 1) % count];
	const float	*current = parser->projected + 2 * parser->remaining[corner];
	const float	*next = parser->projected + 2 * parser->remaining[(corner + 1) % count];
	const float	*other;
	int			i;

	if (turn(previous, current, next) <= 0.0f)
		return (0);
	i = 0;
	while (i < count)
	{
		other = parser->projected + 2 * parser->remaining[i];
		if (!same_point(other, previous) && !same_point(other, current) && !same_point(other, next) && turn(previous, current, other) >= 0.0f && turn(current, next, other) >= 0.0f && turn(next, previous, other) >= 0.0f)
			return (0);
		i++;
	}
	return (1);
}

static int	ear_clip(ObjMesh *obj, ObjParser *parser, int count)
{
	int	*left = parser->remaining;
	int	corner;
	int	misses;

	corner = 0;
	misses = 0;
	while (count > 3 && misses < count)
	{
		misses++;
		if (is_ear(parser, count, corner))
		{
			if (!emit_triangle(obj, parser->polygon[left[(corner + count - 1) % count]], parser->polygon[left[corner]], parser->polygon[left[(corner + 1) % count]]))
				return (0);
			memmove(left + corner, left + corner + 1, (size_t)(count - corner - 1) * sizeof(int));
			count--;
			misses = 0;
		}
		corner = (corner + 1) % count;
	}
	return (fan(obj, parser, count));
}

int	obj_triangulate(ObjMesh *obj, ObjParser *parser, int corner_count)
{
	int		*remaining;
	float	*projected;
	Vec3	normal;
	int		i;

	if (corner_count == 3)
		return (emit_triangle(obj, parser->polygon[0], parser->polygon[1], parser->polygon[2]));
	remaining = array_reserve(parser->remaining, &parser->remainingCapacity, corner_count, sizeof(int));
	if (!remaining)
		return (0);
	parser->remaining = remaining;
	projected = array_reserve(parser->projected, &parser->projectedCapacity, 2 * corner_count, sizeof(float));
	if (!projected)
		return (0);
	parser->projected = projected;
	i = 0;
	while (i < corner_count)
	{
		parser->remaining[i] = i;
		i++;
	}
	normal = newell_normal(obj, parser->polygon, corner_count);
	if (!(vec3_length(normal) > obj->minArea))
		return (fan(obj, parser, corner_count));
	project_polygon(obj, parser, corner_count, normal);
	return (ear_clip(obj, parser, corner_count));
}
