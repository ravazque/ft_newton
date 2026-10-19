#include "newton.h"

// Any triangle soup made a closed surface wound outward: vertices welded, faces turned to agree, holes closed by fans.

typedef struct WeldKey
{
	long long	x;
	long long	y;
	long long	z;
	int			vertex;
}	WeldKey;

typedef struct TriangleKey
{
	int	sorted[3];
	int	triangle;
}	TriangleKey;

typedef struct EdgeUse
{
	int	low;
	int	high;
	int	triangle;
	int	edge;
}	EdgeUse;

typedef struct BoundaryEdge
{
	int	from;
	int	to;
	int	used;
}	BoundaryEdge;

typedef struct RepairReport
{
	int	welded;
	int	duplicates;
	int	turned;
	int	holes;
	int	outward;
}	RepairReport;

static int	compare_long(long long a, long long b)
{
	if (a < b)
		return (-1);
	if (a > b)
		return (1);
	return (0);
}

static int	compare_keys(const void *first, const void *second)
{
	const WeldKey	*a = first;
	const WeldKey	*b = second;

	if (a->x != b->x)
		return (compare_long(a->x, b->x));
	if (a->y != b->y)
		return (compare_long(a->y, b->y));
	return (compare_long(a->z, b->z));
}

static int	weld(MeshShape *mesh, RepairReport *report)
{
	WeldKey	*keys = malloc((size_t)mesh->vertexCount * sizeof(WeldKey));
	int		*remap = malloc((size_t)mesh->vertexCount * sizeof(int));
	Vec3	*kept = malloc((size_t)mesh->vertexCount * sizeof(Vec3));
	float	cell = MESH_WELD_EPSILON * mesh->size;
	int		count;
	int		i;

	if (!keys || !remap || !kept)
		return (free(keys), free(remap), free(kept), fprintf(stderr, "Out of memory\n"), 0);
	i = 0;
	while (i < mesh->vertexCount)
	{
		keys[i] = (WeldKey){llroundf(mesh->vertices[i].x / cell), llroundf(mesh->vertices[i].y / cell), llroundf(mesh->vertices[i].z / cell), i};
		i++;
	}
	qsort(keys, (size_t)mesh->vertexCount, sizeof(WeldKey), compare_keys);
	count = 0;
	i = 0;
	while (i < mesh->vertexCount)
	{
		if (i == 0 || compare_keys(&keys[i], &keys[i - 1]) != 0)
			kept[count++] = mesh->vertices[keys[i].vertex];
		remap[keys[i].vertex] = count - 1;
		i++;
	}
	i = 0;
	while (i < mesh->triangleCount * 3)
	{
		mesh->corners[i] = remap[mesh->corners[i]];
		i++;
	}
	report->welded = mesh->vertexCount - count;
	free(mesh->vertices);
	mesh->vertices = kept;
	mesh->vertexCount = count;
	return (free(keys), free(remap), 1);
}

static void	sort_three(const int *corner, int *sorted)
{
	int	swap;

	memcpy(sorted, corner, 3 * sizeof(int));
	if (sorted[0] > sorted[1])
	{
		swap = sorted[0];
		sorted[0] = sorted[1];
		sorted[1] = swap;
	}
	if (sorted[1] > sorted[2])
	{
		swap = sorted[1];
		sorted[1] = sorted[2];
		sorted[2] = swap;
	}
	if (sorted[0] > sorted[1])
	{
		swap = sorted[0];
		sorted[0] = sorted[1];
		sorted[1] = swap;
	}
}

static int	compare_triangles(const void *first, const void *second)
{
	const TriangleKey	*a = first;
	const TriangleKey	*b = second;
	int					k;

	k = 0;
	while (k < 3)
	{
		if (a->sorted[k] != b->sorted[k])
			return (compare_long(a->sorted[k], b->sorted[k]));
		k++;
	}
	return (compare_long(a->triangle, b->triangle));
}

static int	same_corners(const TriangleKey *a, const TriangleKey *b)
{
	return (a->sorted[0] == b->sorted[0] && a->sorted[1] == b->sorted[1] && a->sorted[2] == b->sorted[2]);
}

static int	drop_bad_faces(MeshShape *mesh, RepairReport *report)
{
	TriangleKey	*keys = malloc((size_t)mesh->triangleCount * sizeof(TriangleKey));
	int			kept;
	int			t;

	if (!keys)
		return (fprintf(stderr, "Out of memory\n"), 0);
	t = 0;
	while (t < mesh->triangleCount)
	{
		sort_three(&mesh->corners[t * 3], keys[t].sorted);
		keys[t].triangle = t;
		t++;
	}
	qsort(keys, (size_t)mesh->triangleCount, sizeof(TriangleKey), compare_triangles);
	t = 0;
	while (t < mesh->triangleCount)
	{
		if (keys[t].sorted[0] == keys[t].sorted[1] || keys[t].sorted[1] == keys[t].sorted[2])
			mesh->corners[keys[t].triangle * 3] = -1;
		else if (t > 0 && same_corners(&keys[t], &keys[t - 1]))
			mesh->corners[keys[t].triangle * 3] = -1;
		t++;
	}
	free(keys);
	kept = 0;
	t = 0;
	while (t < mesh->triangleCount)
	{
		if (mesh->corners[t * 3] >= 0)
		{
			memmove(&mesh->corners[kept * 3], &mesh->corners[t * 3], 3 * sizeof(int));
			kept++;
		}
		t++;
	}
	report->duplicates = mesh->triangleCount - kept;
	mesh->triangleCount = kept;
	return (1);
}

static int	next_corner(int corner)
{
	return (corner - corner % 3 + (corner % 3 + 1) % 3);
}

static int	compare_edges(const void *first, const void *second)
{
	const EdgeUse	*a = first;
	const EdgeUse	*b = second;

	if (a->low != b->low)
		return (compare_long(a->low, b->low));
	return (compare_long(a->high, b->high));
}

static void	collect_edges(const MeshShape *mesh, EdgeUse *edges, int *neighbours)
{
	int	i;

	i = 0;
	while (i < mesh->triangleCount * 3)
	{
		edges[i] = (EdgeUse){mesh->corners[i], mesh->corners[next_corner(i)], i / 3, i % 3};
		if (edges[i].high < edges[i].low)
			edges[i] = (EdgeUse){mesh->corners[next_corner(i)], mesh->corners[i], i / 3, i % 3};
		neighbours[i] = -1;
		i++;
	}
	qsort(edges, (size_t)mesh->triangleCount * 3, sizeof(EdgeUse), compare_edges);
}

static int	find_neighbours(const MeshShape *mesh, int *neighbours, const char *path)
{
	EdgeUse	*edges = malloc((size_t)mesh->triangleCount * 3 * sizeof(EdgeUse));
	int		crowded;
	int		run;
	int		i;

	if (!edges)
		return (fprintf(stderr, "Out of memory\n"), 0);
	collect_edges(mesh, edges, neighbours);
	crowded = 0;
	i = 0;
	while (i < mesh->triangleCount * 3)
	{
		run = 1;
		while (i + run < mesh->triangleCount * 3 && compare_edges(&edges[i], &edges[i + run]) == 0)
			run++;
		if (run > 2)
			crowded++;
		if (run == 2)
		{
			neighbours[edges[i].triangle * 3 + edges[i].edge] = edges[i + 1].triangle;
			neighbours[edges[i + 1].triangle * 3 + edges[i + 1].edge] = edges[i].triangle;
		}
		i += run;
	}
	free(edges);
	if (crowded > 0)
		return (fprintf(stderr, "%s: %d edges are shared by more than two faces\n", path, crowded), 0);
	return (1);
}

static int	runs_from_to(const int *corners, int triangle, int from, int to)
{
	int	k;

	k = 0;
	while (k < 3)
	{
		if (corners[triangle * 3 + k] == from && corners[triangle * 3 + (k + 1) % 3] == to)
			return (1);
		k++;
	}
	return (0);
}

static void	flip_triangle(int *corners, int *neighbours, int triangle)
{
	int	swap;

	swap = corners[triangle * 3 + 1];
	corners[triangle * 3 + 1] = corners[triangle * 3 + 2];
	corners[triangle * 3 + 2] = swap;
	swap = neighbours[triangle * 3];
	neighbours[triangle * 3] = neighbours[triangle * 3 + 2];
	neighbours[triangle * 3 + 2] = swap;
}

static int	visit_neighbour(MeshShape *mesh, int *neighbours, int *component, int triangle, int edge)
{
	int	other = neighbours[triangle * 3 + edge];
	int	from = mesh->corners[triangle * 3 + edge];
	int	to = mesh->corners[triangle * 3 + (edge + 1) % 3];

	if (other < 0)
		return (0);
	if (component[other] >= 0 && runs_from_to(mesh->corners, other, from, to))
		return (-1);
	if (component[other] >= 0)
		return (0);
	component[other] = component[triangle];
	if (!runs_from_to(mesh->corners, other, from, to))
		return (1);
	flip_triangle(mesh->corners, neighbours, other);
	return (2);
}

static int	keep_majority_winding(MeshShape *mesh, int *neighbours, const int *piece, int piece_size, int turned)
{
	int	i;

	if (turned * 2 <= piece_size)
		return (turned);
	i = 0;
	while (i < piece_size)
	{
		flip_triangle(mesh->corners, neighbours, piece[i]);
		i++;
	}
	return (piece_size - turned);
}

static int	orient_component(MeshShape *mesh, int *neighbours, int *component, int *queue, int seed)
{
	int	turned;
	int	head;
	int	tail;
	int	edge;
	int	visit;

	turned = 0;
	head = 0;
	tail = 0;
	queue[tail++] = seed;
	while (head < tail)
	{
		edge = 0;
		while (edge < 3)
		{
			visit = visit_neighbour(mesh, neighbours, component, queue[head], edge);
			if (visit < 0)
				return (-1);
			if (visit > 0)
				queue[tail++] = neighbours[queue[head] * 3 + edge];
			if (visit == 2)
				turned++;
			edge++;
		}
		head++;
	}
	return (keep_majority_winding(mesh, neighbours, queue, tail, turned));
}

static int	orient(MeshShape *mesh, int *neighbours, int *component, const char *path, int *turned)
{
	int	*queue = malloc((size_t)mesh->triangleCount * sizeof(int));
	int	components;
	int	result;
	int	t;

	if (!queue)
		return (fprintf(stderr, "Out of memory\n"), -1);
	memset(component, -1, (size_t)mesh->triangleCount * sizeof(int));
	components = 0;
	*turned = 0;
	t = 0;
	while (t < mesh->triangleCount)
	{
		if (component[t] < 0)
		{
			component[t] = components++;
			result = orient_component(mesh, neighbours, component, queue, t);
			if (result < 0)
				return (free(queue), fprintf(stderr, "%s: the surface cannot be oriented (its two sides meet)\n", path), -1);
			*turned += result;
		}
		t++;
	}
	free(queue);
	return (components);
}

static int	compare_boundary(const void *first, const void *second)
{
	const BoundaryEdge	*a = first;
	const BoundaryEdge	*b = second;

	return (compare_long(a->from, b->from));
}

static int	find_boundary(const BoundaryEdge *boundary, int count, int from)
{
	int	low;
	int	high;
	int	middle;

	low = 0;
	high = count - 1;
	while (low <= high)
	{
		middle = (low + high) / 2;
		if (boundary[middle].from == from)
			return (middle);
		if (boundary[middle].from < from)
			low = middle + 1;
		else
			high = middle - 1;
	}
	return (-1);
}

// One hole: a fan from its centroid, each new triangle running the rim edge the other way round.
static void	close_hole(MeshShape *mesh, BoundaryEdge *boundary, int count, int start)
{
	int		center = mesh->vertexCount;
	Vec3	sum = vec3(0.0f, 0.0f, 0.0f);
	int		length;
	int		edge;
	int		*corner;

	length = 0;
	edge = start;
	while (edge >= 0 && !boundary[edge].used)
	{
		boundary[edge].used = 1;
		sum = vec3_add(sum, mesh->vertices[boundary[edge].from]);
		corner = &mesh->corners[mesh->triangleCount * 3];
		corner[0] = boundary[edge].to;
		corner[1] = boundary[edge].from;
		corner[2] = center;
		mesh->triangleCount++;
		length++;
		edge = find_boundary(boundary, count, boundary[edge].to);
	}
	mesh->vertices[center] = vec3_scale(sum, 1.0f / (float)length);
	mesh->vertexCount++;
}

static int	reserve_caps(MeshShape *mesh, int rim_edges)
{
	Vec3	*vertices = realloc(mesh->vertices, (size_t)(mesh->vertexCount + rim_edges) * sizeof(Vec3));
	int		*corners;

	if (!vertices)
		return (0);
	mesh->vertices = vertices;
	corners = realloc(mesh->corners, (size_t)(mesh->triangleCount + rim_edges) * 3 * sizeof(int));
	if (!corners)
		return (0);
	mesh->corners = corners;
	return (1);
}

static int	close_holes(MeshShape *mesh, const int *neighbours, const char *path, RepairReport *report)
{
	BoundaryEdge	*boundary = malloc((size_t)(mesh->triangleCount * 3 + 1) * sizeof(BoundaryEdge));
	int				count;
	int				i;

	if (!boundary)
		return (fprintf(stderr, "Out of memory\n"), 0);
	count = 0;
	i = 0;
	while (i < mesh->triangleCount * 3)
	{
		if (neighbours[i] < 0)
			boundary[count++] = (BoundaryEdge){mesh->corners[i], mesh->corners[next_corner(i)], 0};
		i++;
	}
	qsort(boundary, (size_t)count, sizeof(BoundaryEdge), compare_boundary);
	i = 1;
	while (i < count && boundary[i].from != boundary[i - 1].from)
		i++;
	if (i < count)
		return (free(boundary), fprintf(stderr, "%s: a hole touches itself at a vertex and cannot be closed\n", path), 0);
	if (count > 0 && !reserve_caps(mesh, count))
		return (free(boundary), fprintf(stderr, "Out of memory\n"), 0);
	i = 0;
	while (i < count)
	{
		if (!boundary[i].used)
		{
			close_hole(mesh, boundary, count, i);
			report->holes++;
		}
		i++;
	}
	free(boundary);
	return (1);
}

static void	turn_outward(MeshShape *mesh, const int *component, int components, float *volumes, RepairReport *report)
{
	Vec3	corner[3];
	int		t;

	memset(volumes, 0, (size_t)components * sizeof(float));
	t = 0;
	while (t < mesh->triangleCount)
	{
		mesh_triangle_corners(mesh, t, corner);
		volumes[component[t]] += vec3_dot(corner[0], vec3_cross(corner[1], corner[2]));
		t++;
	}
	t = 0;
	while (t < components)
	{
		if (volumes[t] < 0.0f)
			report->outward++;
		t++;
	}
	t = 0;
	while (t < mesh->triangleCount)
	{
		if (volumes[component[t]] < 0.0f)
			flip_triangle(mesh->corners, mesh->neighbours, t);
		t++;
	}
}

static void	print_report(const char *path, const RepairReport *report)
{
	if (report->welded + report->duplicates + report->turned + report->holes + report->outward == 0)
		return ;
	printf("%s: repaired (%d vertices welded, %d faces dropped, %d faces turned, %d holes closed, %d pieces turned outward)\n", path, report->welded, report->duplicates, report->turned, report->holes, report->outward);
}

static int	orient_and_close(MeshShape *mesh, const char *path, RepairReport *report)
{
	int	*neighbours = malloc((size_t)mesh->triangleCount * 3 * sizeof(int));
	int	*component = malloc((size_t)mesh->triangleCount * sizeof(int));
	int	ok;

	ok = (neighbours && component);
	if (!ok)
		fprintf(stderr, "Out of memory\n");
	ok = ok && find_neighbours(mesh, neighbours, path);
	ok = ok && orient(mesh, neighbours, component, path, &report->turned) >= 0;
	ok = ok && close_holes(mesh, neighbours, path, report);
	free(neighbours);
	free(component);
	return (ok);
}

int	mesh_repair(MeshShape *mesh, const char *path)
{
	RepairReport	report;
	int				*component;
	float			*volumes;
	int				components;
	int				unused;

	memset(&report, 0, sizeof(report));
	if (!weld(mesh, &report) || !drop_bad_faces(mesh, &report))
		return (0);
	if (mesh->triangleCount == 0)
		return (fprintf(stderr, "%s: no face is left once repeated corners are dropped\n", path), 0);
	if (!orient_and_close(mesh, path, &report) || !mesh_topology_build(mesh, path))
		return (0);
	component = malloc((size_t)mesh->triangleCount * sizeof(int));
	volumes = malloc((size_t)mesh->triangleCount * sizeof(float));
	components = -1;
	if (component && volumes)
		components = orient(mesh, mesh->neighbours, component, path, &unused);
	if (components > 0)
		turn_outward(mesh, component, components, volumes, &report);
	free(component);
	free(volumes);
	if (components <= 0)
		return (0);
	print_report(path, &report);
	return (1);
}
