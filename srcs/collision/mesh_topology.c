#include "newton.h"

// Every edge shared by two faces running it opposite ways gives each triangle its neighbours; the normals feed [F30].

typedef struct EdgeEntry
{
	int	low;
	int	high;
	int	triangle;
	int	edge;
	int	forward;
}	EdgeEntry;

static int	compare_edges(const void *first, const void *second)
{
	const EdgeEntry	*a = first;
	const EdgeEntry	*b = second;

	if (a->low != b->low)
		return ((a->low > b->low) - (a->low < b->low));
	return ((a->high > b->high) - (a->high < b->high));
}

static EdgeEntry	*collect_edges(const MeshShape *mesh)
{
	EdgeEntry	*edges = malloc((size_t)mesh->triangleCount * 3 * sizeof(EdgeEntry));
	int			from;
	int			to;
	int			i;

	if (!edges)
		return (NULL);
	i = 0;
	while (i < mesh->triangleCount * 3)
	{
		from = mesh->corners[i];
		to = mesh->corners[i - i % 3 + (i % 3 + 1) % 3];
		edges[i].low = from;
		edges[i].high = to;
		if (to < from)
		{
			edges[i].low = to;
			edges[i].high = from;
		}
		edges[i].triangle = i / 3;
		edges[i].edge = i % 3;
		edges[i].forward = (from == edges[i].low);
		i++;
	}
	qsort(edges, (size_t)mesh->triangleCount * 3, sizeof(EdgeEntry), compare_edges);
	return (edges);
}

static void	pair_edges(MeshShape *mesh, const EdgeEntry *edges, int *open, int *same_way)
{
	int	total = mesh->triangleCount * 3;
	int	run;
	int	i;

	i = 0;
	while (i < total)
	{
		run = 1;
		while (i + run < total && edges[i + run].low == edges[i].low && edges[i + run].high == edges[i].high)
			run++;
		if (run != 2)
			*open += run;
		else if (edges[i].forward == edges[i + 1].forward)
			*same_way += 1;
		else
		{
			mesh->neighbours[edges[i].triangle * 3 + edges[i].edge] = edges[i + 1].triangle;
			mesh->neighbours[edges[i + 1].triangle * 3 + edges[i + 1].edge] = edges[i].triangle;
			mesh->edges[mesh->edgeCount * 2] = edges[i].low;
			mesh->edges[mesh->edgeCount * 2 + 1] = edges[i].high;
			mesh->edgeCount++;
		}
		i += run;
	}
}

int	mesh_topology_build(MeshShape *mesh, const char *path)
{
	EdgeEntry	*edges;
	int			open;
	int			same_way;

	mesh->neighbours = malloc((size_t)mesh->triangleCount * 3 * sizeof(int));
	mesh->edges = malloc((size_t)mesh->triangleCount * 3 * sizeof(int));
	mesh->edgeCount = 0;
	edges = collect_edges(mesh);
	if (!edges || !mesh->neighbours || !mesh->edges)
		return (free(edges), fprintf(stderr, "Out of memory\n"), 0);
	open = 0;
	same_way = 0;
	pair_edges(mesh, edges, &open, &same_way);
	free(edges);
	if (open > 0)
		return (fprintf(stderr, "%s: not a closed surface (%d edges are not shared by exactly two faces)\n", path, open), 0);
	if (same_way > 0)
		return (fprintf(stderr, "%s: faces are not wound consistently (%d edges run the same way in both faces)\n", path, same_way), 0);
	return (1);
}

// Angle of the triangle at one corner: the weight of that face in the vertex's pseudo-normal.
static float	corner_angle(const MeshShape *mesh, int triangle, int corner)
{
	Vec3	apex = mesh->vertices[mesh->corners[triangle * 3 + corner]];
	Vec3	side_a = vec3_normalized(vec3_sub(mesh->vertices[mesh->corners[triangle * 3 + (corner + 1) % 3]], apex));
	Vec3	side_b = vec3_normalized(vec3_sub(mesh->vertices[mesh->corners[triangle * 3 + (corner + 2) % 3]], apex));

	return (acosf(clampf(vec3_dot(side_a, side_b), -1.0f, 1.0f)));
}

int	mesh_normals_build(MeshShape *mesh)
{
	Vec3	corner[3];
	Vec3	*vertex_normal;
	int		triangle;
	int		vertex;
	int		k;

	mesh->faceNormals = malloc((size_t)mesh->triangleCount * sizeof(Vec3));
	mesh->vertexNormals = calloc((size_t)mesh->vertexCount, sizeof(Vec3));
	if (!mesh->faceNormals || !mesh->vertexNormals)
		return (fprintf(stderr, "Out of memory\n"), 0);
	triangle = 0;
	while (triangle < mesh->triangleCount)
	{
		mesh_triangle_corners(mesh, triangle, corner);
		mesh->faceNormals[triangle] = vec3_normalized(vec3_cross(vec3_sub(corner[1], corner[0]), vec3_sub(corner[2], corner[0])));
		k = 0;
		while (k < 3)
		{
			vertex_normal = &mesh->vertexNormals[mesh->corners[triangle * 3 + k]];
			*vertex_normal = vec3_add(*vertex_normal, vec3_scale(mesh->faceNormals[triangle], corner_angle(mesh, triangle, k)));   // [F30]
			k++;
		}
		triangle++;
	}
	vertex = 0;
	while (vertex < mesh->vertexCount)
	{
		mesh->vertexNormals[vertex] = vec3_normalized(mesh->vertexNormals[vertex]);
		vertex++;
	}
	return (1);
}
