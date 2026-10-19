#include "newton.h"

// Bounding volume tree of a mesh [F28]: nodes split at the median triangle centre along their longest side, so it stays balanced.

static float	coordinate(const Vec3 *centroid, int triangle, int axis)
{
	if (axis == 0)
		return (centroid[triangle].x);
	if (axis == 1)
		return (centroid[triangle].y);
	return (centroid[triangle].z);
}

// Quickselect: reorders the run so its 'middle' first triangles have the lowest centres along the axis.
static void	select_median(int *run, int count, int middle, const Vec3 *centroid, int axis)
{
	float	pivot;
	int		low = 0;
	int		high = count - 1;
	int		left;
	int		right;
	int		swap;

	while (low < high)
	{
		pivot = coordinate(centroid, run[(low + high) / 2], axis);
		left = low;
		right = high;
		while (left <= right)
		{
			while (coordinate(centroid, run[left], axis) < pivot)
				left++;
			while (coordinate(centroid, run[right], axis) > pivot)
				right--;
			if (left <= right)
			{
				swap = run[left];
				run[left++] = run[right];
				run[right--] = swap;
			}
		}
		if (middle <= right)
			high = right;
		else if (middle >= left)
			low = left;
		else
			return ;
	}
}

static int	longest_axis(Vec3 extent)
{
	if (extent.x >= extent.y && extent.x >= extent.z)
		return (0);
	if (extent.y >= extent.z)
		return (1);
	return (2);
}

static int	build_node(MeshShape *mesh, const Vec3 *centroid, int first, int count)
{
	int		index = mesh->nodeCount++;
	BvhNode	*node = &mesh->nodes[index];
	Vec3	centre_min = centroid[mesh->triangleOrder[first]];
	Vec3	centre_max = centre_min;
	int		corner;
	int		i;

	node->boundsMin = mesh->vertices[mesh->corners[mesh->triangleOrder[first] * 3]];
	node->boundsMax = node->boundsMin;
	i = 0;
	while (i < count * 3)
	{
		corner = mesh->corners[mesh->triangleOrder[first + i / 3] * 3 + i % 3];
		node->boundsMin = vec3_min(node->boundsMin, mesh->vertices[corner]);
		node->boundsMax = vec3_max(node->boundsMax, mesh->vertices[corner]);
		centre_min = vec3_min(centre_min, centroid[mesh->triangleOrder[first + i / 3]]);
		centre_max = vec3_max(centre_max, centroid[mesh->triangleOrder[first + i / 3]]);
		i++;
	}
	node->left = -1;
	node->right = -1;
	node->firstIndex = first;
	node->triangleCount = count;
	if (count <= BVH_LEAF_TRIANGLES)
		return (index);
	select_median(mesh->triangleOrder + first, count, count / 2, centroid, longest_axis(vec3_sub(centre_max, centre_min)));
	node->left = build_node(mesh, centroid, first, count / 2);
	node->right = build_node(mesh, centroid, first + count / 2, count - count / 2);
	return (index);
}

int	bvh_build(MeshShape *mesh)
{
	Vec3	*centroid = malloc((size_t)mesh->triangleCount * sizeof(Vec3));
	Vec3	corner[3];
	int		triangle;

	mesh->triangleOrder = malloc((size_t)mesh->triangleCount * sizeof(int));
	mesh->nodes = malloc((size_t)mesh->triangleCount * 2 * sizeof(BvhNode));
	if (!centroid || !mesh->triangleOrder || !mesh->nodes)
		return (free(centroid), fprintf(stderr, "Out of memory\n"), 0);
	triangle = 0;
	while (triangle < mesh->triangleCount)
	{
		mesh->triangleOrder[triangle] = triangle;
		mesh_triangle_corners(mesh, triangle, corner);
		centroid[triangle] = vec3_scale(vec3_add(vec3_add(corner[0], corner[1]), corner[2]), 1.0f / 3.0f);
		triangle++;
	}
	mesh->nodeCount = 0;
	build_node(mesh, centroid, 0, mesh->triangleCount);
	free(centroid);
	return (1);
}

static int	boxes_meet(const BvhNode *node, Vec3 bounds_min, Vec3 bounds_max)
{
	return (node->boundsMin.x <= bounds_max.x && bounds_min.x <= node->boundsMax.x && node->boundsMin.y <= bounds_max.y && bounds_min.y <= node->boundsMax.y && node->boundsMin.z <= bounds_max.z && bounds_min.z <= node->boundsMax.z);   // [F11]
}

void	bvh_query(const MeshShape *mesh, Vec3 bounds_min, Vec3 bounds_max, void (*visit)(void *context, int triangle), void *context)
{
	const BvhNode	*node;
	int				stack[BVH_STACK];
	int				top;
	int				i;

	top = 0;
	if (mesh->nodeCount > 0)
		stack[top++] = 0;
	while (top > 0)
	{
		node = &mesh->nodes[stack[--top]];
		if (!boxes_meet(node, bounds_min, bounds_max))
			continue ;
		i = 0;
		while (node->left < 0 && i < node->triangleCount)
			visit(context, mesh->triangleOrder[node->firstIndex + i++]);   // [F28]
		if (node->left >= 0 && top + 2 <= BVH_STACK)
		{
			stack[top++] = node->left;
			stack[top++] = node->right;
		}
	}
}
