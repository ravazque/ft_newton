#include "newton.h"

// The surface point of a mesh closest to a point, and whether that point is inside, by the closest feature's pseudo-normal [F30].

typedef struct ClosestQuery
{
	const MeshShape	*mesh;
	Vec3			point;
	MeshHit			*hit;
	int				found;
}	ClosestQuery;

void	mesh_triangle_corners(const MeshShape *mesh, int triangle, Vec3 *corner)
{
	corner[0] = mesh->vertices[mesh->corners[triangle * 3]];
	corner[1] = mesh->vertices[mesh->corners[triangle * 3 + 1]];
	corner[2] = mesh->vertices[mesh->corners[triangle * 3 + 2]];
}

Vec3	mesh_feature_normal(const MeshShape *mesh, int triangle, int feature)
{
	if (feature == 0)
		return (mesh->faceNormals[triangle]);
	if (feature <= 3)
		return (mesh->vertexNormals[mesh->corners[triangle * 3 + feature - 1]]);
	return (vec3_normalized(vec3_add(mesh->faceNormals[triangle], mesh->faceNormals[mesh->neighbours[triangle * 3 + feature - 4]])));   // [F30]
}

static void	closest_visit(void *context, int triangle)
{
	ClosestQuery	*query = context;
	Vec3			corner[3];
	Vec3			point;
	float			distance;
	int				feature;

	mesh_triangle_corners(query->mesh, triangle, corner);
	point = closest_on_triangle(query->point, corner[0], corner[1], corner[2], &feature);
	distance = vec3_length(vec3_sub(query->point, point));
	if (distance >= query->hit->distance)
		return ;
	query->hit->point = point;
	query->hit->distance = distance;
	query->hit->normal = mesh_feature_normal(query->mesh, triangle, feature);
	query->found = 1;
}

int	mesh_closest_point(const MeshShape *mesh, Vec3 point, float radius, MeshHit *hit)
{
	ClosestQuery	query;
	Vec3			reach = vec3(radius, radius, radius);

	query.mesh = mesh;
	query.point = point;
	query.hit = hit;
	query.found = 0;
	hit->distance = radius;
	bvh_query(mesh, vec3_sub(point, reach), vec3_add(point, reach), closest_visit, &query);
	if (!query.found)
		return (0);
	hit->inside = (vec3_dot(vec3_sub(point, hit->point), hit->normal) < 0.0f);   // [F30]
	return (1);
}
