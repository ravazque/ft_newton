#include "newton.h"

// A mesh against a sphere (closest point of each nearby triangle), a plane (its vertices) and a mesh (vertices and edges inside).

typedef struct SphereQuery
{
	const MeshShape		*mesh;
	Vec3				center;
	float				radius;
	ContactCandidates	set;
	MeshHit				closest;
}	SphereQuery;

static void	sphere_visit(void *context, int triangle)
{
	SphereQuery	*query = context;
	Vec3		corner[3];
	Vec3		point;
	Vec3		normal;
	float		distance;
	int			feature;

	mesh_triangle_corners(query->mesh, triangle, corner);
	point = closest_on_triangle(query->center, corner[0], corner[1], corner[2], &feature);   // [F29]
	distance = vec3_length(vec3_sub(query->center, point));
	if (distance >= query->radius)
		return ;
	normal = mesh_feature_normal(query->mesh, triangle, feature);
	if (distance < query->closest.distance)
	{
		query->closest.point = point;
		query->closest.normal = normal;
		query->closest.distance = distance;
		query->closest.inside = (vec3_dot(vec3_sub(query->center, point), normal) < 0.0f);   // [F30]
	}
	if (distance <= 1e-6f)
		manifold_add_candidate(&query->set, point, normal, query->radius);
	else if (vec3_dot(vec3_sub(query->center, point), normal) >= 0.0f)
		manifold_add_candidate(&query->set, point, vec3_scale(vec3_sub(query->center, point), 1.0f / distance), query->radius - distance);
}

static int	deepest_new(const ContactCandidates *set, const int *kept, int kept_count)
{
	int	best;
	int	i;
	int	k;

	best = -1;
	i = 0;
	while (i < set->count)
	{
		k = 0;
		while (k < kept_count && vec3_dot(set->contacts[i].normal, set->contacts[kept[k]].normal) < MESH_NORMAL_MERGE)
			k++;
		if (k == kept_count && (best < 0 || set->contacts[i].penetration > set->contacts[best].penetration))
			best = i;
		i++;
	}
	return (best);
}

static int	distinct_contacts(const SphereQuery *query, const RigidBody *mesh, Contact *out)
{
	int	kept[MAX_CONTACTS_PER_PAIR] = {0};
	int	count;
	int	best;

	count = 0;
	best = deepest_new(&query->set, kept, count);
	while (best >= 0 && count < MAX_CONTACTS_PER_PAIR)
	{
		kept[count] = best;
		out[count].point = mesh_to_world(mesh, query->set.contacts[best].point);
		out[count].normal = vec3_neg(quat_rotate(mesh->orientation, query->set.contacts[best].normal));
		out[count].penetration = query->set.contacts[best].penetration;
		count++;
		best = deepest_new(&query->set, kept, count);
	}
	return (count);
}

int	contact_sphere_mesh(const RigidBody *sphere, const RigidBody *mesh, Contact *out)
{
	SphereQuery	query;
	Vec3		reach;

	query.mesh = mesh->collider.mesh;
	query.center = mesh_to_local(mesh, sphere->position);
	query.radius = sphere->collider.radius;
	query.set.count = 0;
	query.closest.distance = INFINITY;
	reach = vec3(query.radius, query.radius, query.radius);
	bvh_query(query.mesh, vec3_sub(query.center, reach), vec3_add(query.center, reach), sphere_visit, &query);
	if (query.closest.distance == INFINITY)
		return (0);
	if (!query.closest.inside)
		return (distinct_contacts(&query, mesh, out));
	out->point = mesh_to_world(mesh, query.closest.point);
	out->normal = vec3_neg(quat_rotate(mesh->orientation, query.closest.normal));
	out->penetration = query.radius + query.closest.distance;
	return (1);
}

int	contact_mesh_plane(const RigidBody *mesh, const RigidBody *plane, Contact *out)
{
	const MeshShape		*shape = mesh->collider.mesh;
	ContactCandidates	set;
	Vec3				point;
	float				distance;
	int					i;

	set.count = 0;
	i = 0;
	while (i < shape->vertexCount)
	{
		point = mesh_to_world(mesh, shape->vertices[i]);
		distance = collider_plane_distance(&plane->collider, point);   // [F13] per vertex
		if (distance < 0.0f && collider_plane_covers(&plane->collider, point))
			manifold_add_candidate(&set, point, vec3_neg(plane->collider.normal), -distance);
		i++;
	}
	return (manifold_reduce_candidates(&set, out));
}

static void	vertices_inside(const RigidBody *from, const RigidBody *into, float sign, ContactCandidates *set)
{
	const MeshShape	*shape = from->collider.mesh;
	MeshHit			hit;
	Vec3			bounds_min;
	Vec3			bounds_max;
	Vec3			point;
	int				i;

	collider_bounds(into, &bounds_min, &bounds_max);
	i = 0;
	while (i < shape->vertexCount)
	{
		point = mesh_to_world(from, shape->vertices[i]);
		if (point.x >= bounds_min.x && point.x <= bounds_max.x && point.y >= bounds_min.y && point.y <= bounds_max.y && point.z >= bounds_min.z && point.z <= bounds_max.z && mesh_closest_point(into->collider.mesh, mesh_to_local(into, point), MESH_MAX_DEPTH, &hit) && hit.inside)
			manifold_add_candidate(set, point, vec3_scale(quat_rotate(into->orientation, hit.normal), sign), hit.distance);   // [F30]
		i++;
	}
}

typedef struct EdgeQuery
{
	const MeshShape	*mesh;
	Vec3			start;
	Vec3			end;
	float			crossings[MESH_EDGE_CROSSINGS];
	int				count;
}	EdgeQuery;

static void	edge_visit(void *context, int triangle)
{
	EdgeQuery	*query = context;
	Vec3		corner[3];
	float		fraction;

	mesh_triangle_corners(query->mesh, triangle, corner);
	if (query->count < MESH_EDGE_CROSSINGS && segment_crosses_triangle(query->start, query->end, corner, &fraction))   // [F37]
	{
		query->crossings[query->count] = fraction;
		query->count++;
	}
}

static void	sort_crossings(EdgeQuery *query)
{
	float	moved;
	int		i;
	int		k;

	i = 1;
	while (i < query->count)
	{
		moved = query->crossings[i];
		k = i - 1;
		while (k >= 0 && query->crossings[k] > moved)
		{
			query->crossings[k + 1] = query->crossings[k];
			k--;
		}
		query->crossings[k + 1] = moved;
		i++;
	}
}

// Crossings alternate entering and leaving; the middle of each part of the edge inside the other mesh is a contact.
static void	edge_contacts(const RigidBody *into, const EdgeQuery *query, float sign, ContactCandidates *set)
{
	MeshHit	hit;
	Vec3	middle;
	int		k;

	if (query->count < 2 || query->count % 2 != 0)
		return ;
	k = 0;
	while (k + 1 < query->count)
	{
		middle = vec3_add(query->start, vec3_scale(vec3_sub(query->end, query->start), (query->crossings[k] + query->crossings[k + 1]) * 0.5f));
		if (mesh_closest_point(query->mesh, middle, MESH_MAX_DEPTH, &hit) && hit.inside)
			manifold_add_candidate(set, mesh_to_world(into, middle), vec3_scale(quat_rotate(into->orientation, hit.normal), sign), hit.distance);   // [F30]
		k += 2;
	}
}

static void	edges_inside(const RigidBody *from, const RigidBody *into, float sign, ContactCandidates *set)
{
	const MeshShape	*shape = from->collider.mesh;
	Quat			turn = quat_mul(quat_conjugate(into->orientation), from->orientation);
	Vec3			offset = mesh_to_local(into, from->position);
	EdgeQuery		query;
	int				edge;

	query.mesh = into->collider.mesh;
	edge = 0;
	while (edge < shape->edgeCount)
	{
		query.start = vec3_add(offset, quat_rotate(turn, shape->vertices[shape->edges[edge * 2]]));
		query.end = vec3_add(offset, quat_rotate(turn, shape->vertices[shape->edges[edge * 2 + 1]]));
		query.count = 0;
		bvh_query(query.mesh, vec3_min(query.start, query.end), vec3_max(query.start, query.end), edge_visit, &query);
		sort_crossings(&query);
		edge_contacts(into, &query, sign, set);
		edge++;
	}
}

int	contact_mesh_mesh(const RigidBody *mesh_a, const RigidBody *mesh_b, Contact *out)
{
	ContactCandidates	set;

	set.count = 0;
	vertices_inside(mesh_a, mesh_b, -1.0f, &set);
	vertices_inside(mesh_b, mesh_a, 1.0f, &set);
	edges_inside(mesh_a, mesh_b, -1.0f, &set);
	edges_inside(mesh_b, mesh_a, 1.0f, &set);
	return (manifold_reduce_candidates(&set, out));
}
