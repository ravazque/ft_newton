#include "newton.h"

// A model turned into a collider: scaled to its size, repaired, centred on its centre of mass [F27] and indexed by a tree.

// Only the vertices a face uses are kept: a stray one would act as a contact point.
static int	take_geometry(MeshShape *mesh, const ObjMesh *obj, float size)
{
	int		*remap = malloc((size_t)obj->positionCount * sizeof(int));
	Vec3	low;
	Vec3	high;
	Vec3	extent;
	int		i;

	mesh->vertices = malloc((size_t)obj->positionCount * sizeof(Vec3));
	mesh->corners = malloc((size_t)obj->triangleCount * 3 * sizeof(int));
	if (!remap || !mesh->vertices || !mesh->corners)
		return (free(remap), fprintf(stderr, "Out of memory\n"), 0);
	memset(remap, -1, (size_t)obj->positionCount * sizeof(int));
	i = 0;
	while (i < obj->triangleCount * 3)
	{
		if (remap[obj->corners[i]] < 0)
		{
			remap[obj->corners[i]] = mesh->vertexCount;
			mesh->vertices[mesh->vertexCount++] = obj->positions[obj->corners[i]];
		}
		mesh->corners[i] = remap[obj->corners[i]];
		i++;
	}
	mesh->triangleCount = obj->triangleCount;
	free(remap);
	low = mesh->vertices[0];
	high = mesh->vertices[0];
	i = 0;
	while (i < mesh->vertexCount)
	{
		low = vec3_min(low, mesh->vertices[i]);
		high = vec3_max(high, mesh->vertices[i]);
		i++;
	}
	extent = vec3_sub(high, low);
	i = 0;
	while (i < mesh->vertexCount)
	{
		mesh->vertices[i] = vec3_scale(mesh->vertices[i], size / fmaxf(extent.x, fmaxf(extent.y, extent.z)));
		i++;
	}
	return (1);
}

static void	recentre(MeshShape *mesh, Vec3 center)
{
	int	i;

	mesh->boundsMin = vec3_sub(mesh->vertices[0], center);
	mesh->boundsMax = mesh->boundsMin;
	i = 0;
	while (i < mesh->vertexCount)
	{
		mesh->vertices[i] = vec3_sub(mesh->vertices[i], center);
		mesh->boundsMin = vec3_min(mesh->boundsMin, mesh->vertices[i]);
		mesh->boundsMax = vec3_max(mesh->boundsMax, mesh->vertices[i]);
		i++;
	}
}

static int	mass_properties(MeshShape *mesh, const char *path)
{
	Vec3	center;
	Mat3	inertia;
	float	volume;

	volume = mesh_mass_properties(mesh, &center, &inertia);
	if (!(volume > MESH_MIN_VOLUME * mesh->size * mesh->size * mesh->size))
		return (fprintf(stderr, "%s: the surface encloses no volume\n", path), 0);
	recentre(mesh, center);
	mesh->volume = volume;
	mesh->inertiaPerKg = mat3_scale(inertia, 1.0f / volume);
	return (1);
}

int	mesh_shape_load(MeshShape *mesh, const char *path, float size, int model_index)
{
	ObjMesh	obj;
	int		ok;

	memset(mesh, 0, sizeof(*mesh));
	mesh->modelIndex = model_index;
	snprintf(mesh->path, sizeof(mesh->path), "%s", path);
	mesh->size = size;
	if (!obj_load(path, &obj))
		return (0);
	ok = take_geometry(mesh, &obj, size);
	obj_free(&obj);
	ok = ok && mesh_repair(mesh, path) && mass_properties(mesh, path) && mesh_normals_build(mesh) && bvh_build(mesh);
	if (!ok)
		mesh_shape_free(mesh);
	return (ok);
}

void	mesh_shape_free(MeshShape *mesh)
{
	free(mesh->vertices);
	free(mesh->corners);
	free(mesh->neighbours);
	free(mesh->edges);
	free(mesh->faceNormals);
	free(mesh->vertexNormals);
	free(mesh->nodes);
	free(mesh->triangleOrder);
	memset(mesh, 0, sizeof(*mesh));
}

void	mesh_shape_rescale(MeshShape *mesh, float size)
{
	float	factor = size / mesh->size;
	int		i;

	i = 0;
	while (i < mesh->vertexCount)
	{
		mesh->vertices[i] = vec3_scale(mesh->vertices[i], factor);
		i++;
	}
	i = 0;
	while (i < mesh->nodeCount)
	{
		mesh->nodes[i].boundsMin = vec3_scale(mesh->nodes[i].boundsMin, factor);
		mesh->nodes[i].boundsMax = vec3_scale(mesh->nodes[i].boundsMax, factor);
		i++;
	}
	mesh->boundsMin = vec3_scale(mesh->boundsMin, factor);
	mesh->boundsMax = vec3_scale(mesh->boundsMax, factor);
	mesh->volume *= factor * factor * factor;
	mesh->inertiaPerKg = mat3_scale(mesh->inertiaPerKg, factor * factor);
	mesh->size = size;
}

Vec3	mesh_to_local(const RigidBody *body, Vec3 world_point)
{
	return (quat_rotate(quat_conjugate(body->orientation), vec3_sub(world_point, body->position)));
}

Vec3	mesh_to_world(const RigidBody *body, Vec3 local_point)
{
	return (vec3_add(body->position, quat_rotate(body->orientation, local_point)));
}
