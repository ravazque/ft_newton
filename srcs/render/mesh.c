#include "newton.h"

// Geometry on the GPU: the unit cube, sphere and square every primitive is drawn with, and one mesh per model.

// Interleaved position and normal (6 floats per vertex) and an index list, into a vertex buffer, index buffer and vertex array.
static Mesh	mesh_upload(const float *vertices, int vertex_floats, const unsigned int *indices, int index_count)
{
	Mesh	mesh;
	GLsizei	stride;

	memset(&mesh, 0, sizeof(mesh));
	mesh.indexCount = index_count;
	glGenVertexArrays(1, &mesh.vao);
	glGenBuffers(1, &mesh.vbo);
	glGenBuffers(1, &mesh.ebo);
	glBindVertexArray(mesh.vao);
	glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
	glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)((size_t)vertex_floats * sizeof(float)), vertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)((size_t)index_count * sizeof(unsigned int)), indices, GL_STATIC_DRAW);
	stride = 6 * (GLsizei)sizeof(float);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void *)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void *)(3 * sizeof(float)));
	glBindVertexArray(0);
	return (mesh);
}

Mesh	mesh_cube(void)
{
	const float		half = 0.5f;
	const float		vertices[] = {
		half, -half, -half, 1, 0, 0, half, half, -half, 1, 0, 0,
		half, half, half, 1, 0, 0, half, -half, half, 1, 0, 0,
		-half, -half, half, -1, 0, 0, -half, half, half, -1, 0, 0,
		-half, half, -half, -1, 0, 0, -half, -half, -half, -1, 0, 0,
		-half, half, -half, 0, 1, 0, -half, half, half, 0, 1, 0,
		half, half, half, 0, 1, 0, half, half, -half, 0, 1, 0,
		-half, -half, half, 0, -1, 0, -half, -half, -half, 0, -1, 0,
		half, -half, -half, 0, -1, 0, half, -half, half, 0, -1, 0,
		-half, -half, half, 0, 0, 1, half, -half, half, 0, 0, 1,
		half, half, half, 0, 0, 1, -half, half, half, 0, 0, 1,
		half, -half, -half, 0, 0, -1, -half, -half, -half, 0, 0, -1,
		-half, half, -half, 0, 0, -1, half, half, -half, 0, 0, -1};
	unsigned int	indices[36];
	unsigned int	face;
	unsigned int	first;

	face = 0;
	while (face < 6)
	{
		first = face * 6;
		indices[first + 0] = face * 4 + 0;
		indices[first + 1] = face * 4 + 1;
		indices[first + 2] = face * 4 + 2;
		indices[first + 3] = face * 4 + 0;
		indices[first + 4] = face * 4 + 2;
		indices[first + 5] = face * 4 + 3;
		face++;
	}
	return (mesh_upload(vertices, 24 * 6, indices, 36));
}

static void	sphere_vertices(float *vertices, int segments)
{
	float	ring_radius;
	float	height;
	float	angle;
	int		written;
	int		ring;
	int		slice;

	written = 0;
	ring = 0;
	while (ring <= segments)
	{
		height = 0.5f * cosf(FTN_PI * (float)ring / (float)segments);
		ring_radius = 0.5f * sinf(FTN_PI * (float)ring / (float)segments);
		slice = 0;
		while (slice <= segments)
		{
			angle = 2.0f * FTN_PI * (float)slice / (float)segments;
			vertices[written++] = ring_radius * cosf(angle);
			vertices[written++] = height;
			vertices[written++] = ring_radius * sinf(angle);
			vertices[written++] = ring_radius * cosf(angle) / 0.5f;
			vertices[written++] = height / 0.5f;
			vertices[written++] = ring_radius * sinf(angle) / 0.5f;
			slice++;
		}
		ring++;
	}
}

static void	sphere_indices(unsigned int *indices, int segments)
{
	int	written;
	int	ring;
	int	slice;

	written = 0;
	ring = 0;
	while (ring < segments)
	{
		slice = 0;
		while (slice < segments)
		{
			indices[written++] = (unsigned int)(ring * (segments + 1) + slice);
			indices[written++] = (unsigned int)((ring + 1) * (segments + 1) + slice);
			indices[written++] = (unsigned int)(ring * (segments + 1) + slice + 1);
			indices[written++] = (unsigned int)(ring * (segments + 1) + slice + 1);
			indices[written++] = (unsigned int)((ring + 1) * (segments + 1) + slice);
			indices[written++] = (unsigned int)((ring + 1) * (segments + 1) + slice + 1);
			slice++;
		}
		ring++;
	}
}

Mesh	mesh_sphere(int segments)
{
	float			*vertices;
	unsigned int	*indices;
	Mesh			mesh;

	if (segments < 3)
		segments = 3;
	vertices = malloc((size_t)(segments + 1) * (segments + 1) * 6 * sizeof(float));
	indices = malloc((size_t)segments * segments * 6 * sizeof(unsigned int));
	memset(&mesh, 0, sizeof(mesh));
	if (!vertices || !indices)
		return (free(vertices), free(indices), mesh);
	sphere_vertices(vertices, segments);
	sphere_indices(indices, segments);
	mesh = mesh_upload(vertices, (segments + 1) * (segments + 1) * 6, indices, segments * segments * 6);
	free(vertices);
	free(indices);
	return (mesh);
}

Mesh	mesh_plane(void)
{
	const float			half = 0.5f;
	const float			vertices[] = {
		-half, 0.0f, -half, 0.0f, 1.0f, 0.0f,
		half, 0.0f, -half, 0.0f, 1.0f, 0.0f,
		half, 0.0f, half, 0.0f, 1.0f, 0.0f,
		-half, 0.0f, half, 0.0f, 1.0f, 0.0f};
	const unsigned int	indices[] = {0, 1, 2, 0, 2, 3};

	return (mesh_upload(vertices, 4 * 6, indices, 6));
}

Mesh	mesh_from_shape(const MeshShape *shape)
{
	float			*vertices = malloc((size_t)shape->triangleCount * 18 * sizeof(float));
	unsigned int	*indices = malloc((size_t)shape->triangleCount * 3 * sizeof(unsigned int));
	Mesh			mesh;
	Vec3			position;
	Vec3			normal;
	int				written;
	int				corner;

	memset(&mesh, 0, sizeof(mesh));
	if (!vertices || !indices)
		return (free(vertices), free(indices), mesh);
	written = 0;
	corner = 0;
	while (corner < shape->triangleCount * 3)
	{
		position = shape->vertices[shape->corners[corner]];
		normal = shape->faceNormals[corner / 3];
		vertices[written++] = position.x;
		vertices[written++] = position.y;
		vertices[written++] = position.z;
		vertices[written++] = normal.x;
		vertices[written++] = normal.y;
		vertices[written++] = normal.z;
		indices[corner] = (unsigned int)corner;
		corner++;
	}
	mesh = mesh_upload(vertices, shape->triangleCount * 18, indices, shape->triangleCount * 3);
	free(vertices);
	free(indices);
	return (mesh);
}

void	mesh_draw(const Mesh *mesh)
{
	glBindVertexArray(mesh->vao);
	glDrawElements(GL_TRIANGLES, mesh->indexCount, GL_UNSIGNED_INT, NULL);
	glBindVertexArray(0);
}

void	mesh_release(Mesh *mesh)
{
	if (mesh->ebo)
		glDeleteBuffers(1, &mesh->ebo);
	if (mesh->vbo)
		glDeleteBuffers(1, &mesh->vbo);
	if (mesh->vao)
		glDeleteVertexArrays(1, &mesh->vao);
	memset(mesh, 0, sizeof(*mesh));
}
