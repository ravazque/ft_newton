
#include "newton.h"

/* Uploads interleaved vertex data (position + normal, 6 floats/vertex) and an
 * index list to the GPU, building the VBO + EBO + VAO. Shared by every shape. */
static Mesh	mesh_upload(const float *verts, int vert_floats, const unsigned int *indices, int index_count)
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
	glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)((size_t)vert_floats * sizeof(float)), verts, GL_STATIC_DRAW);
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
	const float		h = 0.5f;
	const float		verts[] = {
		h, -h, -h, 1, 0, 0, h, h, -h, 1, 0, 0,
		h, h, h, 1, 0, 0, h, -h, h, 1, 0, 0,
		-h, -h, h, -1, 0, 0, -h, h, h, -1, 0, 0,
		-h, h, -h, -1, 0, 0, -h, -h, -h, -1, 0, 0,
		-h, h, -h, 0, 1, 0, -h, h, h, 0, 1, 0,
		h, h, h, 0, 1, 0, h, h, -h, 0, 1, 0,
		-h, -h, h, 0, -1, 0, -h, -h, -h, 0, -1, 0,
		h, -h, -h, 0, -1, 0, h, -h, h, 0, -1, 0,
		-h, -h, h, 0, 0, 1, h, -h, h, 0, 0, 1,
		h, h, h, 0, 0, 1, -h, h, h, 0, 0, 1,
		h, -h, -h, 0, 0, -1, -h, -h, -h, 0, 0, -1,
		-h, h, -h, 0, 0, -1, h, h, -h, 0, 0, -1};
	unsigned int	indices[36];
	unsigned int	f;
	unsigned int	o;

	f = 0;
	while (f < 6)
	{
		o = f * 6;
		indices[o + 0] = f * 4 + 0;
		indices[o + 1] = f * 4 + 1;
		indices[o + 2] = f * 4 + 2;
		indices[o + 3] = f * 4 + 0;
		indices[o + 4] = f * 4 + 2;
		indices[o + 5] = f * 4 + 3;
		f++;
	}
	return (mesh_upload(verts, 24 * 6, indices, 36));
}

Mesh	mesh_sphere(int segments)
{
	float			*verts;
	unsigned int	*idx;
	Mesh			mesh;
	int				i;
	int				j;
	int				v;
	int				o;
	float			phi;
	float			theta;
	float			y;
	float			ring;
	float			px;
	float			pz;
	const float		radius = 0.5f;

	if (segments < 3)
		segments = 3;
	verts = malloc((size_t)(segments + 1) * (segments + 1) * 6 * sizeof(float));
	idx = malloc((size_t)segments * segments * 6 * sizeof(unsigned int));
	if (!verts || !idx)
	{
		free(verts);
		free(idx);
		memset(&mesh, 0, sizeof(mesh));
		return (mesh);
	}
	v = 0;
	i = 0;
	while (i <= segments)
	{
		phi = FTN_PI * (float)i / (float)segments;
		y = radius * cosf(phi);
		ring = radius * sinf(phi);
		j = 0;
		while (j <= segments)
		{
			theta = 2.0f * FTN_PI * (float)j / (float)segments;
			px = ring * cosf(theta);
			pz = ring * sinf(theta);
			verts[v++] = px;
			verts[v++] = y;
			verts[v++] = pz;
			verts[v++] = px / radius;
			verts[v++] = y / radius;
			verts[v++] = pz / radius;
			j++;
		}
		i++;
	}
	o = 0;
	i = 0;
	while (i < segments)
	{
		j = 0;
		while (j < segments)
		{
			idx[o++] = (unsigned int)(i * (segments + 1) + j);
			idx[o++] = (unsigned int)((i + 1) * (segments + 1) + j);
			idx[o++] = (unsigned int)(i * (segments + 1) + j + 1);
			idx[o++] = (unsigned int)(i * (segments + 1) + j + 1);
			idx[o++] = (unsigned int)((i + 1) * (segments + 1) + j);
			idx[o++] = (unsigned int)((i + 1) * (segments + 1) + j + 1);
			j++;
		}
		i++;
	}
	mesh = mesh_upload(verts, (segments + 1) * (segments + 1) * 6, idx, segments * segments * 6);
	free(verts);
	free(idx);
	return (mesh);
}

Mesh	mesh_plane(float size)
{
	const float			h = size * 0.5f;
	const float			verts[] = {
		-h, 0.0f, -h, 0.0f, 1.0f, 0.0f,
		h, 0.0f, -h, 0.0f, 1.0f, 0.0f,
		h, 0.0f, h, 0.0f, 1.0f, 0.0f,
		-h, 0.0f, h, 0.0f, 1.0f, 0.0f};
	const unsigned int	indices[] = {0, 1, 2, 0, 2, 3};

	return (mesh_upload(verts, 4 * 6, indices, 6));
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
