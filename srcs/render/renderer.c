#include "newton.h"

// Draw calls of the 3D scene: camera matrices once per frame, then a model matrix, a colour and a shading mode per mesh.

int	renderer_init(Renderer *renderer)
{
	if (!shader_load(&renderer->shader, SHADER_VERT, SHADER_FRAG))
		return (0);
	glEnable(GL_DEPTH_TEST);
	glClearColor(CLEAR_R, CLEAR_G, CLEAR_B, 1.0f);
	renderer->view = mat4_identity();
	renderer->projection = mat4_identity();
	return (1);
}

void	renderer_begin_frame(Renderer *renderer, const Camera *camera, float aspect)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	renderer->view = camera_view(camera);
	renderer->projection = camera_projection(camera, aspect);
	shader_use(&renderer->shader);
	shader_set_mat4(&renderer->shader, "uView", renderer->view);
	shader_set_mat4(&renderer->shader, "uProjection", renderer->projection);
}

void	renderer_draw(Renderer *renderer, const Mesh *mesh, Mat4 model, Vec3 color)
{
	shader_use(&renderer->shader);
	shader_set_mat4(&renderer->shader, "uModel", model);
	shader_set_vec3(&renderer->shader, "uColor", color);
	shader_set_int(&renderer->shader, "uLit", 1);
	mesh_draw(mesh);
}

void	renderer_draw_flat(Renderer *renderer, const Mesh *mesh, Mat4 model, Vec3 color)
{
	shader_use(&renderer->shader);
	shader_set_mat4(&renderer->shader, "uModel", model);
	shader_set_vec3(&renderer->shader, "uColor", color);
	shader_set_int(&renderer->shader, "uLit", 0);
	mesh_draw(mesh);
}

void	renderer_set_wireframe(Renderer *renderer, int enabled)
{
	(void)renderer;
	if (enabled)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

// A unit mesh scaled to the collider and placed at the body's pose; a model's mesh is already at its size, a plane sits on its equation.
Mat4	renderer_body_model(const RigidBody *body, float inflate)
{
	const Collider	*collider = &body->collider;
	float			side;

	if (collider->type == SHAPE_SPHERE)
	{
		side = collider->radius * 2.0f * inflate;
		return (mat4_transform(body->position, body->orientation, vec3(side, side, side)));
	}
	if (collider->type == SHAPE_BOX)
		return (mat4_transform(body->position, body->orientation, vec3_scale(collider->halfExtents, 2.0f * inflate)));
	if (collider->type == SHAPE_MESH)
		return (mat4_transform(body->position, body->orientation, vec3(inflate, inflate, inflate)));
	side = collider->halfSize * 2.0f;
	return (mat4_transform(collider_plane_origin(collider), collider_plane_rotation(collider), vec3(side, 1.0f, side)));
}

const Mesh	*renderer_body_mesh(const RigidBody *body, const Meshes *meshes)
{
	if (body->collider.type == SHAPE_SPHERE)
		return (&meshes->sphere);
	if (body->collider.type == SHAPE_BOX)
		return (&meshes->cube);
	if (body->collider.type == SHAPE_MESH)
		return (&meshes->models[body->collider.mesh->modelIndex]);
	return (&meshes->plane);
}
