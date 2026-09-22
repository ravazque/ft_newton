#include "newton.h"

/* Draw calls of the 3D scene: per-frame camera matrices, then one model matrix, color and
 * lighting mode per mesh. */

int	renderer_init(Renderer *r)
{
	if (!shader_load(&r->shader, SHADER_VERT, SHADER_FRAG))
		return (0);
	glEnable(GL_DEPTH_TEST);
	glClearColor(CLEAR_R, CLEAR_G, CLEAR_B, 1.0f);
	r->view = mat4_identity();
	r->proj = mat4_identity();
	return (1);
}

void	renderer_begin_frame(Renderer *r, const Camera *cam, float aspect)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	r->view = camera_view(cam);
	r->proj = camera_projection(cam, aspect);
	shader_use(&r->shader);
	shader_set_mat4(&r->shader, "uView", r->view);
	shader_set_mat4(&r->shader, "uProjection", r->proj);
}

void	renderer_draw(Renderer *r, const Mesh *mesh, Mat4 model, Vec3 color)
{
	shader_use(&r->shader);
	shader_set_mat4(&r->shader, "uModel", model);
	shader_set_vec3(&r->shader, "uColor", color);
	shader_set_int(&r->shader, "uLit", 1);
	mesh_draw(mesh);
}

/* Unlit: debug wireframes and the aim bar. */
void	renderer_draw_flat(Renderer *r, const Mesh *mesh, Mat4 model, Vec3 color)
{
	shader_use(&r->shader);
	shader_set_mat4(&r->shader, "uModel", model);
	shader_set_vec3(&r->shader, "uColor", color);
	shader_set_int(&r->shader, "uLit", 0);
	mesh_draw(mesh);
}

void	renderer_set_wireframe(Renderer *r, int on)
{
	(void)r;
	if (on)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

/* Unit mesh -> the body's collider: scaled to its size, placed at its pose. A plane is placed
 * by its equation, not its body position. */
Mat4	renderer_body_model(const RigidBody *b, float inflate)
{
	const Collider	*c = &b->collider;
	float			s;

	if (c->type == SHAPE_SPHERE)
	{
		s = c->radius * 2.0f * inflate;
		return (mat4_transform(b->position, b->orientation, vec3(s, s, s)));
	}
	if (c->type == SHAPE_BOX)
		return (mat4_transform(b->position, b->orientation, vec3_scale(c->halfExtents, 2.0f * inflate)));
	s = c->halfSize * 2.0f;
	return (mat4_transform(collider_plane_origin(c), collider_plane_rotation(c), vec3(s, 1.0f, s)));
}

const Mesh	*renderer_body_mesh(const RigidBody *b, const Mesh *cube, const Mesh *sphere, const Mesh *plane)
{
	if (b->collider.type == SHAPE_SPHERE)
		return (sphere);
	if (b->collider.type == SHAPE_BOX)
		return (cube);
	return (plane);
}
