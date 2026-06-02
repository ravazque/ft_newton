
#include "newton.h"

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

/* Same as renderer_draw but with flat (unlit) shading: used for debug
 * wireframes and any overlay where lighting would only get in the way. */
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
