#include "newton.h"

// The debug display (F1): every collider as a slightly inflated wireframe, then every contact as a red marker and a cyan normal.

static void	draw_colliders(const World *world, Renderer *renderer, const Meshes *meshes)
{
	const RigidBody	*body;
	Mat4			model;
	int				i;

	renderer_set_wireframe(renderer, 1);
	i = 0;
	while (i < world->bodyCount)
	{
		body = &world->bodies[i];
		model = renderer_body_model(body, DEBUG_WIRE_INFLATE);
		if (body->collider.type == SHAPE_PLANE)
		{
			model.cells[12] += body->collider.normal.x * DEBUG_PLANE_LIFT;
			model.cells[13] += body->collider.normal.y * DEBUG_PLANE_LIFT;
			model.cells[14] += body->collider.normal.z * DEBUG_PLANE_LIFT;
		}
		renderer_draw_flat(renderer, renderer_body_mesh(body, meshes), model, vec3(1.0f, 0.85f, 0.20f));
		i++;
	}
	renderer_set_wireframe(renderer, 0);
}

static void	draw_contacts(const World *world, Renderer *renderer, const Mesh *cube)
{
	const Contact	*contact;
	Vec3			stick_center;
	Vec3			stick_size;
	float			size = DEBUG_CONTACT_SIZE;
	int				i;

	glDisable(GL_DEPTH_TEST);
	stick_size = vec3(size * 0.3f, size * 0.3f, DEBUG_NORMAL_LENGTH);
	i = 0;
	while (i < world->contactCount)
	{
		contact = &world->contacts[i];
		stick_center = vec3_add(contact->point, vec3_scale(contact->normal, DEBUG_NORMAL_LENGTH * 0.5f));
		renderer_draw_flat(renderer, cube, mat4_transform(contact->point, quat_identity(), vec3(size, size, size)), vec3(1.0f, 0.15f, 0.15f));
		renderer_draw_flat(renderer, cube, mat4_transform(stick_center, quat_from_to(vec3(0.0f, 0.0f, 1.0f), contact->normal), stick_size), vec3(0.30f, 0.90f, 1.0f));
		i++;
	}
	glEnable(GL_DEPTH_TEST);
}

void	debugdraw_draw(const DebugDraw *debug, const World *world, Renderer *renderer, const Meshes *meshes)
{
	if (!debug->enabled)
		return ;
	draw_colliders(world, renderer, meshes);
	draw_contacts(world, renderer, &meshes->cube);
}
