#include "newton.h"

/* The debug display (F1): every collider drawn as a flat wireframe, slightly inflated (the
 * plane slightly lifted) so it sits on top of the solid render. */

void	debugdraw_draw_colliders(const DebugDraw *d, const World *w, Renderer *r, const Mesh *cube, const Mesh *sphere, const Mesh *plane)
{
	const RigidBody	*b;
	Mat4			model;
	int				i;

	if (!d->enabled)
		return ;
	renderer_set_wireframe(r, 1);
	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		model = renderer_body_model(b, DEBUG_WIRE_INFLATE);
		if (b->collider.type == SHAPE_PLANE)
		{
			model.m[12] += b->collider.normal.x * DEBUG_PLANE_LIFT;
			model.m[13] += b->collider.normal.y * DEBUG_PLANE_LIFT;
			model.m[14] += b->collider.normal.z * DEBUG_PLANE_LIFT;
		}
		renderer_draw_flat(r, renderer_body_mesh(b, cube, sphere, plane), model, vec3(1.0f, 0.85f, 0.20f));
		i++;
	}
	renderer_set_wireframe(r, 0);
}
