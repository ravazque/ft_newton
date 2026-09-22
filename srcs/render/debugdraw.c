
#include "newton.h"

/* The debug display: every collider drawn as a wireframe,
 * toggleable at runtime. Shapes are slightly inflated (and the plane slightly
 * lifted) so the lines sit on top of the solid render without z-fighting. */
void	debugdraw_draw_colliders(const DebugDraw *d, const World *w, Renderer *r, const Mesh *cube, const Mesh *sphere, const Mesh *plane)
{
	const RigidBody	*b;
	Vec3			color;
	Vec3			lifted;
	Mat4			model;
	float			s;
	int				i;

	if (!d->enabled)
		return ;
	renderer_set_wireframe(r, 1);
	color = vec3(1.0f, 0.85f, 0.20f);
	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		if (b->collider.type == SHAPE_SPHERE)
		{
			s = b->collider.radius * 2.0f * 1.01f;
			model = mat4_transform(b->position, b->orientation, vec3(s, s, s));
			renderer_draw_flat(r, sphere, model, color);
		}
		else if (b->collider.type == SHAPE_BOX)
		{
			model = mat4_transform(b->position, b->orientation, vec3_scale(b->collider.halfExtents, 2.0f * 1.01f));
			renderer_draw_flat(r, cube, model, color);
		}
		else
		{
			lifted = vec3_add(collider_plane_origin(&b->collider), vec3_scale(b->collider.normal, 0.01f));
			model = mat4_transform(lifted, collider_plane_rotation(&b->collider), vec3(1.0f, 1.0f, 1.0f));
			renderer_draw_flat(r, plane, model, color);
		}
		i++;
	}
	renderer_set_wireframe(r, 0);
}
