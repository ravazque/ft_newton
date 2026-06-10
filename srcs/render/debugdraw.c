
#include "newton.h"

/* The mandatory debug display: every collider drawn as a wireframe,
 * toggleable at runtime. Shapes are slightly inflated (and the plane slightly
 * lifted) so the lines sit on top of the solid render without z-fighting. */
void	debugdraw_draw_colliders(const DebugDraw *d, const World *w, Renderer *r, const Mesh *cube, const Mesh *sphere, const Mesh *plane)
{
	Vec3	color;
	int		i;

	if (!d->enabled)
		return ;
	renderer_set_wireframe(r, 1);
	color = vec3(1.0f, 0.85f, 0.20f);
	i = 0;
	while (i < w->bodyCount)
	{
		const RigidBody	*b = &w->bodies[i];

		if (b->collider.type == SHAPE_SPHERE)
		{
			float	s = b->collider.radius * 2.0f * 1.01f;

			renderer_draw_flat(r, sphere,
				mat4_transform(b->position, b->orientation, vec3(s, s, s)),
				color);
		}
		else if (b->collider.type == SHAPE_BOX)
			renderer_draw_flat(r, cube,
				mat4_transform(b->position, b->orientation,
					vec3_scale(b->collider.halfExtents, 2.0f * 1.01f)), color);
		else
			renderer_draw_flat(r, plane,
				mat4_transform(
					vec3_add(b->position,
						vec3_scale(b->collider.normal, 0.01f)),
					b->orientation, vec3(1.0f, 1.0f, 1.0f)), color);
		i++;
	}
	renderer_set_wireframe(r, 0);
}
