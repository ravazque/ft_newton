
#include "newton.h"

/*
 * [TODO] The mandatory debug display: draw every body's collider as a
 * wireframe so the actual physics shapes are visible. Toggleable via
 * d->enabled (flip it from a key press). See docs/info.md 4.4.
 *
 * Sketch:
 *   if (!d->enabled) return;
 *   renderer_set_wireframe(r, 1);
 *   for each body in w:
 *       pick mesh 00:00:00 by body->collider.type (cube / sphere / plane)
 *       model = mat4_transform(body->position, body->orientation, size)
 *       renderer_draw_flat(r, mesh, model, debug_color);
 *   renderer_set_wireframe(r, 0);
*/
void	debugdraw_draw_colliders(const DebugDraw *d, const World *w, Renderer *r, const Mesh *cube, const Mesh *sphere, const Mesh *plane)
{
	(void)d;
	(void)w;
	(void)r;
	(void)cube;
	(void)sphere;
	(void)plane;
}
