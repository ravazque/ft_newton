
#include "newton.h"

Hud	hud_default(void)
{
	Hud	h;

	h.visible = 1;
	h.fps = 0.0f;
	return (h);
}

void	hud_update(Hud *h, float frame_time_seconds)
{
	/* TODO: smooth the FPS from the per-frame time, e.g. an exponential moving
	 * average: h->fps += (1/frame_time - h->fps) * 0.1f (guard dt > 0). */
	(void)h;
	(void)frame_time_seconds;
}

void	hud_draw(const Hud *h, const World *w)
{
	/* TODO (mandatory): show the FPS counter and the object counter
	 * (w->bodyCount). Hide everything when !h->visible for a clean view.
	 * Easiest path: glfwSetWindowTitle with a formatted string.
	 * Nicer path: a textured bitmap-font overlay (e.g. stb_easy_font).
	*/
	(void)h;
	(void)w;
}
