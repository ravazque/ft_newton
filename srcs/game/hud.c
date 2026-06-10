
#include "newton.h"

Hud	hud_default(void)
{
	Hud	h;

	h.visible = 1;
	h.fps = 0.0f;
	return (h);
}

/* Exponential moving average so the FPS readout does not flicker. */
void	hud_update(Hud *h, float frame_time_seconds)
{
	if (frame_time_seconds <= 0.0f)
		return ;
	h->fps += (1.0f / frame_time_seconds - h->fps) * 0.1f;
}

/* The mandatory FPS + object counters, shown in the window title so the
 * scene itself stays clean. Hidden -> plain title. */
void	hud_draw(const Hud *h, const World *w, Window *win)
{
	char	title[128];

	if (!h->visible)
	{
		glfwSetWindowTitle(window_handle(win), WIN_TITLE);
		return ;
	}
	snprintf(title, sizeof(title), WIN_TITLE "  |  FPS: %.0f  |  objects: %d",
		(double)h->fps, w->bodyCount);
	glfwSetWindowTitle(window_handle(win), title);
}
