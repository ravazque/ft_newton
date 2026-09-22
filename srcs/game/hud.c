
#include "newton.h"

#define HUD_REFRESH_PERIOD 0.2f

Hud	hud_default(void)
{
	Hud	h;

	h.visible = 1;
	h.fps = 0.0f;
	h.refreshTimer = 0.0f;
	h.refresh = 1;
	return (h);
}

/* Exponential moving average so the FPS readout does not flicker; the title
 * is only rewritten a few times per second (window managers dislike more). */
void	hud_update(Hud *h, float frame_time_seconds)
{
	if (frame_time_seconds <= 0.0f)
		return ;
	h->fps += (1.0f / frame_time_seconds - h->fps) * 0.1f;
	h->refreshTimer += frame_time_seconds;
	if (h->refreshTimer >= HUD_REFRESH_PERIOD)
	{
		h->refreshTimer = 0.0f;
		h->refresh = 1;
	}
}

/* The FPS + object counters plus the live-tuned values, shown in
 * the window title so the scene itself stays clean. Hidden -> plain title. */
void	hud_draw(Hud *h, const World *w, const Trebuchet *c, float time_scale, Window *win, int paused)
{
	char		title[192];
	const char	*state;
	int			n;

	if (!h->refresh)
		return ;
	h->refresh = 0;
	if (!h->visible)
	{
		glfwSetWindowTitle(window_handle(win), WIN_TITLE);
		return ;
	}
	state = "";
	if (paused)
		state = "  |  PAUSED";
	n = snprintf(title, sizeof(title), WIN_TITLE "  |  FPS: %.0f  |  objects: %d  |  g: %.2f  |  time: x%.2f", (double)h->fps, w->bodyCount, (double)w->gravity.y, (double)time_scale);
	snprintf(title + n, sizeof(title) - (size_t)n, "  |  speed: %.1f m/s  |  angle: %.0f deg  |  mass: %.2f kg%s", (double)c->launchSpeed, (double)c->launchAngle, (double)c->projectileMass, state);
	glfwSetWindowTitle(window_handle(win), title);
}
