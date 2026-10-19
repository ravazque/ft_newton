#include "newton.h"

// The live readout in the window title (FPS, objects, gravity, time, launch settings, selected model); H hides it.

Hud	hud_default(void)
{
	Hud	hud;

	hud.visible = 1;
	hud.fps = 0.0f;
	hud.refreshTimer = 0.0f;
	hud.refresh = 1;
	return (hud);
}

// A moving average keeps the FPS from flickering; the title is rewritten only every HUD_REFRESH_PERIOD.
void	hud_update(Hud *hud, float frame_time)
{
	if (frame_time <= 0.0f)
		return ;
	hud->fps += (1.0f / frame_time - hud->fps) * 0.1f;
	hud->refreshTimer += frame_time;
	if (hud->refreshTimer >= HUD_REFRESH_PERIOD)
	{
		hud->refreshTimer = 0.0f;
		hud->refresh = 1;
	}
}

void	hud_draw(Hud *hud, const Game *game)
{
	char		title[320];
	const char	*model = "none";
	const char	*state = "";
	int			length;

	if (!hud->refresh)
		return ;
	hud->refresh = 0;
	if (!hud->visible)
	{
		glfwSetWindowTitle(window_handle(&game->window), WIN_TITLE);
		return ;
	}
	if (game->models.count > 0)
		model = game->models.names[game->models.selected];
	if (game->paused)
		state = "  |  PAUSED";
	length = snprintf(title, sizeof(title), WIN_TITLE "  |  FPS: %.0f  |  objects: %d  |  g: %.2f  |  time: x%.2f", (double)hud->fps, game->world.bodyCount, (double)game->world.gravity.y, (double)game->timeScale);
	snprintf(title + length, sizeof(title) - (size_t)length, "  |  speed: %.1f m/s  |  angle: %.0f deg  |  mass: %.2f kg  |  model: %s%s", (double)game->trebuchet.launchSpeed, (double)game->trebuchet.launchAngle, (double)game->trebuchet.projectileMass, model, state);
	glfwSetWindowTitle(window_handle(&game->window), title);
}
