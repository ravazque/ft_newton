
#include "newton.h"

/*
 * [TODO] Top-level application. Once this is ready, replace the milestone-1 body
 * in main.c with: { Game g; if (game_init(&g)) game_run(&g); }
 *
 * game_init: window_init, renderer_init, build the three meshes (cube/sphere/
 *   plane), world_init, build the scene (a static ground plane + a structure),
 *   camera_default, catapult_default, hud_default.
 *
 * game_run: the fixed-timestep main loop (docs/info.md 5.1 and 6):
 *   while (!window_should_close(&g->window)) {
 *       window_poll_events();
 *       process input  (gravity / time scale / launch params / fire / toggles)
 *       accumulator += frame_time;
 *       while (accumulator >= g->fixedDt) {
 *           world_step(&g->world, g->fixedDt * g->timeScale);
 *           accumulator -= g->fixedDt;
 *       }
 *       renderer_begin_frame(); draw bodies; debugdraw; hud; swap;
 *   }
*/

int	game_init(Game *g)
{
	(void)g;
	return (0);
}

void	game_run(Game *g)
{
	(void)g;
}
