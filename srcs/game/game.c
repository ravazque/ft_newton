#include "newton.h"

/*
 * The application: start-up, the main loop and shut-down. Each frame reads the
 * input (input.c), advances the simulation in fixed 1/120 s steps, draws
 * (draw.c, hud.c) and sleeps off the rest of the frame, so the physics never
 * depends on the display rate.
*/

int	game_init(Game *g, int argc, char **argv)
{
	memset(g, 0, sizeof(*g));
	g->camera = camera_default();
	check_input(argc, argv, &g->camera);
	if (!scene_load_assets(g))
		return (0);
	if (!window_init(&g->window, g->camera.win_width, g->camera.win_height, WIN_TITLE))
		return (0);
	if (!renderer_init(&g->renderer))
		return (window_destroy(&g->window), 0);
	if (!ui_init(&g->ui))
		return (window_destroy(&g->window), 0);
	world_init(&g->world);
	menu_rows_build(g);
	g->hud = hud_default();
	g->fixedDt = FIXED_DT;
	g->timeScale = 1.0f;
	g->planeMesh = mesh_plane();
	g->sphereMesh = mesh_sphere(SPHERE_SEGMENTS);
	g->cubeMesh = mesh_cube();
	scene_build(g);
	action_sync_draft(g);
	return (1);
}

void	game_shutdown(Game *g)
{
	ui_destroy(&g->ui);
	mesh_release(&g->planeMesh);
	mesh_release(&g->sphereMesh);
	mesh_release(&g->cubeMesh);
	world_destroy(&g->world);
	window_destroy(&g->window);
}

/* The time scale stretches the simulated time of a frame, never the step: at 4x
 * the world takes four times as many steps, so a fast apple still cannot skip
 * through a block. A scene too heavy to keep up drops time instead of piling it up. */
static void	step_world(Game *g, float *accumulator, float frame_time)
{
	int	steps;

	if (!g->paused)
		*accumulator += frame_time * g->timeScale;
	steps = 0;
	while (*accumulator >= g->fixedDt && steps < MAX_STEPS_PER_FRAME)
	{
		world_step(&g->world, g->fixedDt);
		*accumulator -= g->fixedDt;
		steps++;
	}
	if (steps == MAX_STEPS_PER_FRAME)
		*accumulator = 0.0f;
}

static void	limit_frame_rate(double frame_start)
{
	struct timespec	pause;
	double			remaining;

	remaining = frame_start + 1.0 / FPS_CAP - glfwGetTime();
	if (remaining <= 0.0)
		return ;
	pause.tv_sec = 0;
	pause.tv_nsec = (long)(remaining * 1e9);
	nanosleep(&pause, NULL);
}

void	game_run(Game *g)
{
	double	prev = glfwGetTime();
	double	now;
	float	accumulator = 0.0f;
	float	frame_time;

	while (!window_should_close(&g->window))
	{
		window_poll_events(&g->window);
		now = glfwGetTime();
		frame_time = fminf((float)(now - prev), 0.25f);
		prev = now;
		input_poll(g, frame_time);
		if (g->quit)
			break ;
		step_world(g, &accumulator, frame_time);
		hud_update(&g->hud, frame_time);
		draw_frame(g);
		window_swap_buffers(&g->window);
		limit_frame_rate(now);
	}
}
