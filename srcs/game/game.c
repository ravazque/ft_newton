#include "newton.h"

// Start-up, main loop and shut-down: each frame reads the input, runs fixed physics steps, draws and sleeps the rest.

static void	release_meshes(Game *game)
{
	int	i;

	mesh_release(&game->meshes.plane);
	mesh_release(&game->meshes.sphere);
	mesh_release(&game->meshes.cube);
	i = 0;
	while (i < MODEL_MAX)
	{
		mesh_release(&game->meshes.models[i]);
		i++;
	}
}

static void	upload_meshes(Game *game)
{
	int	i;

	game->meshes.plane = mesh_plane();
	game->meshes.sphere = mesh_sphere(SPHERE_SEGMENTS);
	game->meshes.cube = mesh_cube();
	i = 0;
	while (i < game->models.count)
	{
		game->meshes.models[i] = mesh_from_shape(&game->models.shapes[i]);
		i++;
	}
}

int	game_init(Game *game, int argc, char **argv)
{
	memset(game, 0, sizeof(*game));
	game->camera = camera_default();
	arguments_read(argc, argv, &game->camera);
	if (!scene_load_assets(game))
		return (model_library_free(&game->models), 0);
	if (!window_init(&game->window, game->camera.windowWidth, game->camera.windowHeight, WIN_TITLE))
		return (model_library_free(&game->models), 0);
	if (!renderer_init(&game->renderer) || !ui_init(&game->ui))
		return (ui_destroy(&game->ui), window_destroy(&game->window), model_library_free(&game->models), 0);
	world_init(&game->world);
	game->hud = hud_default();
	game->fixedDt = FIXED_DT;
	game->timeScale = 1.0f;
	upload_meshes(game);
	scene_build(game);
	action_sync_draft(game);
	menu_rows_build(game);
	return (1);
}

void	game_shutdown(Game *game)
{
	ui_destroy(&game->ui);
	release_meshes(game);
	world_destroy(&game->world);
	model_library_free(&game->models);
	window_destroy(&game->window);
}

// The time scale stretches a frame's simulated time, never the step; a scene too heavy to keep up drops time.
static void	step_world(Game *game, float *accumulator, float frame_time)
{
	int	steps;

	if (!game->paused)
		*accumulator += frame_time * game->timeScale;
	steps = 0;
	while (*accumulator >= game->fixedDt && steps < MAX_STEPS_PER_FRAME)
	{
		world_step(&game->world, game->fixedDt);
		*accumulator -= game->fixedDt;
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

void	game_run(Game *game)
{
	double	previous = glfwGetTime();
	double	now;
	float	accumulator = 0.0f;
	float	frame_time;

	while (!window_should_close(&game->window))
	{
		window_poll_events(&game->window);
		now = glfwGetTime();
		frame_time = fminf((float)(now - previous), 0.25f);
		previous = now;
		input_poll(game, frame_time);
		if (game->quit)
			break ;
		step_world(game, &accumulator, frame_time);
		camera_keep_clear(&game->camera, &game->world);
		hud_update(&game->hud, frame_time);
		draw_frame(game);
		window_swap_buffers(&game->window);
		limit_frame_rate(now);
	}
}
