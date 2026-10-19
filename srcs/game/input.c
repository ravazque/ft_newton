#include "newton.h"

// Every key of the game: they act on the running simulation at once, while the open menu edits a draft instead.

static int	key_pressed(Game *game, int key)
{
	static char	was_down[GLFW_KEY_LAST + 1];
	int			down = (glfwGetKey(window_handle(&game->window), key) == GLFW_PRESS);

	if (down && !was_down[key])
	{
		was_down[key] = 1;
		return (1);
	}
	was_down[key] = (char)down;
	return (0);
}

static int	key_down(Game *game, int key)
{
	return (glfwGetKey(window_handle(&game->window), key) == GLFW_PRESS);
}

static void	place_selected_model(Game *game)
{
	if (game->models.count == 0)
		return ;
	model_spawn(&game->world, &game->modelDef, &game->models.shapes[game->models.selected], vec3(TARGET_X, 0.0f, 0.0f));
}

static void	select_model(Game *game, int direction)
{
	model_library_step(&game->models, direction);
	game->hud.refresh = 1;
}

static void	handle_spawns(Game *game)
{
	if (key_pressed(game, GLFW_KEY_SPACE))
		trebuchet_fire(&game->trebuchet, &game->appleDef, &game->world);
	if (key_pressed(game, GLFW_KEY_1))
		structure_spawn_wall(&game->world, &game->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 6, 6);
	if (key_pressed(game, GLFW_KEY_2))
		structure_spawn_pyramid(&game->world, &game->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 7);
	if (key_pressed(game, GLFW_KEY_3))
		structure_spawn_tower(&game->world, &game->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 8);
	if (key_pressed(game, GLFW_KEY_4))
		structure_spawn_wall(&game->world, &game->blockDef, vec3(STRESS_X, 0.0f, 0.0f), 15, 10);
	if (key_pressed(game, GLFW_KEY_5))
		ragdoll_spawn(&game->world, &game->ragdollDef, vec3(TARGET_X, 0.0f, 0.0f));
	if (key_pressed(game, GLFW_KEY_6))
		place_selected_model(game);
	if (key_pressed(game, GLFW_KEY_7))
		select_model(game, 1);
	if (key_pressed(game, GLFW_KEY_8))
		select_model(game, -1);
	if (key_pressed(game, GLFW_KEY_9))
		rain_drop(&game->world, &game->appleDef, vec3(TARGET_X, 0.0f, 0.0f));
}

static void	toggle_gravity(Game *game)
{
	if (game->world.gravity.y != 0.0f)
	{
		game->gravityBeforeOff = game->world.gravity.y;
		game->world.gravity.y = 0.0f;
	}
	else if (game->gravityBeforeOff != 0.0f)
		game->world.gravity.y = game->gravityBeforeOff;
	else
		game->world.gravity.y = GRAVITY_Y;
	world_wake_all(&game->world);
	game->hud.refresh = 1;
}

static void	handle_toggles(Game *game)
{
	if (key_pressed(game, GLFW_KEY_0))
		toggle_gravity(game);
	if (key_pressed(game, GLFW_KEY_P))
	{
		game->paused = !game->paused;
		game->hud.refresh = 1;
	}
	if (key_pressed(game, GLFW_KEY_F1))
		game->debug.enabled = !game->debug.enabled;
	if (key_pressed(game, GLFW_KEY_H))
	{
		game->hud.visible = !game->hud.visible;
		game->hud.refresh = 1;
	}
}

static void	handle_launch(Game *game, float frame_time)
{
	Trebuchet	*trebuchet = &game->trebuchet;

	if (key_down(game, GLFW_KEY_I))
		trebuchet->launchAngle = fminf(LAUNCH_ANGLE_MAX, trebuchet->launchAngle + CTRL_ANGLE_RATE * frame_time);
	if (key_down(game, GLFW_KEY_K))
		trebuchet->launchAngle = fmaxf(LAUNCH_ANGLE_MIN, trebuchet->launchAngle - CTRL_ANGLE_RATE * frame_time);
	if (key_down(game, GLFW_KEY_L))
		trebuchet->launchSpeed = fminf(LAUNCH_SPEED_MAX, trebuchet->launchSpeed + CTRL_SPEED_RATE * frame_time);
	if (key_down(game, GLFW_KEY_J))
		trebuchet->launchSpeed = fmaxf(LAUNCH_SPEED_MIN, trebuchet->launchSpeed - CTRL_SPEED_RATE * frame_time);
	if (key_down(game, GLFW_KEY_E))
		game->appleDef.mass = fminf(PROJECTILE_MASS_MAX, game->appleDef.mass + CTRL_MASS_RATE * frame_time);
	if (key_down(game, GLFW_KEY_Q))
		game->appleDef.mass = fmaxf(PROJECTILE_MASS_MIN, game->appleDef.mass - CTRL_MASS_RATE * frame_time);
	trebuchet->projectileMass = game->appleDef.mass;
}

static void	handle_world_tuning(Game *game, float frame_time)
{
	if (key_down(game, GLFW_KEY_G))
	{
		game->world.gravity.y = fminf(GRAVITY_MAX, game->world.gravity.y + CTRL_GRAVITY_RATE * frame_time);
		world_wake_all(&game->world);
	}
	if (key_down(game, GLFW_KEY_B))
	{
		game->world.gravity.y = fmaxf(GRAVITY_MIN, game->world.gravity.y - CTRL_GRAVITY_RATE * frame_time);
		world_wake_all(&game->world);
	}
	if (key_down(game, GLFW_KEY_Y))
		game->timeScale = fminf(TIME_SCALE_MAX, game->timeScale + CTRL_TIME_RATE * frame_time);
	if (key_down(game, GLFW_KEY_T))
		game->timeScale = fmaxf(0.0f, game->timeScale - CTRL_TIME_RATE * frame_time);
}

static void	handle_camera(Game *game, float frame_time)
{
	float	step = game->camera.moveSpeed * frame_time;
	float	turn = CAM_LOOK_SPEED * frame_time;
	Vec3	move = vec3(0.0f, 0.0f, 0.0f);

	if (key_down(game, GLFW_KEY_W))
		move.z += step;
	if (key_down(game, GLFW_KEY_S))
		move.z -= step;
	if (key_down(game, GLFW_KEY_D))
		move.x += step;
	if (key_down(game, GLFW_KEY_A))
		move.x -= step;
	if (key_down(game, GLFW_KEY_R))
		move.y += step;
	if (key_down(game, GLFW_KEY_F))
		move.y -= step;
	camera_fly(&game->camera, &game->world, move);
	if (key_down(game, GLFW_KEY_RIGHT))
		camera_look(&game->camera, turn, 0.0f);
	if (key_down(game, GLFW_KEY_LEFT))
		camera_look(&game->camera, -turn, 0.0f);
	if (key_down(game, GLFW_KEY_UP))
		camera_look(&game->camera, 0.0f, turn);
	if (key_down(game, GLFW_KEY_DOWN))
		camera_look(&game->camera, 0.0f, -turn);
	if (key_down(game, GLFW_KEY_EQUAL) || key_down(game, GLFW_KEY_KP_ADD))
		camera_change_speed(&game->camera, CAM_SPEED_RATE * frame_time);
	if (key_down(game, GLFW_KEY_MINUS) || key_down(game, GLFW_KEY_KP_SUBTRACT))
		camera_change_speed(&game->camera, -CAM_SPEED_RATE * frame_time);
}

// ESC opens the menu and pauses; from the models page it goes back to the main page, from the main page it resumes.
static void	handle_escape(Game *game)
{
	if (game->menu.open && game->menu.page == MENU_PAGE_MODELS)
	{
		action_run(game, ACTION_BACK);
		return ;
	}
	game->menu.open = !game->menu.open;
	game->paused = game->menu.open;
	if (game->menu.open)
		action_sync_draft(game);
	game->hud.refresh = 1;
}

static void	handle_menu(Game *game, float frame_time)
{
	float	width;
	float	height;

	window_size(&game->window, &width, &height);
	menu_layout(&game->menu, width, height);
	menu_update(&game->menu, &game->window, frame_time);
	if (game->menu.pending != ACTION_NONE)
		action_run(game, game->menu.pending);
}

void	input_poll(Game *game, float frame_time)
{
	if (key_pressed(game, GLFW_KEY_ESCAPE))
		handle_escape(game);
	if (game->menu.open)
	{
		handle_menu(game, frame_time);
		return ;
	}
	handle_spawns(game);
	handle_toggles(game);
	handle_launch(game, frame_time);
	handle_world_tuning(game, frame_time);
	handle_camera(game, frame_time);
}
