#include "newton.h"

/*
 * Every key the game reads. These act on the running simulation at once (direct
 * control of gravity, time, launch speed, angle and apple mass); the menu, which
 * owns keyboard and mouse while open, edits a draft instead (actions.c).
*/

/* Pressed this frame (edge), not held. */
static int	key_pressed(Game *g, int key)
{
	static char	prev[GLFW_KEY_LAST + 1];
	int			down;

	down = (glfwGetKey(window_handle(&g->window), key) == GLFW_PRESS);
	if (down && !prev[key])
	{
		prev[key] = 1;
		return (1);
	}
	prev[key] = (char)down;
	return (0);
}

static int	key_down(Game *g, int key)
{
	return (glfwGetKey(window_handle(&g->window), key) == GLFW_PRESS);
}

/* Once per press: firing, spawning and the display toggles. */
static void	handle_actions(Game *g)
{
	if (key_pressed(g, GLFW_KEY_SPACE))
		trebuchet_fire(&g->trebuchet, &g->appleDef, &g->world);
	if (key_pressed(g, GLFW_KEY_1))
		structure_spawn_wall(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 6, 6);
	if (key_pressed(g, GLFW_KEY_2))
		structure_spawn_pyramid(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 7);
	if (key_pressed(g, GLFW_KEY_3))
		structure_spawn_tower(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 8);
	if (key_pressed(g, GLFW_KEY_4))
		structure_spawn_wall(&g->world, &g->blockDef, vec3(STRESS_X, 0.0f, 0.0f), 15, 10);
	if (key_pressed(g, GLFW_KEY_P))
	{
		g->paused = !g->paused;
		g->hud.refresh = 1;
	}
	if (key_pressed(g, GLFW_KEY_F1))
		g->debug.enabled = !g->debug.enabled;
	if (key_pressed(g, GLFW_KEY_H))
	{
		g->hud.visible = !g->hud.visible;
		g->hud.refresh = 1;
	}
}

/* Held keys change a value at a fixed rate per second. The apple mass is the mass of the NEXT apples. */
static void	handle_launch(Game *g, float frame_time)
{
	if (key_down(g, GLFW_KEY_I))
		g->trebuchet.launchAngle = fminf(LAUNCH_ANGLE_MAX, g->trebuchet.launchAngle + CTRL_ANGLE_RATE * frame_time);
	if (key_down(g, GLFW_KEY_K))
		g->trebuchet.launchAngle = fmaxf(LAUNCH_ANGLE_MIN, g->trebuchet.launchAngle - CTRL_ANGLE_RATE * frame_time);
	if (key_down(g, GLFW_KEY_L))
		g->trebuchet.launchSpeed = fminf(LAUNCH_SPEED_MAX, g->trebuchet.launchSpeed + CTRL_SPEED_RATE * frame_time);
	if (key_down(g, GLFW_KEY_J))
		g->trebuchet.launchSpeed = fmaxf(LAUNCH_SPEED_MIN, g->trebuchet.launchSpeed - CTRL_SPEED_RATE * frame_time);
	if (key_down(g, GLFW_KEY_E))
		g->appleDef.mass = fminf(PROJECTILE_MASS_MAX, g->appleDef.mass + CTRL_MASS_RATE * frame_time);
	if (key_down(g, GLFW_KEY_Q))
		g->appleDef.mass = fmaxf(PROJECTILE_MASS_MIN, g->appleDef.mass - CTRL_MASS_RATE * frame_time);
	g->trebuchet.projectileMass = g->appleDef.mass;
}

/* Changing gravity wakes everything, so a resting pile reacts again. */
static void	handle_world_tuning(Game *g, float frame_time)
{
	if (key_down(g, GLFW_KEY_G))
	{
		g->world.gravity.y = fminf(GRAVITY_MAX, g->world.gravity.y + CTRL_GRAVITY_RATE * frame_time);
		world_wake_all(&g->world);
	}
	if (key_down(g, GLFW_KEY_B))
	{
		g->world.gravity.y = fmaxf(GRAVITY_MIN, g->world.gravity.y - CTRL_GRAVITY_RATE * frame_time);
		world_wake_all(&g->world);
	}
	if (key_down(g, GLFW_KEY_Y))
		g->timeScale = fminf(TIME_SCALE_MAX, g->timeScale + CTRL_TIME_RATE * frame_time);
	if (key_down(g, GLFW_KEY_T))
		g->timeScale = fmaxf(0.0f, g->timeScale - CTRL_TIME_RATE * frame_time);
}

/* WASD fly, R / F rise and fall, arrows turn, + / - change the fly speed. */
static void	handle_camera(Game *g, float frame_time)
{
	float	step = g->camera.moveSpeed * frame_time;
	Vec3	move = vec3(0.0f, 0.0f, 0.0f);

	if (key_down(g, GLFW_KEY_W))
		move.z += step;
	if (key_down(g, GLFW_KEY_S))
		move.z -= step;
	if (key_down(g, GLFW_KEY_D))
		move.x += step;
	if (key_down(g, GLFW_KEY_A))
		move.x -= step;
	if (key_down(g, GLFW_KEY_R))
		move.y += step;
	if (key_down(g, GLFW_KEY_F))
		move.y -= step;
	camera_move(&g->camera, move);
	if (key_down(g, GLFW_KEY_RIGHT))
		camera_look(&g->camera, CAM_LOOK_SPEED * frame_time, 0.0f);
	if (key_down(g, GLFW_KEY_LEFT))
		camera_look(&g->camera, -CAM_LOOK_SPEED * frame_time, 0.0f);
	if (key_down(g, GLFW_KEY_UP))
		camera_look(&g->camera, 0.0f, CAM_LOOK_SPEED * frame_time);
	if (key_down(g, GLFW_KEY_DOWN))
		camera_look(&g->camera, 0.0f, -CAM_LOOK_SPEED * frame_time);
	if (key_down(g, GLFW_KEY_EQUAL) || key_down(g, GLFW_KEY_KP_ADD))
		camera_change_speed(&g->camera, CAM_SPEED_RATE * frame_time);
	if (key_down(g, GLFW_KEY_MINUS) || key_down(g, GLFW_KEY_KP_SUBTRACT))
		camera_change_speed(&g->camera, -CAM_SPEED_RATE * frame_time);
}

/* ESC opens the menu (pausing) or closes it (resuming). */
static void	toggle_menu(Game *g)
{
	g->menu.open = !g->menu.open;
	g->paused = g->menu.open;
	if (g->menu.open)
		action_sync_draft(g);
	g->hud.refresh = 1;
}


static void	handle_menu(Game *g, float frame_time)
{
	float	width;
	float	height;

	window_size(&g->window, &width, &height);
	menu_layout(&g->menu, width, height);
	menu_update(&g->menu, &g->window, frame_time);
	if (g->menu.pending != ACTION_NONE)
		action_run(g, g->menu.pending);
}


void	input_poll(Game *g, float frame_time)
{
	if (key_pressed(g, GLFW_KEY_ESCAPE))
		toggle_menu(g);
	if (g->menu.open)
	{
		handle_menu(g, frame_time);
		return ;
	}
	handle_actions(g);
	handle_launch(g, frame_time);
	handle_world_tuning(g, frame_time);
	handle_camera(g, frame_time);
}
