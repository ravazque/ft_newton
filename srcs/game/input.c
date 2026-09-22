
#include "newton.h"

/*
 * THE KEYBOARD: every key the game reads, in one place.
 *
 * Two sets of controls coexist on purpose. These keys act on the running
 * simulation at once, which is what a full and direct control of gravity,
 * time, launch speed, angle and projectile mass means. The menu instead edits
 * a draft and changes nothing until Apply (srcs/game/scene.c), so a value can
 * be dialled in without the scene reacting halfway through.
 *
 * While the menu is open it owns the keyboard and the mouse, so the arrows
 * move its cursor instead of turning the camera.
 *
 * Nothing here computes physics or draws: it only moves numbers that
 * srcs/physics/ and srcs/render/ then act on.
*/

/* One-shot key detection (edge, not hold). */
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

/* Firing, spawning and the display toggles: things that happen once per
 * press, not while a key is held. */
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

/* I / K aim, J / L set the launch speed and Q / E the projectile mass. Held
 * keys change the value continuously, scaled by the frame time so the rate is
 * the same at any frame rate. */
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
}

/* Gravity and the time scale, the two knobs that act on the world itself.
 * Changing gravity wakes everything, so a resting pile falls again. */
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

/* WASD flies the eye, R / F raise and lower it, the arrows turn the view and
 * + / - change how fast it travels. The camera is free, so the whole 3D scene
 * can be inspected even though the gameplay stays on the XY plane. */
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

/* ESC: the menu takes over input and pauses the simulation; closing it
 * resumes, which is why the menu is also where Resume and Quit live. Opening
 * it refreshes the draft, so it always starts from the live values. */
static void	toggle_menu(Game *g)
{
	g->menu.open = !g->menu.open;
	g->paused = g->menu.open;
	if (g->menu.open)
		scene_sync_draft(g);
	g->hud.refresh = 1;
}

/* Values edited in the menu only move the draft; Apply is what commits them,
 * which is why nothing is pushed onto the world here. */
static void	handle_menu(Game *g, float frame_time)
{
	float	width;
	float	height;

	window_size(&g->window, &width, &height);
	menu_layout(&g->menu, width, height);
	menu_update(&g->menu, &g->window, frame_time);
	if (g->menu.pending != ACTION_NONE)
		scene_run_action(g, g->menu.pending);
}

/* One frame of input: ESC first, then either the menu or the game's own keys. */
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
	scene_apply_definitions(g);
}
