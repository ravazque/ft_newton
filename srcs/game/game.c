
#include "newton.h"

/*
 * THE APPLICATION: start up, run the loop, draw a frame, shut down.
 *
 * This file wires the modules together and owns the frame. It deliberately
 * holds no physics, no maths and no OpenGL call of its own:
 *
 *   maths (vectors, matrices, quaternions)  -> srcs/math/
 *   simulation (integration, sleeping)      -> srcs/physics/
 *   collision (broad, narrow, response)     -> srcs/collision/
 *   drawing (GL, shaders, meshes, overlay)  -> srcs/render/
 *   what exists in the world, and the menu  -> srcs/game/scene.c
 *   the keyboard                            -> srcs/game/input.c
 *
 * The loop runs the simulation at a fixed step and the display at whatever
 * rate FPS_CAP allows, so the physics never depends on the frame rate.
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
	scene_build_menu(g);
	g->hud = hud_default();
	g->debug.enabled = 0;
	g->fixedDt = FIXED_DT;
	g->timeScale = 1.0f;
	g->planeMesh = mesh_plane(g->groundDef.extent);
	g->sphereMesh = mesh_sphere(32);
	g->cubeMesh = mesh_cube();
	scene_build(g);
	scene_sync_draft(g);
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

/* ========================================================================== */
/*  DRAWING ONE FRAME                                                         */
/* ========================================================================== */

/* Each body is drawn with the mesh of its collider shape and its own color;
 * the plane's transform comes from its equation, not its body position. */
static void	draw_bodies(Game *g)
{
	const RigidBody	*b;
	Mat4			model;
	float			s;
	int				i;

	i = 0;
	while (i < g->world.bodyCount)
	{
		b = &g->world.bodies[i];
		if (b->collider.type == SHAPE_SPHERE)
		{
			s = b->collider.radius * 2.0f;
			model = mat4_transform(b->position, b->orientation, vec3(s, s, s));
			renderer_draw(&g->renderer, &g->sphereMesh, model, b->color);
		}
		else if (b->collider.type == SHAPE_PLANE)
		{
			model = mat4_transform(collider_plane_origin(&b->collider), collider_plane_rotation(&b->collider), vec3(1.0f, 1.0f, 1.0f));
			renderer_draw(&g->renderer, &g->planeMesh, model, b->color);
		}
		else
		{
			model = mat4_transform(b->position, b->orientation, vec3_scale(b->collider.halfExtents, 2.0f));
			renderer_draw(&g->renderer, &g->cubeMesh, model, b->color);
		}
		i++;
	}
}

/*
 * The firing line, in two pieces.
 *
 * The thick bar runs from the arm tip to the exact point an apple appears at.
 * That distance is the same at every launch angle, so the apple always leaves
 * from the bar's tip instead of creeping forward as the shot is aimed lower.
 * The thin line carries on past it and is the speed gauge: its length is the
 * launch speed, so aiming and power read as one picture.
 */
static void	draw_aim(Game *g)
{
	Vec3	direction = trebuchet_direction(&g->trebuchet);
	Quat	rot = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(g->trebuchet.launchAngle));
	float	barrel = g->trebuchet.spawnDistance;
	float	tail = g->trebuchet.launchSpeed * AIM_LENGTH_PER_MPS;
	Vec3	barrel_center = vec3_add(g->trebuchet.launchPoint, vec3_scale(direction, barrel * 0.5f));
	Vec3	tail_center = vec3_add(trebuchet_spawn_point(&g->trebuchet), vec3_scale(direction, tail * 0.5f));

	renderer_set_wireframe(&g->renderer, 1);
	renderer_draw_flat(&g->renderer, &g->cubeMesh, mat4_transform(barrel_center, rot, vec3(barrel, AIM_BAR_THICKNESS, AIM_BAR_THICKNESS)), vec3(1.0f, 0.90f, 0.20f));
	renderer_draw_flat(&g->renderer, &g->cubeMesh, mat4_transform(tail_center, rot, vec3(tail, AIM_TAIL_THICKNESS, AIM_TAIL_THICKNESS)), vec3(1.0f, 0.62f, 0.12f));
	renderer_set_wireframe(&g->renderer, 0);
}

static void	draw_overlay(Game *g)
{
	float	width;
	float	height;

	if (!g->menu.open)
		return ;
	window_size(&g->window, &width, &height);
	ui_begin(&g->ui, width, height);
	menu_draw(&g->menu, &g->ui);
	ui_end(&g->ui, &g->renderer);
}

static void	draw_frame(Game *g)
{
	renderer_begin_frame(&g->renderer, &g->camera, window_aspect(&g->window));
	draw_bodies(g);
	draw_aim(g);
	debugdraw_draw_colliders(&g->debug, &g->world, &g->renderer, &g->cubeMesh, &g->sphereMesh, &g->planeMesh);
	draw_overlay(g);
	hud_draw(&g->hud, &g->world, &g->trebuchet, g->timeScale, &g->window, g->paused);
}

/* ========================================================================== */
/*  THE LOOP                                                                  */
/* ========================================================================== */

/* The time scale stretches the simulated time of a frame, never the step:
 * at 4x the world takes four times as many 1/120 s steps, so a fast apple
 * still cannot skip through a block. When a heavy scene cannot keep up, the
 * leftover time is dropped (the game slows down) instead of piling up. */
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

/* Sleeps away the rest of the frame so the loop runs at FPS_CAP: the physics
 * keeps its fixed step whatever the display does, and no work goes into
 * frames nobody would see. */
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
		frame_time = (float)(now - prev);
		prev = now;
		if (frame_time > 0.25f)
			frame_time = 0.25f;
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
