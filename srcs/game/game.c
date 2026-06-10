
#include "newton.h"

/*
 * Scene + main loop. scene_default() is the place to compose your scene:
 * everything it needs (spawners, catapult, meshes, renderer) is ready, so a
 * body added here shows up on screen immediately. Movement only starts once
 * the physics (integrator + resolver) is implemented.
 */

static void	scene_default(Game *g)
{
	RigidBody	ball;
	RigidBody	ground;

	world_clear(&g->world);
	ground = rb_make();
	ground.collider = collider_plane(vec3(0.0f, 1.0f, 0.0f), 0.0f);
	rb_make_static(&ground);
	world_add_body(&g->world, ground);
	ball = rb_make();
	ball.position = vec3(0.0f, 5.0f, 0.0f);
	ball.collider = collider_sphere(1.5f);
	rb_set_mass(&ball, 2.0f);
	world_add_body(&g->world, ball);
}

int	game_init(Game *g, int argc, char **argv)
{
	g->camera = camera_default();
	check_input(argc, argv, &g->camera);
	if (!window_init(&g->window, g->camera.win_width, g->camera.win_height, WIN_TITLE))
		return (0);
	if (!renderer_init(&g->renderer))
		return (window_destroy(&g->window), 0);
	world_init(&g->world);
	g->catapult = catapult_default();
	g->hud = hud_default();
	g->debug.enabled = 0;
	g->fixedDt = FIXED_DT;
	g->timeScale = 1.0f;
	g->running = 1;
	g->planeMesh = mesh_plane(20.0f);
	g->sphereMesh = mesh_sphere(32);
	g->cubeMesh = mesh_cube();
	scene_default(g);
	return (1);
}

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

static void	rotate_launch_direction(Catapult *c, float radians)
{
	float	angle;

	angle = atan2f(c->launchDirection.y, c->launchDirection.x) + radians;
	c->launchDirection = vec3(cosf(angle), sinf(angle), 0.0f);
}

/*
 * Live controls (subject: time, gravity and projectile speed / direction /
 * mass must be tunable without recompiling):
 *   SPACE        fire an apple
 *   1 / 2 / 3    spawn wall / pyramid / tower
 *   R            reset the scene
 *   F1           debug wireframe on/off
 *   H            HUD on/off
 *   W / S        launch speed +/-
 *   A / D        launch angle
 *   Q / E        projectile mass -/+
 *   G / B        gravity +/-
 *   T / Y        time scale -/+
 */
static void	handle_input(Game *g, float frameTime)
{
	if (key_pressed(g, GLFW_KEY_SPACE))
		catapult_fire(&g->catapult, &g->world);
	if (key_pressed(g, GLFW_KEY_1))
		structure_spawn_wall(&g->world, vec3(8.0f, 0.0f, 0.0f), 6, 6);
	if (key_pressed(g, GLFW_KEY_2))
		structure_spawn_pyramid(&g->world, vec3(8.0f, 0.0f, 0.0f), 7);
	if (key_pressed(g, GLFW_KEY_3))
		structure_spawn_tower(&g->world, vec3(8.0f, 0.0f, 0.0f), 8);
	if (key_pressed(g, GLFW_KEY_R))
		scene_default(g);
	if (key_pressed(g, GLFW_KEY_F1))
		g->debug.enabled = !g->debug.enabled;
	if (key_pressed(g, GLFW_KEY_H))
		g->hud.visible = !g->hud.visible;
	if (key_down(g, GLFW_KEY_W))
		g->catapult.launchSpeed += 10.0f * frameTime;
	if (key_down(g, GLFW_KEY_S))
		g->catapult.launchSpeed = fmaxf(1.0f, g->catapult.launchSpeed - 10.0f * frameTime);
	if (key_down(g, GLFW_KEY_A))
		rotate_launch_direction(&g->catapult, DEG2RAD(60.0f) * frameTime);
	if (key_down(g, GLFW_KEY_D))
		rotate_launch_direction(&g->catapult, -DEG2RAD(60.0f) * frameTime);
	if (key_down(g, GLFW_KEY_E))
		g->catapult.projectileMass += 1.0f * frameTime;
	if (key_down(g, GLFW_KEY_Q))
		g->catapult.projectileMass = fmaxf(0.1f, g->catapult.projectileMass - 1.0f * frameTime);
	if (key_down(g, GLFW_KEY_G))
		g->world.gravity.y += 5.0f * frameTime;
	if (key_down(g, GLFW_KEY_B))
		g->world.gravity.y -= 5.0f * frameTime;
	if (key_down(g, GLFW_KEY_Y))
		g->timeScale = fminf(4.0f, g->timeScale + 1.0f * frameTime);
	if (key_down(g, GLFW_KEY_T))
		g->timeScale = fmaxf(0.0f, g->timeScale - 1.0f * frameTime);
}

static void	draw_bodies(Game *g)
{
	int	i;

	i = 0;
	while (i < g->world.bodyCount)
	{
		RigidBody	*b = &g->world.bodies[i];

		if (b->collider.type == SHAPE_SPHERE)
		{
			float	s = b->collider.radius * 2.0f;

			renderer_draw(&g->renderer, &g->sphereMesh, mat4_transform(b->position, b->orientation, vec3(s, s, s)), vec3(0.85f, 0.25f, 0.25f));
		}
		else if (b->collider.type == SHAPE_PLANE)
			renderer_draw(&g->renderer, &g->planeMesh, mat4_transform(b->position, b->orientation, vec3(1, 1, 1)), vec3(0.30f, 0.55f, 0.35f));
		else
			renderer_draw(&g->renderer, &g->cubeMesh, mat4_transform(b->position, b->orientation, vec3_scale(b->collider.halfExtents, 2.0f)), vec3(0.40f, 0.60f, 0.90f));
		i++;
	}
}

void	game_run(Game *g)
{
	double	prev = glfwGetTime();
	float	accumulator = 0.0f;

	while (!window_should_close(&g->window))
	{
		double	now;
		float	frameTime;

		window_poll_events(&g->window);
		if (glfwGetKey(window_handle(&g->window), GLFW_KEY_ESCAPE) == GLFW_PRESS)
			break ;
		now = glfwGetTime();
		frameTime = (float)(now - prev);
		prev = now;
		if (frameTime > 0.25f)
			frameTime = 0.25f;
		handle_input(g, frameTime);
		accumulator += frameTime;
		while (accumulator >= g->fixedDt)
		{
			world_step(&g->world, g->fixedDt * g->timeScale);
			accumulator -= g->fixedDt;
		}
		hud_update(&g->hud, frameTime);
		renderer_begin_frame(&g->renderer, &g->camera, window_aspect(&g->window));
		draw_bodies(g);
		debugdraw_draw_colliders(&g->debug, &g->world, &g->renderer, &g->cubeMesh, &g->sphereMesh, &g->planeMesh);
		hud_draw(&g->hud, &g->world, &g->window);
		window_swap_buffers(&g->window);
	}
}
