#include "newton.h"

/* What one frame shows: every body with its collider's mesh, the aim bar, the collider
 * wireframes (F1), the menu overlay and the title readout. */

static void	draw_bodies(Game *g)
{
	const RigidBody	*b;
	int				i;

	i = 0;
	while (i < g->world.bodyCount)
	{
		b = &g->world.bodies[i];
		renderer_draw(&g->renderer, renderer_body_mesh(b, &g->cubeMesh, &g->sphereMesh, &g->planeMesh), renderer_body_model(b, 1.0f), b->color);
		i++;
	}
}

/* The thick bar runs from the arm tip to where apples appear (the same distance at
 * every angle); the thin tail past it grows with the launch speed. */
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

static void	draw_menu(Game *g)
{
	float	width;
	float	height;

	if (!g->menu.open)
		return ;
	window_size(&g->window, &width, &height);
	menu_layout(&g->menu, width, height);
	ui_begin(&g->ui, width, height);
	menu_draw(&g->menu, &g->ui);
	ui_end(&g->ui, &g->renderer);
}

void	draw_frame(Game *g)
{
	renderer_begin_frame(&g->renderer, &g->camera, window_aspect(&g->window));
	draw_bodies(g);
	draw_aim(g);
	debugdraw_draw_colliders(&g->debug, &g->world, &g->renderer, &g->cubeMesh, &g->sphereMesh, &g->planeMesh);
	draw_menu(g);
	hud_draw(&g->hud, &g->world, &g->trebuchet, g->timeScale, &g->window, g->paused);
}
