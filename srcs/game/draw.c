#include "newton.h"

// One frame: every body with its collider's mesh, the aim bar, the debug display (F1), the menu and the title readout.

static void	draw_bodies(Game *game)
{
	const RigidBody	*body;
	int				i;

	i = 0;
	while (i < game->world.bodyCount)
	{
		body = &game->world.bodies[i];
		renderer_draw(&game->renderer, renderer_body_mesh(body, &game->meshes), renderer_body_model(body, 1.0f), body->color);
		i++;
	}
}

// A thick bar from the arm tip to where apples appear, then a thin tail that grows with the launch speed.
static void	draw_aim(Game *game)
{
	const Trebuchet	*trebuchet = &game->trebuchet;
	Vec3			direction = trebuchet_direction(trebuchet);
	Quat			rotation = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(trebuchet->launchAngle));
	float			bar_length = trebuchet->spawnDistance;
	float			tail_length = trebuchet->launchSpeed * AIM_LENGTH_PER_MPS;
	Vec3			bar_center = vec3_add(trebuchet->launchPoint, vec3_scale(direction, bar_length * 0.5f));
	Vec3			tail_center = vec3_add(trebuchet_spawn_point(trebuchet), vec3_scale(direction, tail_length * 0.5f));

	renderer_set_wireframe(&game->renderer, 1);
	renderer_draw_flat(&game->renderer, &game->meshes.cube, mat4_transform(bar_center, rotation, vec3(bar_length, AIM_BAR_THICKNESS, AIM_BAR_THICKNESS)), vec3(1.0f, 0.90f, 0.20f));
	renderer_draw_flat(&game->renderer, &game->meshes.cube, mat4_transform(tail_center, rotation, vec3(tail_length, AIM_TAIL_THICKNESS, AIM_TAIL_THICKNESS)), vec3(1.0f, 0.62f, 0.12f));
	renderer_set_wireframe(&game->renderer, 0);
}

static void	draw_menu(Game *game)
{
	float	width;
	float	height;

	if (!game->menu.open)
		return ;
	window_size(&game->window, &width, &height);
	menu_layout(&game->menu, width, height);
	ui_begin(&game->ui, width, height);
	menu_draw(&game->menu, &game->ui);
	ui_end(&game->ui, &game->renderer);
}

void	draw_frame(Game *game)
{
	renderer_begin_frame(&game->renderer, &game->camera, window_aspect(&game->window));
	draw_bodies(game);
	draw_aim(game);
	debugdraw_draw(&game->debug, &game->world, &game->renderer, &game->meshes);
	draw_menu(game);
	hud_draw(&game->hud, game);
}
