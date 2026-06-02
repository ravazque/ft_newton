/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   00:00:00 by: ravazque <ravazque@student.42madrid.com>   +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2026/06/02 00:00:00 by ravazque                   #+#    #+#             */
/*   Updated: 2026/06/02 00:00:00 by ravazque                  ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#define GLFW_INCLUDE_NONE

#include "newton.h"

/*
 * Milestone 1: a window with a shaded, rotating cube.
 *
 * This proves the render pipeline end to end in C: GLFW + GLAD context, a Mesh
 * on the GPU (VBO/VAO/EBO), our own GLSL shaders, the MVP matrices and a 3D
 * camera. The physics modules come next (they are scaffolded as [TODO] stubs).
 *
 * When you start building the game, replace this body with: game_init(&game)
 * and game_run(&game).
 *
 * Controls: ESC quits, TAB toggles wireframe.
*/
int	main(void)
{
	Window		window;
	Renderer	renderer;
	Camera		camera;
	Mesh		cube;
	int			wireframe;
	int			tab_was_down;
	int			tab_down;
	float		t;
	Quat		spin;
	Mat4		model;

	if (!window_init(&window, WIN_WIDTH, WIN_HEIGHT, WIN_TITLE " - milestone 1"))
		return (1);
	if (!renderer_init(&renderer))
		return (window_destroy(&window), 1);
	camera = camera_default();
	camera.position = vec3(3.5f, 2.5f, 5.0f);
	camera.target = vec3(0.0f, 0.0f, 0.0f);
	cube = mesh_cube();
	wireframe = 0;
	tab_was_down = 0;
	while (!window_should_close(&window))
	{
		window_poll_events(&window);
		if (glfwGetKey(window_handle(&window), GLFW_KEY_ESCAPE) == GLFW_PRESS)
			break ;
		tab_down = (glfwGetKey(window_handle(&window), GLFW_KEY_TAB)
				== GLFW_PRESS);
		if (tab_down && !tab_was_down)
		{
			wireframe = !wireframe;
			renderer_set_wireframe(&renderer, wireframe);
		}
		tab_was_down = tab_down;
		t = (float)glfwGetTime();
		spin = quat_from_axis_angle(
				vec3_normalized(vec3(0.3f, 1.0f, 0.2f)), t);
		model = mat4_transform(vec3(0.0f, 0.0f, 0.0f), spin, vec3(1.0f, 1.0f, 1.0f));
		renderer_begin_frame(&renderer, &camera, window_aspect(&window));
		renderer_draw(&renderer, &cube, model, vec3(0.90f, 0.45f, 0.20f));
		window_swap_buffers(&window);
	}
	mesh_release(&cube);
	window_destroy(&window);
	return (0);
}
