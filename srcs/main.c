
#include "newton.h"

/*
 * Entry point — render pipeline + shaders ready, the scene intentionally empty.
 *
 * This boots the whole render stack in C (GLFW + GLAD context, our own GLSL
 * shaders, the MVP/camera plumbing) and runs an empty frame loop: clear the
 * screen and present it. There are no objects and no environment on purpose:
 * building the world (rigid bodies, colliders, the catapult scene) and the
 * physics is the next step, and it is yours to write (see docs/info.md 12-13).
 *
 * To start the game later, replace this body with:
 *     Game g;
 *     if (game_init(&g))
 *         game_run(&g);
 *
 * Controls: ESC quits.
*/
int	main(void)
{
	Window		window;
	Renderer	renderer;
	Camera		camera;

	if (!window_init(&window, WIN_WIDTH, WIN_HEIGHT, WIN_TITLE))
		return (1);
	if (!renderer_init(&renderer))
		return (window_destroy(&window), 1);
	camera = camera_default();
	while (!window_should_close(&window))
	{
		window_poll_events(&window);
		if (glfwGetKey(window_handle(&window), GLFW_KEY_ESCAPE) == GLFW_PRESS)
			break ;
		renderer_begin_frame(&renderer, &camera, window_aspect(&window));
		window_swap_buffers(&window);
	}
	window_destroy(&window);
	return (0);
}
