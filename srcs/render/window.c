#include "newton.h"

// The window and its OpenGL 3.3 Core context (GLFW creates it, GLAD loads the functions), and the raw input queries.

static void	framebuffer_size_callback(struct GLFWwindow *handle, int width, int height)
{
	(void)handle;
	glViewport(0, 0, width, height);
}

static void	glfw_error_callback(int code, const char *description)
{
	fprintf(stderr, "GLFW error %d: %s\n", code, description);
}

static GLADapiproc	gl_loader(const char *name)
{
	return (glfwGetProcAddress(name));
}

int	window_init(Window *window, int width, int height, const char *title)
{
	int	framebuffer_width;
	int	framebuffer_height;

	window->handle = NULL;
	window->width = width;
	window->height = height;
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return (fprintf(stderr, "Failed to initialize GLFW\n"), 0);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	window->handle = glfwCreateWindow(width, height, title, NULL, NULL);
	if (!window->handle)
		return (fprintf(stderr, "Failed to create GLFW window\n"), glfwTerminate(), 0);
	glfwMakeContextCurrent(window->handle);
	if (gladLoadGL(gl_loader) == 0)
		return (fprintf(stderr, "Failed to load OpenGL via GLAD\n"), glfwDestroyWindow(window->handle), glfwTerminate(), 0);
	glfwSwapInterval(0);
	glfwGetFramebufferSize(window->handle, &framebuffer_width, &framebuffer_height);
	glViewport(0, 0, framebuffer_width, framebuffer_height);
	glfwSetFramebufferSizeCallback(window->handle, framebuffer_size_callback);
	return (1);
}

void	window_destroy(Window *window)
{
	if (window->handle)
		glfwDestroyWindow(window->handle);
	glfwTerminate();
	window->handle = NULL;
}

int	window_should_close(const Window *window)
{
	return (glfwWindowShouldClose(window->handle));
}

void	window_poll_events(Window *window)
{
	(void)window;
	glfwPollEvents();
}

void	window_swap_buffers(Window *window)
{
	glfwSwapBuffers(window->handle);
}

void	window_size(const Window *window, float *width, float *height)
{
	int	framebuffer_width;
	int	framebuffer_height;

	glfwGetFramebufferSize(window->handle, &framebuffer_width, &framebuffer_height);
	*width = (float)framebuffer_width;
	*height = (float)framebuffer_height;
}

// Cursor in framebuffer pixels from the top-left corner, the space the overlay is laid out in.
void	window_cursor(const Window *window, float *x, float *y)
{
	double	cursor_x;
	double	cursor_y;
	int		window_width;
	int		window_height;
	int		framebuffer_width;
	int		framebuffer_height;

	glfwGetCursorPos(window->handle, &cursor_x, &cursor_y);
	glfwGetWindowSize(window->handle, &window_width, &window_height);
	glfwGetFramebufferSize(window->handle, &framebuffer_width, &framebuffer_height);
	*x = (float)cursor_x;
	*y = (float)cursor_y;
	if (window_width > 0 && window_height > 0)
	{
		*x = (float)cursor_x * (float)framebuffer_width / (float)window_width;
		*y = (float)cursor_y * (float)framebuffer_height / (float)window_height;
	}
}

int	window_mouse_down(const Window *window)
{
	return (glfwGetMouseButton(window->handle, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
}

int	window_key_down(const Window *window, int key)
{
	return (glfwGetKey(window->handle, key) == GLFW_PRESS);
}

float	window_aspect(const Window *window)
{
	int	width;
	int	height;

	glfwGetFramebufferSize(window->handle, &width, &height);
	if (height > 0)
		return ((float)width / (float)height);
	return (1.0f);
}

struct GLFWwindow	*window_handle(const Window *window)
{
	return (window->handle);
}
