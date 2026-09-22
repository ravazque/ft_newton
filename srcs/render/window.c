
#include "newton.h"

static void	framebuffer_size_callback(struct GLFWwindow *win, int width, int height)
{
	(void)win;
	glViewport(0, 0, width, height);
}

static void	glfw_error_callback(int code, const char *description)
{
	fprintf(stderr, "GLFW error %d: %s\n", code, description);
}

/* GLAD expects a loader returning GLADapiproc (a function pointer). Matching
 * the signature exactly avoids a cast that -Werror would reject. GLFW's
 * GLFWglproc and GLAD's GLADapiproc are both void(*)(void), so this is clean. */
static GLADapiproc	gl_loader(const char *name)
{
	return (glfwGetProcAddress(name));
}

int	window_init(Window *win, int width, int height, const char *title)
{
	int	fb_w;
	int	fb_h;

	win->handle = NULL;
	win->width = width;
	win->height = height;
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return (fprintf(stderr, "Failed to initialize GLFW\n"), 0);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	win->handle = glfwCreateWindow(width, height, title, NULL, NULL);
	if (!win->handle)
		return (fprintf(stderr, "Failed to create GLFW window\n"), glfwTerminate(), 0);
	glfwMakeContextCurrent(win->handle);
	if (gladLoadGL(gl_loader) == 0)
		return (fprintf(stderr, "Failed to load OpenGL via GLAD\n"), glfwDestroyWindow(win->handle), glfwTerminate(), 0);
	glfwSwapInterval(0);
	glfwGetFramebufferSize(win->handle, &fb_w, &fb_h);
	glViewport(0, 0, fb_w, fb_h);
	glfwSetFramebufferSizeCallback(win->handle, framebuffer_size_callback);
	return (1);
}

void	window_destroy(Window *win)
{
	if (win->handle)
		glfwDestroyWindow(win->handle);
	glfwTerminate();
	win->handle = NULL;
}

int	window_should_close(const Window *win)
{
	return (glfwWindowShouldClose(win->handle));
}

void	window_poll_events(Window *win)
{
	(void)win;
	glfwPollEvents();
}

void	window_swap_buffers(Window *win)
{
	glfwSwapBuffers(win->handle);
}

void	window_size(const Window *win, float *width, float *height)
{
	int	w;
	int	h;

	glfwGetFramebufferSize(win->handle, &w, &h);
	*width = (float)w;
	*height = (float)h;
}

/* Cursor position in the same pixel space the overlay is laid out in:
 * origin at the top-left corner, y growing downward. */
void	window_cursor(const Window *win, float *x, float *y)
{
	double	cx;
	double	cy;
	int		win_w;
	int		win_h;
	int		fb_w;
	int		fb_h;

	glfwGetCursorPos(win->handle, &cx, &cy);
	glfwGetWindowSize(win->handle, &win_w, &win_h);
	glfwGetFramebufferSize(win->handle, &fb_w, &fb_h);
	*x = (float)cx;
	*y = (float)cy;
	if (win_w > 0 && win_h > 0)
	{
		*x = (float)cx * (float)fb_w / (float)win_w;
		*y = (float)cy * (float)fb_h / (float)win_h;
	}
}

int	window_mouse_down(const Window *win)
{
	return (glfwGetMouseButton(win->handle, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
}

int	window_key_down(const Window *win, int key)
{
	return (glfwGetKey(win->handle, key) == GLFW_PRESS);
}

float	window_aspect(const Window *win)
{
	int	w;
	int	h;

	glfwGetFramebufferSize(win->handle, &w, &h);
	if (h > 0)
		return ((float)w / (float)h);
	return (1.0f);
}

struct GLFWwindow	*window_handle(const Window *win)
{
	return (win->handle);
}
