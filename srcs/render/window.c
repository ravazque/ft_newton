
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
	glfwSwapInterval(1);
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
