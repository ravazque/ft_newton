
#include "newton.h"

Camera	camera_default(void)
{
	Camera	c;

	c.position = vec3(0.0f, 5.0f, 18.0f);
	c.target = vec3(0.0f, 3.0f, 0.0f);
	c.up = vec3(0.0f, 1.0f, 0.0f);
	c.fovYDegrees = 60.0f;
	c.nearPlane = 0.1f;
	c.farPlane = 500.0f;
	c.win_width = WIN_WIDTH;
	c.win_height = WIN_HEIGHT;
	return (c);
}

Mat4	camera_view(const Camera *cam)
{
	return (mat4_look_at(cam->position, cam->target, cam->up));
}

Mat4	camera_projection(const Camera *cam, float aspect)
{
	return (mat4_perspective(DEG2RAD(cam->fovYDegrees), aspect, cam->nearPlane, cam->farPlane));
}
