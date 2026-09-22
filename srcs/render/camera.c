
#include "newton.h"

/* The default view sits on the +Z side, looking at the middle of the scene
 * (catapult on the left, structures on the right). */
Camera	camera_default(void)
{
	Camera	c;

	c.target = vec3(0.0f, 4.0f, 0.0f);
	c.up = vec3(0.0f, 1.0f, 0.0f);
	c.yaw = DEG2RAD(90.0f);
	c.pitch = DEG2RAD(8.0f);
	c.distance = 34.0f;
	c.fovYDegrees = 60.0f;
	c.nearPlane = 0.1f;
	c.farPlane = 500.0f;
	c.win_width = WIN_WIDTH;
	c.win_height = WIN_HEIGHT;
	camera_update(&c);
	return (c);
}

/* Spherical coordinates around the target -> eye position. */
void	camera_update(Camera *cam)
{
	float	cp = cosf(cam->pitch);

	cam->position = vec3_add(cam->target, vec3_scale(vec3(cp * cosf(cam->yaw), sinf(cam->pitch), cp * sinf(cam->yaw)), cam->distance));
}

void	camera_orbit(Camera *cam, float delta_yaw, float delta_pitch)
{
	cam->yaw += delta_yaw;
	cam->pitch = fminf(fmaxf(cam->pitch + delta_pitch, CAM_MIN_PITCH), CAM_MAX_PITCH);
	camera_update(cam);
}

void	camera_zoom(Camera *cam, float delta_distance)
{
	cam->distance = fminf(fmaxf(cam->distance + delta_distance, CAM_MIN_DISTANCE), CAM_MAX_DISTANCE);
	camera_update(cam);
}

Mat4	camera_view(const Camera *cam)
{
	return (mat4_look_at(cam->position, cam->target, cam->up));
}

Mat4	camera_projection(const Camera *cam, float aspect)
{
	return (mat4_perspective(DEG2RAD(cam->fovYDegrees), aspect, cam->nearPlane, cam->farPlane));
}
