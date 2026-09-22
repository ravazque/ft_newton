
#include "newton.h"

/*
 * A free-flying eye: a position plus a yaw/pitch look direction. Nothing ties
 * it to a fixed point, so the whole 3D world can be walked through even though
 * the gameplay itself stays on the XY plane.
*/

/* Starts on the +Z side, high enough to frame the trebuchet on the left and
 * the structures on the right, looking back along -Z and slightly down. */
Camera	camera_default(void)
{
	Camera	c;

	c.position = vec3(0.0f, 8.75f, 33.7f);
	c.up = vec3(0.0f, 1.0f, 0.0f);
	c.yaw = DEG2RAD(-90.0f);
	c.pitch = DEG2RAD(-8.0f);
	c.moveSpeed = CAM_MOVE_SPEED;
	c.fovYDegrees = 60.0f;
	c.nearPlane = 0.1f;
	c.farPlane = 500.0f;
	c.win_width = WIN_WIDTH;
	c.win_height = WIN_HEIGHT;
	return (c);
}

/* Spherical angles -> the unit direction the eye looks along. */
static Vec3	camera_forward(const Camera *cam)
{
	float	cp = cosf(cam->pitch);

	return (vec3(cp * cosf(cam->yaw), sinf(cam->pitch), cp * sinf(cam->yaw)));
}

/* The horizontal axis to the right of the view (forward x up), so strafing
 * stays level whatever the pitch is. */
static Vec3	camera_right(const Camera *cam)
{
	return (vec3_normalized(vec3_cross(camera_forward(cam), cam->up)));
}

void	camera_look(Camera *cam, float delta_yaw, float delta_pitch)
{
	cam->yaw += delta_yaw;
	cam->pitch = fminf(fmaxf(cam->pitch + delta_pitch, CAM_MIN_PITCH), CAM_MAX_PITCH);
}

/* Moves the eye by a delta given in its own frame: x to the right of the view,
 * y along world up (so rising and falling never tilt), z along the view. */
void	camera_move(Camera *cam, Vec3 local_delta)
{
	cam->position = vec3_add(cam->position, vec3_scale(camera_right(cam), local_delta.x));
	cam->position = vec3_add(cam->position, vec3_scale(cam->up, local_delta.y));
	cam->position = vec3_add(cam->position, vec3_scale(camera_forward(cam), local_delta.z));
}

void	camera_change_speed(Camera *cam, float delta)
{
	cam->moveSpeed = fminf(fmaxf(cam->moveSpeed + delta, CAM_SPEED_MIN), CAM_SPEED_MAX);
}

Mat4	camera_view(const Camera *cam)
{
	return (mat4_look_at(cam->position, vec3_add(cam->position, camera_forward(cam)), cam->up));
}

Mat4	camera_projection(const Camera *cam, float aspect)
{
	return (mat4_perspective(DEG2RAD(cam->fovYDegrees), aspect, cam->nearPlane, cam->farPlane));
}
