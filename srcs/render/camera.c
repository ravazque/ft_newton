#include "newton.h"

// Free-flying camera: a position and a yaw and pitch, turned into the view and projection matrices.

Camera	camera_default(void)
{
	Camera	camera;

	camera.position = vec3(0.0f, 8.75f, 33.7f);
	camera.up = vec3(0.0f, 1.0f, 0.0f);
	camera.yaw = DEG2RAD(-90.0f);
	camera.pitch = DEG2RAD(-8.0f);
	camera.moveSpeed = CAM_MOVE_SPEED;
	camera.fovYDegrees = 60.0f;
	camera.nearPlane = 0.1f;
	camera.farPlane = 500.0f;
	camera.windowWidth = WIN_WIDTH;
	camera.windowHeight = WIN_HEIGHT;
	return (camera);
}

static Vec3	camera_forward(const Camera *camera)
{
	float	cos_pitch = cosf(camera->pitch);

	return (vec3(cos_pitch * cosf(camera->yaw), sinf(camera->pitch), cos_pitch * sinf(camera->yaw)));
}

// Horizontal, so strafing stays level whatever the pitch.
static Vec3	camera_right(const Camera *camera)
{
	return (vec3_normalized(vec3_cross(camera_forward(camera), camera->up)));
}

void	camera_look(Camera *camera, float delta_yaw, float delta_pitch)
{
	camera->yaw += delta_yaw;
	camera->pitch = fminf(fmaxf(camera->pitch + delta_pitch, CAM_MIN_PITCH), CAM_MAX_PITCH);
}

Vec3	camera_world_delta(const Camera *camera, Vec3 local_delta)
{
	Vec3	delta = vec3_scale(camera_right(camera), local_delta.x);

	delta = vec3_add(delta, vec3_scale(camera->up, local_delta.y));
	return (vec3_add(delta, vec3_scale(camera_forward(camera), local_delta.z)));
}

void	camera_change_speed(Camera *camera, float delta)
{
	camera->moveSpeed = fminf(fmaxf(camera->moveSpeed + delta, CAM_SPEED_MIN), CAM_SPEED_MAX);
}

Mat4	camera_view(const Camera *camera)
{
	return (mat4_look_at(camera->position, vec3_add(camera->position, camera_forward(camera)), camera->up));
}

Mat4	camera_projection(const Camera *camera, float aspect)
{
	return (mat4_perspective(DEG2RAD(camera->fovYDegrees), aspect, camera->nearPlane, camera->farPlane));
}
