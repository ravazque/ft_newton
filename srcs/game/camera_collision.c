#include "newton.h"

// The camera is a sphere of CAM_RADIUS pushed out of every body; a centre swallowed by a mesh leaves through its closest surface point [F30].

static int	within_reach(Vec3 center, const RigidBody *body)
{
	Vec3	bounds_min;
	Vec3	bounds_max;

	collider_bounds(body, &bounds_min, &bounds_max);
	if (center.x + CAM_RADIUS < bounds_min.x || center.x - CAM_RADIUS > bounds_max.x)
		return (0);
	if (center.y + CAM_RADIUS < bounds_min.y || center.y - CAM_RADIUS > bounds_max.y)
		return (0);
	return (center.z + CAM_RADIUS >= bounds_min.z && center.z - CAM_RADIUS <= bounds_max.z);
}

static float	swallowed_depth(Vec3 center, const RigidBody *body, Vec3 *normal)
{
	const MeshShape	*shape = body->collider.mesh;
	Vec3			local;
	MeshHit			hit;

	if (body->collider.type != SHAPE_MESH)
		return (0.0f);
	local = mesh_to_local(body, center);
	if (!mesh_closest_point(shape, local, shape->size, &hit) || !hit.inside)
		return (0.0f);
	*normal = quat_rotate(body->orientation, vec3_scale(vec3_sub(local, hit.point), 1.0f / fmaxf(hit.distance, 1e-6f)));
	return (hit.distance + CAM_RADIUS);
}

float	camera_body_overlap(Vec3 center, const RigidBody *body, Vec3 *normal)
{
	Contact		found[MAX_CONTACTS_PER_PAIR];
	RigidBody	eye = rigidbody_make();
	float		depth = 0.0f;
	int			count;

	if (!within_reach(center, body))
		return (0.0f);
	eye.position = center;
	eye.collider = collider_sphere(CAM_RADIUS);
	count = narrowphase_pair(&eye, body, found);
	while (count > 0)
	{
		count--;
		if (found[count].penetration > depth)
		{
			depth = found[count].penetration;
			*normal = found[count].normal;
		}
	}
	if (depth == 0.0f)
		depth = swallowed_depth(center, body, normal);
	return (depth);
}

static float	deepest_overlap(const World *world, Vec3 center)
{
	Vec3	normal;
	float	deepest = 0.0f;
	int		i;

	i = 0;
	while (i < world->bodyCount)
	{
		deepest = fmaxf(deepest, camera_body_overlap(center, &world->bodies[i], &normal));
		i++;
	}
	return (deepest);
}

static void	push_out(Camera *camera, const World *world)
{
	Vec3	normal;
	float	depth;
	int		pass;
	int		i;

	pass = 0;
	while (pass < CAM_PUSH_PASSES)
	{
		i = 0;
		while (i < world->bodyCount)
		{
			depth = camera_body_overlap(camera->position, &world->bodies[i], &normal);
			if (depth > 0.0f)
				camera->position = vec3_sub(camera->position, vec3_scale(normal, depth));
			i++;
		}
		pass++;
	}
}

static void	escape_pinned(Camera *camera, const World *world)
{
	const Vec3	directions[CAM_ESCAPE_DIRECTIONS] = {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}, {-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
	Vec3		candidate;
	int			distance;
	int			i;

	distance = 1;
	while (distance <= CAM_ESCAPE_MAX)
	{
		i = 0;
		while (i < CAM_ESCAPE_DIRECTIONS)
		{
			candidate = vec3_add(camera->position, vec3_scale(directions[i], CAM_STEP * (float)distance));
			if (deepest_overlap(world, candidate) <= SPAWN_CLEARANCE)
			{
				camera->position = candidate;
				return ;
			}
			i++;
		}
		distance++;
	}
}

void	camera_keep_clear(Camera *camera, const World *world)
{
	push_out(camera, world);
	if (deepest_overlap(world, camera->position) > SPAWN_CLEARANCE)
		escape_pinned(camera, world);
}

// Steps of CAM_STEP at most; a step that ends deeper inside something than the camera started (a gap narrower than it) is undone.
void	camera_fly(Camera *camera, const World *world, Vec3 local_delta)
{
	Vec3	delta = camera_world_delta(camera, local_delta);
	Vec3	previous;
	float	allowed;
	int		steps = (int)ceilf(vec3_length(delta) / CAM_STEP);
	int		step;

	if (steps == 0)
		return ;
	allowed = fmaxf(deepest_overlap(world, camera->position), SPAWN_CLEARANCE);
	step = 0;
	while (step < steps)
	{
		previous = camera->position;
		camera->position = vec3_add(camera->position, vec3_scale(delta, 1.0f / (float)steps));
		push_out(camera, world);
		if (deepest_overlap(world, camera->position) > allowed)
		{
			camera->position = previous;
			return ;
		}
		step++;
	}
}
