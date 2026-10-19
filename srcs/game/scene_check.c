#include "newton.h"

// The starting scene, built aside from the files: nothing sinks or overlaps, it stands over the ground and the camera starts free.

static int	report_scene(const char *files, const char *problem, Vec3 where)
{
	fprintf(stderr, "%s: %s at (%g, %g, %g)\n", files, problem, (double)where.x, (double)where.y, (double)where.z);
	return (1);
}

static int	over_square(const RigidBody *ground, Vec3 point)
{
	return (collider_plane_covers(&ground->collider, vec3_sub(point, vec3_scale(ground->collider.normal, collider_plane_distance(&ground->collider, point)))));
}

static int	under_ground(const RigidBody *ground, Vec3 point, float clearance)
{
	return (collider_plane_distance(&ground->collider, point) < clearance && over_square(ground, point));
}

static int	all_over_ground(const World *world, int first, int last)
{
	while (first < last && over_square(&world->bodies[0], world->bodies[first].position))
		first++;
	return (first == last);
}

static int	any_in_ground(const World *world, int first, int last)
{
	while (first < last && !overlap_bodies(&world->bodies[first], &world->bodies[0]) && !under_ground(&world->bodies[0], world->bodies[first].position, 0.0f))
		first++;
	return (first < last);
}

static int	any_at_camera(const World *world, int first, int last, Vec3 camera)
{
	Vec3	normal;

	while (first < last && camera_body_overlap(camera, &world->bodies[first], &normal) <= SPAWN_TOLERANCE)
		first++;
	return (first < last);
}

static int	launch_blocked(const World *world, const Trebuchet *machine)
{
	RigidBody	apple = rigidbody_make();

	apple.position = trebuchet_spawn_point(machine);
	apple.collider = collider_sphere(machine->projectileRadius);
	return (overlap_find(world, &apple) >= 0);
}

int	scene_check_start(const ObjectDef *ground, const ObjectDef *block, const Trebuchet *trebuchet)
{
	World		world;
	Trebuchet	machine = *trebuchet;
	Vec3		camera = camera_default().position;
	Vec3		pyramid = vec3(STRUCTURE_X, 0.0f, 0.0f);
	int			first_block;
	int			problems;

	world_init(&world);
	world_add_body(&world, object_def_make_body(ground, vec3(0.0f, 0.0f, 0.0f)));
	trebuchet_build(&machine, &world);
	first_block = world.bodyCount;
	problems = 0;
	if (any_in_ground(&world, 1, first_block))
		problems += report_scene(ASSET_TREBUCHET ", " ASSET_GROUND, "the trebuchet would sink into the ground", machine.position);
	if (!all_over_ground(&world, 1, first_block))
		problems += report_scene(ASSET_TREBUCHET ", " ASSET_GROUND, "the trebuchet is not over the ground square", machine.position);
	if (structure_spawn_pyramid(&world, block, pyramid, START_PYRAMID_ROWS) != 0.0f || any_in_ground(&world, first_block, world.bodyCount))
		problems += report_scene(ASSET_BLOCK ", " ASSET_TREBUCHET ", " ASSET_GROUND, "the starting pyramid would overlap the trebuchet or sink into the ground", pyramid);
	else if (!all_over_ground(&world, first_block, world.bodyCount))
		problems += report_scene(ASSET_BLOCK ", " ASSET_GROUND, "the starting pyramid is not over the ground square", pyramid);
	else if (any_at_camera(&world, first_block, world.bodyCount, camera))
		problems += report_scene(ASSET_BLOCK, "the camera would start inside the starting pyramid", camera);
	if (launch_blocked(&world, &machine))
		problems += report_scene(ASSET_TREBUCHET ", " ASSET_APPLE, "the apple spawn point of the trebuchet is taken by another body", trebuchet_spawn_point(&machine));
	if (under_ground(&world.bodies[0], camera, CAM_RADIUS))
		problems += report_scene(ASSET_GROUND, "the camera would start under the ground", camera);
	if (any_at_camera(&world, 1, first_block, camera))
		problems += report_scene(ASSET_TREBUCHET, "the camera would start inside the trebuchet", camera);
	world_destroy(&world);
	return (problems == 0);
}
