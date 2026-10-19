#include "newton.h"

// What is in the world: the object files read into definitions, the starting scene, and edits stamped on live bodies.

static void	report_missing(void)
{
	fprintf(stderr, "every %s*.csv file is required and must be valid\n", ASSET_DIR);
}

static int	load_definitions(Game *game)
{
	ObjectDef	apple;
	ObjectDef	block;
	ObjectDef	ground;
	Trebuchet	trebuchet;
	RagdollDef	ragdoll;
	ObjectDef	model;

	if (!object_def_load(&apple, ASSET_APPLE, KIND_APPLE) || !object_def_load(&block, ASSET_BLOCK, KIND_BLOCK))
		return (report_missing(), 0);
	if (!object_def_load(&ground, ASSET_GROUND, KIND_GROUND) || !object_def_load(&model, ASSET_MODELS, KIND_MODEL))
		return (report_missing(), 0);
	if (!trebuchet_load(&trebuchet, ASSET_TREBUCHET, &apple) || !ragdoll_load(&ragdoll, ASSET_RAGDOLL))
		return (report_missing(), 0);
	if (ground.shape != SHAPE_PLANE || apple.shape != SHAPE_SPHERE || block.shape != SHAPE_BOX || model.shape != SHAPE_MESH)
		return (fprintf(stderr, "%s must be a plane, %s a sphere, %s a box and %s a mesh\n", ASSET_GROUND, ASSET_APPLE, ASSET_BLOCK, ASSET_MODELS), 0);
	if (!scene_check_start(&ground, &block, &trebuchet))
		return (0);
	game->appleDef = apple;
	game->blockDef = block;
	game->groundDef = ground;
	game->trebuchetDef = trebuchet;
	game->ragdollDef = ragdoll;
	game->modelDef = model;
	return (1);
}

int	scene_load_assets(Game *game)
{
	int	first_load = (game->modelDef.shape != SHAPE_MESH);

	if (!load_definitions(game))
		return (0);
	if (first_load)
		return (model_library_load(&game->models, game->modelDef.modelSize));
	return (1);
}

void	scene_build(Game *game)
{
	world_clear(&game->world);
	game->trebuchet = game->trebuchetDef;
	world_add_body(&game->world, object_def_make_body(&game->groundDef, vec3(0.0f, 0.0f, 0.0f)));
	trebuchet_build(&game->trebuchet, &game->world);
	structure_spawn_pyramid(&game->world, &game->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), START_PYRAMID_ROWS);
	game->paused = 0;
	game->hud.refresh = 1;
}

static void	stamp_material(RigidBody *body, float friction, float elasticity, Vec3 color)
{
	body->friction = friction;
	body->elasticity = elasticity;
	body->color = color;
}

static void	stamp_body(Game *game, RigidBody *body)
{
	const ObjectDef	*def = NULL;

	if (body->kind == KIND_APPLE)
		def = &game->appleDef;
	else if (body->kind == KIND_BLOCK)
		def = &game->blockDef;
	else if (body->kind == KIND_GROUND)
		def = &game->groundDef;
	if (def)
	{
		stamp_material(body, def->friction, def->elasticity, def->color);
		body->rollingResistance = def->rollingResistance;
		if (body->inverseMass > 0.0f)
			rigidbody_set_mass(body, def->mass);
	}
	else if (body->kind == KIND_TREBUCHET)
		stamp_material(body, game->trebuchet.friction, game->trebuchet.elasticity, game->trebuchet.color);
	else if (body->kind == KIND_RAGDOLL)
	{
		stamp_material(body, game->ragdollDef.friction, game->ragdollDef.elasticity, game->ragdollDef.color);
		rigidbody_set_mass(body, game->ragdollDef.mass * ragdoll_mass_share(body->ragdollPart));
	}
	else if (body->kind == KIND_MODEL)
	{
		stamp_material(body, game->modelDef.friction, game->modelDef.elasticity, game->modelDef.color);
		rigidbody_set_mass(body, game->modelDef.density * body->collider.mesh->volume);
	}
}

void	scene_apply_definitions(Game *game)
{
	int	i;

	i = 0;
	while (i < game->world.bodyCount)
	{
		stamp_body(game, &game->world.bodies[i]);
		i++;
	}
	game->trebuchet.projectileMass = game->appleDef.mass;
	game->trebuchet.projectileRadius = game->appleDef.radius;
	game->trebuchet.spawnDistance = trebuchet_spawn_distance(&game->trebuchet);
}
