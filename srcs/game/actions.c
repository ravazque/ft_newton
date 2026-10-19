#include "newton.h"

// The menu buttons: Apply commits the draft, Save writes it to the files, Reload reads them back, Reset rebuilds the scene.

void	action_sync_draft(Game *game)
{
	game->draft.apple = game->appleDef;
	game->draft.block = game->blockDef;
	game->draft.ground = game->groundDef;
	game->draft.trebuchet = game->trebuchet;
	game->draft.ragdoll = game->ragdollDef;
	game->draft.model = game->modelDef;
	game->draft.gravityY = game->world.gravity.y;
	game->draft.timeScale = game->timeScale;
	game->menu.dirty = 0;
}

static void	resize_models(Game *game)
{
	int	i;

	if (game->models.count == 0 || game->models.shapes[0].size == game->modelDef.modelSize)
		return ;
	model_resize_all(&game->world, &game->models, game->modelDef.modelSize);
	i = 0;
	while (i < game->models.count)
	{
		mesh_release(&game->meshes.models[i]);
		game->meshes.models[i] = mesh_from_shape(&game->models.shapes[i]);
		i++;
	}
	printf("Models resized to %g m\n", (double)game->modelDef.modelSize);
}

static void	apply_draft(Game *game)
{
	game->appleDef = game->draft.apple;
	game->blockDef = game->draft.block;
	game->groundDef = game->draft.ground;
	game->ragdollDef = game->draft.ragdoll;
	game->modelDef = game->draft.model;
	game->trebuchet.launchSpeed = game->draft.trebuchet.launchSpeed;
	game->trebuchet.launchAngle = game->draft.trebuchet.launchAngle;
	game->trebuchet.friction = game->draft.trebuchet.friction;
	game->trebuchet.elasticity = game->draft.trebuchet.elasticity;
	game->world.gravity.y = game->draft.gravityY;
	game->timeScale = game->draft.timeScale;
	game->trebuchetDef = game->trebuchet;
	resize_models(game);
	scene_apply_definitions(game);
	world_wake_all(&game->world);
	game->menu.dirty = 0;
	game->hud.refresh = 1;
	printf("Menu values applied to the simulation\n");
}

// Mass, material and the model size reach the existing bodies at once; other geometry reaches new bodies, or all after a reset.
static void	reload_definitions(Game *game)
{
	if (!scene_load_assets(game))
	{
		fprintf(stderr, "Reload aborted: the object files above need fixing\n");
		return ;
	}
	game->trebuchet.launchSpeed = game->trebuchetDef.launchSpeed;
	game->trebuchet.launchAngle = game->trebuchetDef.launchAngle;
	game->trebuchet.friction = game->trebuchetDef.friction;
	game->trebuchet.elasticity = game->trebuchetDef.elasticity;
	game->trebuchet.color = game->trebuchetDef.color;
	resize_models(game);
	scene_apply_definitions(game);
	world_wake_all(&game->world);
	action_sync_draft(game);
	game->hud.refresh = 1;
	printf("Object files reloaded: existing objects updated\n");
}

static void	reset_simulation(Game *game)
{
	game->world.gravity = vec3(0.0f, GRAVITY_Y, 0.0f);
	game->timeScale = 1.0f;
	scene_build(game);
	action_sync_draft(game);
	printf("Simulation reset\n");
}

static void	save_definitions(Game *game)
{
	int	ok;

	ok = object_def_save(&game->draft.apple, ASSET_APPLE);
	ok = object_def_save(&game->draft.block, ASSET_BLOCK) && ok;
	ok = object_def_save(&game->draft.ground, ASSET_GROUND) && ok;
	ok = trebuchet_save(&game->draft.trebuchet, ASSET_TREBUCHET) && ok;
	ok = ragdoll_save(&game->draft.ragdoll, ASSET_RAGDOLL) && ok;
	ok = object_def_save(&game->draft.model, ASSET_MODELS) && ok;
	if (ok)
		printf("Object files written (the running simulation is unchanged until Apply)\n");
}

static void	open_page(Game *game, MenuPage page)
{
	game->menu.page = page;
	menu_rows_build(game);
}

static void	close_menu(Game *game)
{
	open_page(game, MENU_PAGE_MAIN);
	game->menu.open = 0;
	game->paused = 0;
	game->hud.refresh = 1;
}

static void	place_model(Game *game, int model)
{
	if (model < 0 || model >= game->models.count)
		return ;
	game->models.selected = model;
	close_menu(game);
	model_spawn(&game->world, &game->modelDef, &game->models.shapes[model], vec3(TARGET_X, 0.0f, 0.0f));
}

void	action_run(Game *game, MenuAction action)
{
	if (action == ACTION_APPLY)
		apply_draft(game);
	else if (action == ACTION_RELOAD)
		reload_definitions(game);
	else if (action == ACTION_SAVE)
		save_definitions(game);
	else if (action == ACTION_RESET)
		reset_simulation(game);
	else if (action == ACTION_RESUME)
		close_menu(game);
	else if (action == ACTION_QUIT)
		game->quit = 1;
	else if (action == ACTION_MODELS)
		open_page(game, MENU_PAGE_MODELS);
	else if (action == ACTION_BACK)
		open_page(game, MENU_PAGE_MAIN);
	else if (action == ACTION_PLACE_MODEL)
		place_model(game, game->menu.pendingModel);
}
