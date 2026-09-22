#include "newton.h"

/*
 * What the menu buttons do. The menu edits a Draft, never the running world:
 * Apply commits it (and makes it what Reset returns to), Save writes it to the
 * object files without applying it, Reload re-reads the files into both, Reset
 * rebuilds the starting scene.
*/

/* Opening the menu starts from the live values, never stale ones. */
void	action_sync_draft(Game *g)
{
	g->draft.apple = g->appleDef;
	g->draft.block = g->blockDef;
	g->draft.ground = g->groundDef;
	g->draft.trebuchet = g->trebuchet;
	g->draft.gravityY = g->world.gravity.y;
	g->draft.timeScale = g->timeScale;
	g->menu.dirty = 0;
}

static void	apply_draft(Game *g)
{
	g->appleDef = g->draft.apple;
	g->blockDef = g->draft.block;
	g->groundDef = g->draft.ground;
	g->trebuchet.launchSpeed = g->draft.trebuchet.launchSpeed;
	g->trebuchet.launchAngle = g->draft.trebuchet.launchAngle;
	g->trebuchet.friction = g->draft.trebuchet.friction;
	g->trebuchet.elasticity = g->draft.trebuchet.elasticity;
	g->world.gravity.y = g->draft.gravityY;
	g->timeScale = g->draft.timeScale;
	g->trebuchetDef = g->trebuchet;
	scene_apply_definitions(g);
	world_wake_all(&g->world);
	g->menu.dirty = 0;
	g->hud.refresh = 1;
	printf("Menu values applied to the simulation\n");
}

/* Mass and material reach existing bodies at once; geometry only reaches new bodies, or all of
 * them after Reset. */
static void	reload_definitions(Game *g)
{
	if (!scene_load_assets(g))
	{
		fprintf(stderr, "Reload aborted: fix the object files and try again\n");
		return ;
	}
	g->trebuchet.launchSpeed = g->trebuchetDef.launchSpeed;
	g->trebuchet.launchAngle = g->trebuchetDef.launchAngle;
	g->trebuchet.friction = g->trebuchetDef.friction;
	g->trebuchet.elasticity = g->trebuchetDef.elasticity;
	g->trebuchet.color = g->trebuchetDef.color;
	scene_apply_definitions(g);
	world_wake_all(&g->world);
	action_sync_draft(g);
	g->hud.refresh = 1;
	printf("Object files reloaded: existing objects updated\n");
}

static void	reset_simulation(Game *g)
{
	g->world.gravity = vec3(0.0f, GRAVITY_Y, 0.0f);
	g->timeScale = 1.0f;
	scene_build(g);
	action_sync_draft(g);
	printf("Simulation reset\n");
}

static void	save_definitions(Game *g)
{
	int	ok;

	ok = objectdef_save(&g->draft.apple, ASSET_APPLE);
	ok = objectdef_save(&g->draft.block, ASSET_BLOCK) && ok;
	ok = objectdef_save(&g->draft.ground, ASSET_GROUND) && ok;
	ok = trebuchet_save(&g->draft.trebuchet, ASSET_TREBUCHET) && ok;
	if (ok)
		printf("Object files written (the running simulation is unchanged until Apply)\n");
}

void	action_run(Game *g, MenuAction action)
{
	if (action == ACTION_APPLY)
		apply_draft(g);
	else if (action == ACTION_RELOAD)
		reload_definitions(g);
	else if (action == ACTION_SAVE)
		save_definitions(g);
	else if (action == ACTION_RESET)
		reset_simulation(g);
	else if (action == ACTION_RESUME)
	{
		g->menu.open = 0;
		g->paused = 0;
		g->hud.refresh = 1;
	}
	else if (action == ACTION_QUIT)
		g->quit = 1;
}
