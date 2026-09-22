
#include "newton.h"

/*
 * THE SCENE: what exists in the world, and what the menu does to it.
 *
 * Everything here answers "which bodies are in the world and what are they
 * made of". It reads the four CSV files, builds the starting scene, stamps
 * material changes onto bodies that already exist, and runs the menu's
 * actions. It never touches the keyboard (srcs/game/input.c), never draws
 * (srcs/render/) and never steps the simulation (srcs/physics/world.c).
 *
 * The object geometry itself comes from srcs/game/objectdef.c (one CSV file
 * to one RigidBody), the machine from srcs/game/trebuchet.c and the stacks of
 * blocks from srcs/game/structure.c.
*/

/* ========================================================================== */
/*  READING THE OBJECT FILES                                                  */
/* ========================================================================== */

static void	report_missing(void)
{
	fprintf(stderr, "%s, %s, %s and %s are all required and must be valid\n", ASSET_APPLE, ASSET_BLOCK, ASSET_GROUND, ASSET_TREBUCHET);
}

/* Loads the four CSV files into temporaries so a broken file leaves the
 * running definitions untouched (and the reason on stderr). */
int	scene_load_assets(Game *g)
{
	ObjectDef	apple;
	ObjectDef	block;
	ObjectDef	ground;
	Trebuchet	trebuchet;

	if (!objectdef_load(&apple, ASSET_APPLE, KIND_APPLE) || !objectdef_load(&block, ASSET_BLOCK, KIND_BLOCK) || !objectdef_load(&ground, ASSET_GROUND, KIND_GROUND))
		return (report_missing(), 0);
	if (!trebuchet_load(&trebuchet, ASSET_TREBUCHET, &apple))
		return (report_missing(), 0);
	if (ground.shape != SHAPE_PLANE || apple.shape != SHAPE_SPHERE || block.shape != SHAPE_BOX)
		return (fprintf(stderr, "%s must be a plane, %s a sphere and %s a box\n", ASSET_GROUND, ASSET_APPLE, ASSET_BLOCK), 0);
	g->appleDef = apple;
	g->blockDef = block;
	g->groundDef = ground;
	g->trebuchetDef = trebuchet;
	return (1);
}

/* ========================================================================== */
/*  BUILDING AND RE-STAMPING THE WORLD                                        */
/* ========================================================================== */

/* The starting scene: ground, trebuchet with its file launch settings, and
 * one pyramid. Everything spawned or fired before is gone. */
void	scene_build(Game *g)
{
	world_clear(&g->world);
	g->trebuchet = g->trebuchetDef;
	world_add_body(&g->world, objectdef_make_body(&g->groundDef, vec3(0.0f, 0.0f, 0.0f)));
	trebuchet_build(&g->trebuchet, &g->world);
	structure_spawn_pyramid(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 5);
	g->paused = 0;
	g->hud.refresh = 1;
}

/* Stamps a definition's material onto every existing body of its kind. The
 * mass goes through rb_set_mass so the inertia follows the current collider. */
static void	restamp_bodies(World *w, ObjectKind kind, float mass, float friction, float elasticity, Vec3 color)
{
	RigidBody	*b;
	int			i;

	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		if (b->kind == kind)
		{
			b->friction = friction;
			b->elasticity = elasticity;
			b->color = color;
			if (b->invMass > 0.0f)
				rb_set_mass(b, mass);
		}
		i++;
	}
}

/* Pushes the current definitions onto the bodies already in the world, so a
 * pile that has come to rest becomes lighter or slipperier immediately. */
void	scene_apply_definitions(Game *g)
{
	restamp_bodies(&g->world, KIND_APPLE, g->appleDef.mass, g->appleDef.friction, g->appleDef.elasticity, g->appleDef.color);
	restamp_bodies(&g->world, KIND_BLOCK, g->blockDef.mass, g->blockDef.friction, g->blockDef.elasticity, g->blockDef.color);
	restamp_bodies(&g->world, KIND_GROUND, 0.0f, g->groundDef.friction, g->groundDef.elasticity, g->groundDef.color);
	restamp_bodies(&g->world, KIND_TREBUCHET, 0.0f, g->trebuchet.friction, g->trebuchet.elasticity, g->trebuchet.color);
	g->trebuchet.projectileMass = g->appleDef.mass;
	g->trebuchet.projectileRadius = g->appleDef.radius;
}

/* ========================================================================== */
/*  THE DRAFT: what the menu edits, and how it reaches the simulation         */
/* ========================================================================== */

/* Copies the running values into the draft the menu edits, so opening the
 * menu always shows what the simulation is actually doing. */
void	scene_sync_draft(Game *g)
{
	g->draft.apple = g->appleDef;
	g->draft.block = g->blockDef;
	g->draft.ground = g->groundDef;
	g->draft.trebuchet = g->trebuchet;
	g->draft.gravityY = g->world.gravity.y;
	g->draft.timeScale = g->timeScale;
	g->menu.dirty = 0;
}

/* Apply: the draft the menu holds becomes the running simulation, and the
 * state a reset goes back to. */
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

/* Re-reads the CSV files and applies what can change on the fly. Existing
 * bodies take the new mass, friction, elasticity and color, the launch
 * settings return to the file values, and everything is woken so a lighter or
 * slipperier pile reacts at once. Geometry (shape, size, radius, the plane,
 * the trebuchet layout) only reaches bodies spawned from now on, or the whole
 * scene after a reset. */
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
	mesh_release(&g->planeMesh);
	g->planeMesh = mesh_plane(g->groundDef.extent);
	world_wake_all(&g->world);
	scene_sync_draft(g);
	g->hud.refresh = 1;
	printf("Object files reloaded: existing objects updated\n");
}

/* Every simulation parameter back to its starting value (gravity, time
 * scale, launch settings) and the scene back to its starting state. */
static void	reset_simulation(Game *g)
{
	g->world.gravity = vec3(0.0f, GRAVITY_Y, 0.0f);
	g->timeScale = 1.0f;
	scene_build(g);
	scene_sync_draft(g);
	printf("Simulation reset\n");
}

/* Writes the draft back to the four files, so values dialled in the menu
 * survive a restart. It does not touch the running simulation: that is what
 * Apply is for. */
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

/* Runs what the menu asked for. Resetting while the menu is open is on
 * purpose: it is where a scene gets set up before resuming. */
void	scene_run_action(Game *g, MenuAction action)
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

/* ========================================================================== */
/*  THE MENU'S ROWS                                                           */
/* ========================================================================== */

/* The left column: every number the menu can edit, each pointing into the
 * draft rather than at the simulation. */
static void	build_menu_values(Menu *m, Game *g)
{
	menu_add_section(m, "SIMULATION", 0);
	menu_add_value(m, "Gravity", &g->draft.gravityY, GRAVITY_MIN, GRAVITY_MAX, 0.5f, 2);
	menu_add_value(m, "Time scale", &g->draft.timeScale, 0.0f, TIME_SCALE_MAX, 0.1f, 2);
	menu_add_section(m, "LAUNCH", 0);
	menu_add_value(m, "Speed", &g->draft.trebuchet.launchSpeed, LAUNCH_SPEED_MIN, LAUNCH_SPEED_MAX, 1.0f, 1);
	menu_add_value(m, "Angle", &g->draft.trebuchet.launchAngle, LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX, 5.0f, 0);
	menu_add_section(m, "APPLE", 0);
	menu_add_value(m, "Mass", &g->draft.apple.mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX, 0.25f, 2);
	menu_add_value(m, "Friction", &g->draft.apple.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Elasticity", &g->draft.apple.elasticity, 0.0f, 1.0f, 0.05f, 2);
	menu_add_section(m, "BLOCK", 0);
	menu_add_value(m, "Mass", &g->draft.block.mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX, 0.25f, 2);
	menu_add_value(m, "Friction", &g->draft.block.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Elasticity", &g->draft.block.elasticity, 0.0f, 1.0f, 0.05f, 2);
	menu_add_section(m, "GROUND", 0);
	menu_add_value(m, "Friction", &g->draft.ground.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Elasticity", &g->draft.ground.elasticity, 0.0f, 1.0f, 0.05f, 2);
	menu_add_section(m, "TREBUCHET", 0);
	menu_add_value(m, "Friction", &g->draft.trebuchet.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Elasticity", &g->draft.trebuchet.elasticity, 0.0f, 1.0f, 0.05f, 2);
}

/* The right column: the key reference and the buttons. */
static void	build_menu_controls(Menu *m)
{
	menu_add_section(m, "CONTROLS", 1);
	menu_add_hint(m, "ESC     menu (pauses)");
	menu_add_hint(m, "SPACE   fire an apple");
	menu_add_hint(m, "1 2 3   wall/pyr/tower");
	menu_add_hint(m, "4       big wall (150)");
	menu_add_hint(m, "P       pause / resume");
	menu_add_hint(m, "F1      wireframe view");
	menu_add_hint(m, "H       title readout");
	menu_add_hint(m, "I K     launch angle + -");
	menu_add_hint(m, "J L     launch speed - +");
	menu_add_hint(m, "Q E     apple mass - +");
	menu_add_hint(m, "G B     gravity + -");
	menu_add_hint(m, "T Y     time scale - +");
	menu_add_hint(m, "W A S D fly the camera");
	menu_add_hint(m, "R F     camera up / down");
	menu_add_hint(m, "arrows  turn the camera");
	menu_add_hint(m, "+ -     camera fly speed");
	menu_add_section(m, "ACTIONS", 1);
	menu_add_action(m, "Apply to simulation", ACTION_APPLY, 1);
	menu_add_action(m, "Reload from files", ACTION_RELOAD, 1);
	menu_add_action(m, "Save to files", ACTION_SAVE, 1);
	menu_add_action(m, "Reset simulation", ACTION_RESET, 1);
	menu_add_action(m, "Resume", ACTION_RESUME, 1);
	menu_add_action(m, "Quit", ACTION_QUIT, 1);
}

void	scene_build_menu(Game *g)
{
	Menu	*m = &g->menu;

	m->rowCount = 0;
	m->selected = 1;
	m->hoverRow = -1;
	m->heldRow = -1;
	build_menu_values(m, g);
	build_menu_controls(m);
}
