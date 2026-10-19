#include "newton.h"

// The menu pages: the main page edits the draft and runs the actions, the models page lists every loaded model.

static void	add_material(Menu *menu, ObjectDef *def)
{
	menu_add_value(menu, "Friction", &def->friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(menu, "Elasticity", &def->elasticity, 0.0f, 1.0f, 0.05f, 2);
}

static void	add_scene_values(Menu *menu, Draft *draft)
{
	menu_add_section(menu, "SIMULATION", 0);
	menu_add_value(menu, "Gravity", &draft->gravityY, GRAVITY_MIN, GRAVITY_MAX, 0.5f, 2);
	menu_add_value(menu, "Time scale", &draft->timeScale, 0.0f, TIME_SCALE_MAX, 0.1f, 2);
	menu_add_section(menu, "LAUNCH", 0);
	menu_add_value(menu, "Speed", &draft->trebuchet.launchSpeed, LAUNCH_SPEED_MIN, LAUNCH_SPEED_MAX, 1.0f, 1);
	menu_add_value(menu, "Angle", &draft->trebuchet.launchAngle, LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX, 5.0f, 0);
	menu_add_section(menu, "APPLE", 0);
	menu_add_value(menu, "Mass", &draft->apple.mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX, 0.25f, 2);
	add_material(menu, &draft->apple);
	menu_add_value(menu, "Rolling res.", &draft->apple.rollingResistance, 0.0f, ROLLING_RESISTANCE_MAX, 0.05f, 2);
	menu_add_section(menu, "BLOCK", 0);
	menu_add_value(menu, "Mass", &draft->block.mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX, 0.25f, 2);
	add_material(menu, &draft->block);
	menu_add_section(menu, "GROUND", 0);
	add_material(menu, &draft->ground);
	menu_add_section(menu, "TREBUCHET", 0);
	menu_add_value(menu, "Friction", &draft->trebuchet.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(menu, "Elasticity", &draft->trebuchet.elasticity, 0.0f, 1.0f, 0.05f, 2);
}

static void	add_body_values(Menu *menu, Draft *draft)
{
	menu_add_section(menu, "RAGDOLL", 1);
	menu_add_value(menu, "Mass", &draft->ragdoll.mass, RAGDOLL_MASS_MIN, RAGDOLL_MASS_MAX, 1.0f, 1);
	menu_add_value(menu, "Friction", &draft->ragdoll.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(menu, "Elasticity", &draft->ragdoll.elasticity, 0.0f, 1.0f, 0.05f, 2);
	menu_add_section(menu, "MODELS", 1);
	menu_add_value(menu, "Size", &draft->model.modelSize, MODEL_SIZE_MIN, MODEL_SIZE_MAX, 0.1f, 1);
	menu_add_value(menu, "Density", &draft->model.density, MODEL_DENSITY_MIN, MODEL_DENSITY_MAX, 10.0f, 0);
	add_material(menu, &draft->model);
	menu_add_action(menu, "Choose a model", ACTION_MODELS);
	menu_add_section(menu, "ACTIONS", 1);
	menu_add_action(menu, "Apply", ACTION_APPLY);
	menu_add_action(menu, "Reload from files", ACTION_RELOAD);
	menu_add_action(menu, "Save to files", ACTION_SAVE);
	menu_add_action(menu, "Reset simulation", ACTION_RESET);
	menu_add_action(menu, "Resume", ACTION_RESUME);
	menu_add_action(menu, "Quit", ACTION_QUIT);
}

static void	add_controls(Menu *menu)
{
	menu_add_section(menu, "CONTROLS", 2);
	menu_add_hint(menu, "ESC     menu (pauses)");
	menu_add_hint(menu, "SPACE   fire an apple");
	menu_add_hint(menu, "1 2 3   wall/pyr/tower");
	menu_add_hint(menu, "4       big wall (150)");
	menu_add_hint(menu, "5       ragdoll");
	menu_add_hint(menu, "6       place the model");
	menu_add_hint(menu, "7 8     next/prev model");
	menu_add_hint(menu, "9       test rain");
	menu_add_hint(menu, "P       pause / resume");
	menu_add_hint(menu, "F1      colliders+contacts");
	menu_add_hint(menu, "H       title readout");
	menu_add_hint(menu, "I K     launch angle + -");
	menu_add_hint(menu, "J L     launch speed - +");
	menu_add_hint(menu, "Q E     apple mass - +");
	menu_add_hint(menu, "G B     gravity + -");
	menu_add_hint(menu, "0       gravity off / on");
	menu_add_hint(menu, "T Y     time scale - +");
	menu_add_hint(menu, "W A S D fly the camera");
	menu_add_hint(menu, "R F     camera up / down");
	menu_add_hint(menu, "arrows  turn the camera");
	menu_add_hint(menu, "+ -     camera fly speed");
}

static void	add_model_page(Menu *menu, const ModelLibrary *library)
{
	int	i;

	menu_add_section(menu, "MODELS", 0);
	if (library->count == 0)
		menu_add_hint(menu, "no model in " MODEL_DIR);
	i = 0;
	while (i < library->count)
	{
		menu_add_model(menu, library->labels[i], i);
		i++;
	}
	menu_add_section(menu, "PLACING", 1);
	menu_add_hint(menu, "click   place it now");
	menu_add_hint(menu, "6       place the selected");
	menu_add_hint(menu, "7 8     change the selection");
	menu_add_hint(menu, "9       test rain on it");
	menu_add_section(menu, "FILES", 1);
	menu_add_hint(menu, MODEL_DIR "*" MODEL_EXTENSION);
	menu_add_hint(menu, ASSET_MODELS);
	menu_add_section(menu, "MENU", 1);
	menu_add_action(menu, "Back", ACTION_BACK);
}

static int	first_selectable(const Menu *menu)
{
	int	i;

	i = 0;
	while (i < menu->rowCount)
	{
		if (menu_is_selectable(&menu->rows[i]))
			return (i);
		i++;
	}
	return (-1);
}

void	menu_rows_build(Game *game)
{
	Menu	*menu = &game->menu;

	menu->rowCount = 0;
	menu->hoverRow = -1;
	menu->heldRow = -1;
	menu->heldPart = -1;
	if (menu->page == MENU_PAGE_MODELS)
		add_model_page(menu, &game->models);
	else
	{
		add_scene_values(menu, &game->draft);
		add_body_values(menu, &game->draft);
		add_controls(menu);
	}
	menu->selected = first_selectable(menu);
}
