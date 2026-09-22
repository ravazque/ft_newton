#include "newton.h"

/* The content of the pause menu: every editable number (pointing into the Draft, never the
 * live world), the key reference and the action buttons. */

static void	add_material(Menu *m, ObjectDef *def)
{
	menu_add_value(m, "Friction", &def->friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Elasticity", &def->elasticity, 0.0f, 1.0f, 0.05f, 2);
}

static void	add_values(Menu *m, Draft *d)
{
	menu_add_section(m, "SIMULATION", 0);
	menu_add_value(m, "Gravity", &d->gravityY, GRAVITY_MIN, GRAVITY_MAX, 0.5f, 2);
	menu_add_value(m, "Time scale", &d->timeScale, 0.0f, TIME_SCALE_MAX, 0.1f, 2);
	menu_add_section(m, "LAUNCH", 0);
	menu_add_value(m, "Speed", &d->trebuchet.launchSpeed, LAUNCH_SPEED_MIN, LAUNCH_SPEED_MAX, 1.0f, 1);
	menu_add_value(m, "Angle", &d->trebuchet.launchAngle, LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX, 5.0f, 0);
	menu_add_section(m, "APPLE", 0);
	menu_add_value(m, "Mass", &d->apple.mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX, 0.25f, 2);
	add_material(m, &d->apple);
	menu_add_value(m, "Rolling res.", &d->apple.rollingResistance, 0.0f, ROLLING_RESISTANCE_MAX, 0.05f, 2);
	menu_add_section(m, "BLOCK", 0);
	menu_add_value(m, "Mass", &d->block.mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX, 0.25f, 2);
	add_material(m, &d->block);
	menu_add_section(m, "GROUND", 0);
	add_material(m, &d->ground);
	menu_add_section(m, "TREBUCHET", 0);
	menu_add_value(m, "Friction", &d->trebuchet.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Elasticity", &d->trebuchet.elasticity, 0.0f, 1.0f, 0.05f, 2);
}

static void	add_controls(Menu *m)
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
}

static void	add_actions(Menu *m)
{
	menu_add_section(m, "ACTIONS", 1);
	menu_add_action(m, "Apply", ACTION_APPLY, 1);
	menu_add_action(m, "Reload from files", ACTION_RELOAD, 1);
	menu_add_action(m, "Save to files", ACTION_SAVE, 1);
	menu_add_action(m, "Reset simulation", ACTION_RESET, 1);
	menu_add_action(m, "Resume", ACTION_RESUME, 1);
	menu_add_action(m, "Quit", ACTION_QUIT, 1);
}

void	menu_rows_build(Game *g)
{
	Menu	*m = &g->menu;

	m->rowCount = 0;
	m->selected = 1;
	m->hoverRow = -1;
	m->heldRow = -1;
	add_values(m, &g->draft);
	add_controls(m);
	add_actions(m);
}
