#include "newton.h"

/*
 * The pause menu's rows. The menu only moves the floats its rows point at and
 * reports which action was clicked; what a value means is the game's business
 * (srcs/game/menu_rows.c and actions.c). Layout, input and drawing live in
 * menu_layout.c, menu_input.c and menu_draw.c and all work on these rows.
*/

static MenuRow	*new_row(Menu *m, MenuRowKind kind, const char *label, int column)
{
	MenuRow	*row;

	if (m->rowCount >= MENU_MAX_ROWS)
		return (NULL);
	row = &m->rows[m->rowCount];
	memset(row, 0, sizeof(*row));
	row->kind = kind;
	row->label = label;
	row->column = column;
	m->rowCount++;
	return (row);
}

void	menu_add_section(Menu *m, const char *label, int column)
{
	new_row(m, MENU_SECTION, label, column);
}

void	menu_add_value(Menu *m, const char *label, float *value, float min_value, float max_value, float step, int decimals)
{
	MenuRow	*row = new_row(m, MENU_VALUE, label, 0);

	if (!row)
		return ;
	row->value = value;
	row->min = min_value;
	row->max = max_value;
	row->step = step;
	row->decimals = decimals;
}

void	menu_add_action(Menu *m, const char *label, MenuAction action, int column)
{
	MenuRow	*row = new_row(m, MENU_ACTION, label, column);

	if (row)
		row->action = action;
}

void	menu_add_hint(Menu *m, const char *label)
{
	new_row(m, MENU_HINT, label, 1);
}

int	menu_is_selectable(const MenuRow *row)
{
	return (row->kind == MENU_VALUE || row->kind == MENU_ACTION);
}

/* One step of a value row, snapped to the step size so repeated clicks land on round numbers. */
void	menu_step_value(Menu *m, MenuRow *row, int direction)
{
	float	v;

	if (row->kind != MENU_VALUE || !row->value)
		return ;
	v = *row->value + (float)direction * row->step;
	v = roundf(v / row->step) * row->step;
	*row->value = clampf(v, row->min, row->max);
	m->valueChanged = 1;
	m->dirty = 1;
}
