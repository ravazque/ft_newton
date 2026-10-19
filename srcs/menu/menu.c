#include "newton.h"

// The menu rows: a value row moves the float it points at, a button reports its action, a section opens a column.

static MenuRow	*new_row(Menu *menu, MenuRowKind kind, const char *label, int column)
{
	MenuRow	*row;

	if (menu->rowCount >= MENU_MAX_ROWS)
		return (NULL);
	row = &menu->rows[menu->rowCount];
	memset(row, 0, sizeof(*row));
	row->kind = kind;
	row->label = label;
	row->column = column;
	row->model = -1;
	menu->rowCount++;
	return (row);
}

void	menu_add_section(Menu *menu, const char *label, int column)
{
	menu->column = column;
	new_row(menu, MENU_SECTION, label, column);
}

void	menu_add_value(Menu *menu, const char *label, float *value, float min_value, float max_value, float step, int decimals)
{
	MenuRow	*row = new_row(menu, MENU_VALUE, label, menu->column);

	if (!row)
		return ;
	row->value = value;
	row->min = min_value;
	row->max = max_value;
	row->step = step;
	row->decimals = decimals;
}

void	menu_add_action(Menu *menu, const char *label, MenuAction action)
{
	MenuRow	*row = new_row(menu, MENU_ACTION, label, menu->column);

	if (row)
		row->action = action;
}

void	menu_add_model(Menu *menu, const char *label, int model)
{
	MenuRow	*row = new_row(menu, MENU_ACTION, label, menu->column);

	if (!row)
		return ;
	row->action = ACTION_PLACE_MODEL;
	row->model = model;
}

void	menu_add_hint(Menu *menu, const char *label)
{
	new_row(menu, MENU_HINT, label, menu->column);
}

int	menu_is_selectable(const MenuRow *row)
{
	return (row->kind == MENU_VALUE || row->kind == MENU_ACTION);
}

// Snapped to the step size, so repeated clicks land on round numbers.
void	menu_step_value(Menu *menu, MenuRow *row, int direction)
{
	float	value;

	if (row->kind != MENU_VALUE || !row->value)
		return ;
	value = *row->value + (float)direction * row->step;
	value = roundf(value / row->step) * row->step;
	*row->value = clampf(value, row->min, row->max);
	menu->valueChanged = 1;
	menu->dirty = 1;
}
