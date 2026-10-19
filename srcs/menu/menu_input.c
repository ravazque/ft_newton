#include "newton.h"

// Mouse and keyboard on the same rows; a frame's results are left in pending, pendingModel, valueChanged and dirty.

static int	point_inside(float point_x, float point_y, float x, float y, float width, float height)
{
	return (point_x >= x && point_x <= x + width && point_y >= y && point_y <= y + height);
}

static void	find_hover(Menu *menu, float cursor_x, float cursor_y)
{
	MenuRow	*row;
	int		i;

	menu->hoverRow = -1;
	menu->hoverPart = -1;
	i = 0;
	while (i < menu->rowCount)
	{
		row = &menu->rows[i];
		if (menu_is_selectable(row) && point_inside(cursor_x, cursor_y, row->x, row->y, row->width, row->height))
		{
			menu->hoverRow = i;
			if (row->kind == MENU_ACTION)
				menu->hoverPart = 2;
			else if (point_inside(cursor_x, cursor_y, row->minusX, row->y, row->buttonWidth, row->height))
				menu->hoverPart = 0;
			else if (point_inside(cursor_x, cursor_y, row->plusX, row->y, row->buttonWidth, row->height))
				menu->hoverPart = 1;
		}
		i++;
	}
}

// Holding [-] or [+] repeats like a spin box: one step on the press, then one every MENU_REPEAT_PERIOD after a delay.
static void	hold_repeat(Menu *menu, float frame_time)
{
	int	direction;

	if (menu->heldRow < 0 || menu->heldPart > 1)
		return ;
	menu->repeatTimer += frame_time;
	direction = 1;
	if (menu->heldPart == 0)
		direction = -1;
	while (menu->repeatTimer >= MENU_REPEAT_DELAY + MENU_REPEAT_PERIOD)
	{
		menu_step_value(menu, &menu->rows[menu->heldRow], direction);
		menu->repeatTimer -= MENU_REPEAT_PERIOD;
	}
}

static void	press_button(Menu *menu, const MenuRow *row)
{
	menu->pending = row->action;
	menu->pendingModel = row->model;
}

static void	press_hovered(Menu *menu)
{
	MenuRow	*row;

	if (menu->hoverRow < 0)
		return ;
	menu->selected = menu->hoverRow;
	row = &menu->rows[menu->hoverRow];
	menu->heldRow = menu->hoverRow;
	menu->heldPart = menu->hoverPart;
	menu->repeatTimer = 0.0f;
	if (menu->hoverPart == 0)
		menu_step_value(menu, row, -1);
	else if (menu->hoverPart == 1)
		menu_step_value(menu, row, 1);
	else if (menu->hoverPart == 2)
		press_button(menu, row);
}

static void	move_selection(Menu *menu, int direction)
{
	int	index = menu->selected;
	int	tried;

	tried = 0;
	while (tried < menu->rowCount)
	{
		index += direction;
		if (index < 0)
			index = menu->rowCount - 1;
		if (index >= menu->rowCount)
			index = 0;
		if (menu_is_selectable(&menu->rows[index]))
		{
			menu->selected = index;
			return ;
		}
		tried++;
	}
}

static int	key_pressed(Window *window, int key, char *was_down)
{
	int	down = window_key_down(window, key);

	if (down && !*was_down)
	{
		*was_down = 1;
		return (1);
	}
	if (!down)
		*was_down = 0;
	return (0);
}

static void	keyboard(Menu *menu, Window *window)
{
	static char	keys_down[5];
	MenuRow		*row;

	if (key_pressed(window, GLFW_KEY_UP, &keys_down[0]))
		move_selection(menu, -1);
	if (key_pressed(window, GLFW_KEY_DOWN, &keys_down[1]))
		move_selection(menu, 1);
	if (menu->selected < 0 || menu->selected >= menu->rowCount)
		return ;
	row = &menu->rows[menu->selected];
	if (key_pressed(window, GLFW_KEY_LEFT, &keys_down[2]))
		menu_step_value(menu, row, -1);
	if (key_pressed(window, GLFW_KEY_RIGHT, &keys_down[3]))
		menu_step_value(menu, row, 1);
	if (key_pressed(window, GLFW_KEY_ENTER, &keys_down[4]) && row->kind == MENU_ACTION)
		press_button(menu, row);
}

void	menu_update(Menu *menu, Window *window, float frame_time)
{
	static int	mouse_was_down = 0;
	float		cursor_x;
	float		cursor_y;
	int			mouse_down;

	menu->pending = ACTION_NONE;
	menu->pendingModel = -1;
	menu->valueChanged = 0;
	window_cursor(window, &cursor_x, &cursor_y);
	find_hover(menu, cursor_x, cursor_y);
	mouse_down = window_mouse_down(window);
	if (mouse_down && !mouse_was_down)
		press_hovered(menu);
	if (!mouse_down)
	{
		menu->heldRow = -1;
		menu->heldPart = -1;
		menu->repeatTimer = 0.0f;
	}
	mouse_was_down = mouse_down;
	hold_repeat(menu, frame_time);
	keyboard(menu, window);
}
