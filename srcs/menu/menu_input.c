#include "newton.h"

/*
 * Menu input, mouse and keyboard on the same rows: the mouse hit-tests the
 * rectangles of the last layout, holding [-] / [+] repeats like a spin box, the
 * arrows move the cursor and change values, ENTER presses a button. Results of
 * the frame are left in m->pending, m->valueChanged and m->dirty.
*/

static int	inside(float px, float py, float x, float y, float w, float h)
{
	return (px >= x && px <= x + w && py >= y && py <= y + h);
}

/* hoverPart: 0 = [-], 1 = [+], 2 = an action button. */
static void	find_hover(Menu *m, float mx, float my)
{
	MenuRow	*row;
	int		i;

	m->hoverRow = -1;
	m->hoverPart = -1;
	i = 0;
	while (i < m->rowCount)
	{
		row = &m->rows[i];
		if (menu_is_selectable(row) && inside(mx, my, row->x, row->y, row->w, row->h))
		{
			m->hoverRow = i;
			if (row->kind == MENU_ACTION)
				m->hoverPart = 2;
			else if (inside(mx, my, row->minusX, row->y, row->buttonW, row->h))
				m->hoverPart = 0;
			else if (inside(mx, my, row->plusX, row->y, row->buttonW, row->h))
				m->hoverPart = 1;
		}
		i++;
	}
}

static void	hold_repeat(Menu *m, float frame_time)
{
	int	direction;

	if (m->heldRow < 0 || m->heldPart > 1)
		return ;
	m->repeatTimer += frame_time;
	direction = 1;
	if (m->heldPart == 0)
		direction = -1;
	while (m->repeatTimer >= MENU_REPEAT_DELAY + MENU_REPEAT_PERIOD)
	{
		menu_step_value(m, &m->rows[m->heldRow], direction);
		m->repeatTimer -= MENU_REPEAT_PERIOD;
	}
}

static void	press(Menu *m)
{
	MenuRow	*row;

	if (m->hoverRow < 0)
		return ;
	m->selected = m->hoverRow;
	row = &m->rows[m->hoverRow];
	m->heldRow = m->hoverRow;
	m->heldPart = m->hoverPart;
	m->repeatTimer = 0.0f;
	if (m->hoverPart == 0)
		menu_step_value(m, row, -1);
	else if (m->hoverPart == 1)
		menu_step_value(m, row, 1);
	else if (m->hoverPart == 2)
		m->pending = row->action;
}

static void	move_selection(Menu *m, int direction)
{
	int	i = m->selected;
	int	guard = 0;

	while (guard < m->rowCount)
	{
		i += direction;
		if (i < 0)
			i = m->rowCount - 1;
		if (i >= m->rowCount)
			i = 0;
		if (menu_is_selectable(&m->rows[i]))
		{
			m->selected = i;
			return ;
		}
		guard++;
	}
}

static int	key_edge(Window *win, int key, char *state)
{
	int	down = window_key_down(win, key);

	if (down && !*state)
	{
		*state = 1;
		return (1);
	}
	if (!down)
		*state = 0;
	return (0);
}

static void	keyboard(Menu *m, Window *win)
{
	static char	keys[5];
	MenuRow		*row;

	if (key_edge(win, GLFW_KEY_UP, &keys[0]))
		move_selection(m, -1);
	if (key_edge(win, GLFW_KEY_DOWN, &keys[1]))
		move_selection(m, 1);
	if (m->selected < 0 || m->selected >= m->rowCount)
		return ;
	row = &m->rows[m->selected];
	if (key_edge(win, GLFW_KEY_LEFT, &keys[2]))
		menu_step_value(m, row, -1);
	if (key_edge(win, GLFW_KEY_RIGHT, &keys[3]))
		menu_step_value(m, row, 1);
	if (key_edge(win, GLFW_KEY_ENTER, &keys[4]) && row->kind == MENU_ACTION)
		m->pending = row->action;
}

void	menu_update(Menu *m, Window *win, float frame_time)
{
	static int	was_down = 0;
	float		mx;
	float		my;
	int			down;

	m->pending = ACTION_NONE;
	m->valueChanged = 0;
	window_cursor(win, &mx, &my);
	find_hover(m, mx, my);
	down = window_mouse_down(win);
	if (down && !was_down)
		press(m);
	if (!down)
	{
		m->heldRow = -1;
		m->heldPart = -1;
		m->repeatTimer = 0.0f;
	}
	was_down = down;
	hold_repeat(m, frame_time);
	keyboard(m, win);
}
