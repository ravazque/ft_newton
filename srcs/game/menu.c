#include "newton.h"

/*
 * The pause menu (ESC): every tunable value with [-] / [+] buttons, the list
 * of controls, and the actions (reload, save, reset, spawn, resume, quit).
 *
 * It is deliberately dumb: rows point at floats and carry an action id, and
 * the menu only moves those floats and reports which action was clicked.
 * What a value means, and what happens when it moves, is the game's business
 * (game.c), which keeps the physics out of the interface and the interface
 * out of the physics.
 *
 * Mouse and keyboard drive the same rows: the layout writes each row's
 * rectangle every frame, the mouse hit-tests those rectangles and the
 * keyboard walks the same list.
*/

static const char	*g_back_note = "ESC or RESUME to go back";
static const char	*g_dirty_note = "APPLY to use the edits";

static const Vec3	g_panel = {0.07f, 0.08f, 0.11f};
static const Vec3	g_panel_edge = {0.35f, 0.40f, 0.50f};
static const Vec3	g_section = {1.00f, 0.80f, 0.25f};
static const Vec3	g_label = {0.85f, 0.88f, 0.92f};
static const Vec3	g_value = {0.55f, 0.85f, 1.00f};
static const Vec3	g_hint = {0.62f, 0.66f, 0.72f};
static const Vec3	g_button = {0.18f, 0.22f, 0.30f};
static const Vec3	g_button_hot = {0.30f, 0.45f, 0.65f};
static const Vec3	g_selected = {0.14f, 0.18f, 0.26f};

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

static int	is_selectable(const MenuRow *row)
{
	return (row->kind == MENU_VALUE || row->kind == MENU_ACTION);
}

/* Extra space before a section title, so groups read apart. */
static float	section_gap(const Menu *m, const MenuRow *row, float column_top, float y)
{
	if (row->kind == MENU_SECTION && y > column_top)
		return (m->line * 0.4f);
	return (0.0f);
}

/* Height the taller column needs at a given font scale. The layout picks the
 * largest scale whose content still fits the window, so the menu is as big as
 * it can be and never runs off the bottom. */
/* A glyph is FONT_GLYPH_SIZE tall inside a row of MENU_LINE_PIXELS, so it
 * sits centred when it starts half the difference down. Every label, value,
 * hint and button caption goes through this, so they share one baseline. */
static float	centered_text_y(float y, float h, float scale)
{
	return (y + (h - (float)FONT_GLYPH_SIZE * scale) * 0.5f);
}

static float	content_height(Menu *m, float scale)
{
	float	line = MENU_LINE_PIXELS * scale;
	float	y[2];
	int		i;

	y[0] = 0.0f;
	y[1] = 0.0f;
	i = 0;
	while (i < m->rowCount)
	{
		if (m->rows[i].kind == MENU_SECTION && y[m->rows[i].column] > 0.0f)
			y[m->rows[i].column] += line * 0.4f;
		y[m->rows[i].column] += line;
		i++;
	}
	return (fmaxf(y[0], y[1]));
}

/* Width one column needs: the widest row in it, where a value row also has
 * to fit its number and its two buttons next to the label. */
static float	column_width(Menu *m, int column, float scale)
{
	float	widest = 0.0f;
	float	needed;
	int		i;

	i = 0;
	while (i < m->rowCount)
	{
		if (m->rows[i].column == column)
		{
			needed = ui_text_width(m->rows[i].label, scale) + 2.0f * FONT_ADVANCE * scale;
			if (m->rows[i].kind == MENU_VALUE)
				needed += (MENU_VALUE_CHARS + 2.0f * MENU_BUTTON_CHARS) * FONT_ADVANCE * scale + MENU_BUTTON_GAP * scale;
			widest = fmaxf(widest, needed);
		}
		i++;
	}
	return (widest);
}

/* The title line sits above the columns, so the panel has to be wide enough
 * for it too or the longest note would run into the title. */
static float	header_width(float scale)
{
	return (ui_text_width("ft_newton", scale) + ui_text_width(g_dirty_note, scale) + 4.0f * FONT_ADVANCE * scale);
}

static float	content_width(Menu *m, float scale)
{
	float	columns = column_width(m, 0, scale) + column_width(m, 1, scale) + 9.0f * scale;

	return (fmaxf(columns, header_width(scale)));
}

/* Text is drawn in whole font pixels, so the scale is a whole number: the
 * menu grows with the window instead of blurring. The largest scale whose
 * content still fits the share of the window the panel may take wins, so
 * nothing is ever clipped and the scene stays visible around it. */
static float	fit_scale(Menu *m, float width, float height)
{
	float	scale = 3.0f;

	while (scale > 1.0f)
	{
		float	line = MENU_LINE_PIXELS * scale;
		float	needed_h = content_height(m, scale) + 3.0f * line;
		float	needed_w = content_width(m, scale) + 24.0f * scale;

		if (needed_h <= height * MENU_HEIGHT_RATIO && needed_w <= width * MENU_WIDTH_RATIO)
			return (scale);
		scale -= 1.0f;
	}
	return (1.0f);
}

/* Places the panel and every row. The left column holds the values, the
 * right column the controls reference and the action buttons. */
void	menu_layout(Menu *m, float width, float height)
{
	float	left_w;
	float	top;
	float	y[2];
	int		i;

	m->scale = fit_scale(m, width, height);
	m->line = MENU_LINE_PIXELS * m->scale;
	m->pad = 6.0f * m->scale;
	m->panelW = fminf(width * MENU_WIDTH_RATIO, content_width(m, m->scale) + 3.0f * m->pad);
	m->panelH = fminf(height * MENU_HEIGHT_RATIO, content_height(m, m->scale) + 3.0f * m->line + 2.0f * m->pad);
	m->panelX = (width - m->panelW) * 0.5f;
	m->panelY = (height - m->panelH) * 0.5f;
	left_w = column_width(m, 0, m->scale);
	top = m->panelY + m->pad + m->line * 1.5f;
	y[0] = top;
	y[1] = top;
	i = 0;
	while (i < m->rowCount)
	{
		MenuRow	*row = &m->rows[i];
		int		col = row->column;

		y[col] += section_gap(m, row, top, y[col]);
		row->x = m->panelX + m->pad + (float)col * (left_w + m->pad);
		row->y = y[col];
		row->w = column_width(m, col, m->scale);
		row->h = m->line;
		row->buttonW = MENU_BUTTON_CHARS * FONT_ADVANCE * m->scale;
		row->plusX = row->x + row->w - row->buttonW;
		row->minusX = row->plusX - row->buttonW - MENU_BUTTON_GAP * m->scale;
		y[col] += m->line;
		i++;
	}
}

static float	clampf(float v, float lo, float hi)
{
	if (v < lo)
		return (lo);
	if (v > hi)
		return (hi);
	return (v);
}

/* One step of a value row. Steps are snapped to the step size so repeated
 * clicks land on round numbers instead of drifting. */
static void	apply_step(Menu *m, MenuRow *row, int direction)
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

static int	inside(float px, float py, float x, float y, float w, float h)
{
	return (px >= x && px <= x + w && py >= y && py <= y + h);
}

/* Which row and which part of it the cursor is over: 0 = [-], 1 = [+],
 * 2 = the row itself (an action button). */
static void	find_hover(Menu *m, float mx, float my)
{
	int	i;

	m->hoverRow = -1;
	m->hoverPart = -1;
	i = 0;
	while (i < m->rowCount)
	{
		MenuRow	*row = &m->rows[i];

		if (is_selectable(row) && inside(mx, my, row->x, row->y, row->w, row->h))
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

/* Holding a [-] / [+] repeats after a short delay, the way a spin box does. */
static void	hold_repeat(Menu *m, float frame_time)
{
	MenuRow	*row;
	int		direction;

	if (m->heldRow < 0 || m->heldPart > 1)
		return ;
	m->repeatTimer += frame_time;
	if (m->repeatTimer < MENU_REPEAT_DELAY)
		return ;
	row = &m->rows[m->heldRow];
	direction = 1;
	if (m->heldPart == 0)
		direction = -1;
	while (m->repeatTimer >= MENU_REPEAT_DELAY + MENU_REPEAT_PERIOD)
	{
		apply_step(m, row, direction);
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
		apply_step(m, row, -1);
	else if (m->hoverPart == 1)
		apply_step(m, row, 1);
	else if (m->hoverPart == 2)
		m->pending = row->action;
}

/* Moves the keyboard cursor to the next selectable row in 'direction'. */
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
		if (is_selectable(&m->rows[i]))
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
	static char	keys[8];
	MenuRow		*row;

	if (key_edge(win, GLFW_KEY_UP, &keys[0]))
		move_selection(m, -1);
	if (key_edge(win, GLFW_KEY_DOWN, &keys[1]))
		move_selection(m, 1);
	if (m->selected < 0 || m->selected >= m->rowCount)
		return ;
	row = &m->rows[m->selected];
	if (key_edge(win, GLFW_KEY_LEFT, &keys[2]))
		apply_step(m, row, -1);
	if (key_edge(win, GLFW_KEY_RIGHT, &keys[3]))
		apply_step(m, row, 1);
	if (key_edge(win, GLFW_KEY_ENTER, &keys[4]) && row->kind == MENU_ACTION)
		m->pending = row->action;
}

/* One frame of menu input. Clears the per-frame results first, so the caller
 * always reads what happened during this very frame. */
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

static void	format_value(const MenuRow *row, char *out, size_t size)
{
	if (row->decimals == 0)
		snprintf(out, size, "%.0f", (double)*row->value);
	else if (row->decimals == 1)
		snprintf(out, size, "%.1f", (double)*row->value);
	else
		snprintf(out, size, "%.2f", (double)*row->value);
}

/* A button fills its whole row apart from one font pixel top and bottom, so
 * every button in the menu is the same height and every pair of stacked
 * buttons is the same distance apart. The caption is centred both ways. */
static void	draw_button(Ui *ui, float x, float y, float w, float h, const char *label, float scale, int hot)
{
	Vec3	fill = g_button;

	if (hot)
		fill = g_button_hot;
	ui_rect(ui, x, y + MENU_BUTTON_INSET * scale, w, h - 2.0f * MENU_BUTTON_INSET * scale, fill);
	ui_text(ui, label, x + (w - ui_text_width(label, scale)) * 0.5f, centered_text_y(y, h, scale), scale, g_label);
}

static void	draw_value_row(const Menu *m, Ui *ui, const MenuRow *row, int index, float scale)
{
	char	text[32];

	ui_text(ui, row->label, row->x + 3.0f * scale, centered_text_y(row->y, row->h, scale), scale, g_label);
	format_value(row, text, sizeof(text));
	ui_text_right(ui, text, row->minusX - MENU_BUTTON_GAP * scale, centered_text_y(row->y, row->h, scale), scale, g_value);
	draw_button(ui, row->minusX, row->y, row->buttonW, row->h, "-", scale, m->hoverRow == index && m->hoverPart == 0);
	draw_button(ui, row->plusX, row->y, row->buttonW, row->h, "+", scale, m->hoverRow == index && m->hoverPart == 1);
}

static void	draw_row(const Menu *m, Ui *ui, int index, float scale)
{
	const MenuRow	*row = &m->rows[index];
	float			text_y = centered_text_y(row->y, row->h, scale);

	if (m->selected == index && is_selectable(row))
		ui_rect(ui, row->x, row->y, row->w, row->h, g_selected);
	if (row->kind == MENU_SECTION)
		ui_text(ui, row->label, row->x, text_y, scale, g_section);
	else if (row->kind == MENU_HINT)
		ui_text(ui, row->label, row->x + 3.0f * scale, text_y, scale, g_hint);
	else if (row->kind == MENU_VALUE)
		draw_value_row(m, ui, row, index, scale);
	else
		draw_button(ui, row->x, row->y, row->w, row->h, row->label, scale, m->hoverRow == index);
}

/* The title line, which doubles as the reminder that edits are still sitting
 * in the draft: nothing the menu changes reaches the simulation until Apply. */
static void	draw_header(const Menu *m, Ui *ui, float scale)
{
	const char	*note = g_back_note;
	Vec3		color = g_hint;

	if (m->dirty)
	{
		note = g_dirty_note;
		color = g_section;
	}
	ui_text(ui, "ft_newton", m->panelX + m->pad, m->panelY + m->pad, scale, g_section);
	ui_text_right(ui, note, m->panelX + m->panelW - m->pad, m->panelY + m->pad, scale, color);
}

/* The panel, the title and every row, using the rectangles menu_layout just
 * computed - the very ones the mouse was hit-tested against. */
void	menu_draw(const Menu *m, Ui *ui)
{
	float	scale = m->scale;
	int		i;

	ui_rect(ui, m->panelX, m->panelY, m->panelW, m->panelH, g_panel);
	ui_border(ui, m->panelX, m->panelY, m->panelW, m->panelH, 2.0f * scale, g_panel_edge);
	draw_header(m, ui, scale);
	i = 0;
	while (i < m->rowCount)
	{
		draw_row(m, ui, i, scale);
		i++;
	}
}
