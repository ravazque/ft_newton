#include "newton.h"

/*
 * Places the panel and every row. The panel covers MENU_PANEL_RATIO of the window
 * in both directions and the content scales with it: everything is measured once
 * in font pixels, the largest scale that fits wins (no whole-number steps, glyph
 * pixels are snapped when drawn), rows stretch a little to fill the height and
 * both columns share the spare width. The rectangles written here are the ones
 * menu_input.c hit-tests and menu_draw.c fills.
*/

/* Rows of the taller column, counting the gap before each section title after the first. */
static float	content_lines(const Menu *m)
{
	float	lines[2];
	int		i;

	lines[0] = 0.0f;
	lines[1] = 0.0f;
	i = 0;
	while (i < m->rowCount)
	{
		if (m->rows[i].kind == MENU_SECTION && lines[m->rows[i].column] > 0.0f)
			lines[m->rows[i].column] += 0.4f;
		lines[m->rows[i].column] += 1.0f;
		i++;
	}
	return (fmaxf(lines[0], lines[1]));
}

/* Font pixels one column needs: its widest label, plus number and buttons for a value row. */
static float	column_width(const Menu *m, int column)
{
	float	widest = 0.0f;
	float	needed;
	int		i;

	i = 0;
	while (i < m->rowCount)
	{
		if (m->rows[i].column == column)
		{
			needed = ui_text_width(m->rows[i].label, 1.0f) + 2.0f * FONT_ADVANCE;
			if (m->rows[i].kind == MENU_VALUE)
				needed += (MENU_VALUE_CHARS + 2.0f * MENU_BUTTON_CHARS) * FONT_ADVANCE + MENU_BUTTON_GAP;
			widest = fmaxf(widest, needed);
		}
		i++;
	}
	return (widest);
}

/* Font pixels of the whole panel at scale 1: columns side by side, or the title line if wider. */
static void	natural_size(const Menu *m, float *w, float *h)
{
	float	columns = column_width(m, 0) + column_width(m, 1) + 3.0f * MENU_PAD;
	float	header = ui_text_width(MENU_TITLE, 1.0f) + ui_text_width(MENU_NOTE_DIRTY, 1.0f) + 4.0f * FONT_ADVANCE + 2.0f * MENU_PAD;

	*w = fmaxf(columns, header);
	*h = (content_lines(m) + 1.5f) * MENU_LINE_PIXELS + 2.0f * MENU_PAD;
}

static void	place_panel(Menu *m, float width, float height)
{
	float	natural_w;
	float	natural_h;

	natural_size(m, &natural_w, &natural_h);
	m->panelW = width * MENU_PANEL_RATIO;
	m->panelH = height * MENU_PANEL_RATIO;
	m->scale = fmaxf(MENU_MIN_SCALE, fminf(m->panelW / natural_w, m->panelH / natural_h));
	m->panelW = fminf(width, fmaxf(m->panelW, natural_w * m->scale));
	m->panelH = fminf(height, fmaxf(m->panelH, natural_h * m->scale));
	m->panelX = (width - m->panelW) * 0.5f;
	m->panelY = (height - m->panelH) * 0.5f;
	m->pad = MENU_PAD * m->scale;
	m->line = (m->panelH - 2.0f * m->pad) / (content_lines(m) + 1.5f);
	m->line = clampf(m->line, MENU_LINE_PIXELS * m->scale, MENU_LINE_PIXELS * m->scale * MENU_MAX_STRETCH);
}

static void	place_row(Menu *m, MenuRow *row, float x, float y, float w)
{
	row->x = x;
	row->y = y;
	row->w = w;
	row->h = m->line;
	row->buttonW = MENU_BUTTON_CHARS * FONT_ADVANCE * m->scale;
	row->plusX = row->x + row->w - row->buttonW;
	row->minusX = row->plusX - row->buttonW - MENU_BUTTON_GAP * m->scale;
}

void	menu_layout(Menu *m, float width, float height)
{
	float	col_w[2];
	float	spare;
	float	top;
	float	y[2];
	int		i;

	place_panel(m, width, height);
	col_w[0] = column_width(m, 0) * m->scale;
	col_w[1] = column_width(m, 1) * m->scale;
	spare = fmaxf(0.0f, (m->panelW - 3.0f * m->pad - col_w[0] - col_w[1]) * 0.5f);
	col_w[0] += spare;
	col_w[1] += spare;
	top = m->panelY + m->pad + 1.5f * m->line;
	top += fmaxf(0.0f, (m->panelY + m->panelH - m->pad - top - content_lines(m) * m->line) * 0.5f);
	y[0] = top;
	y[1] = top;
	i = 0;
	while (i < m->rowCount)
	{
		if (m->rows[i].kind == MENU_SECTION && y[m->rows[i].column] > top)
			y[m->rows[i].column] += 0.4f * m->line;
		place_row(m, &m->rows[i], m->panelX + m->pad + (float)m->rows[i].column * (col_w[0] + m->pad), y[m->rows[i].column], col_w[m->rows[i].column]);
		y[m->rows[i].column] += m->line;
		i++;
	}
}
