#include "newton.h"

// Panel and row rectangles: measured in font pixels at scale 1, then drawn at the largest scale the panel fits.

static float	content_lines(const Menu *menu)
{
	float	lines[MENU_COLUMNS] = {0};
	float	tallest;
	int		column;
	int		i;

	i = 0;
	while (i < menu->rowCount)
	{
		column = menu->rows[i].column;
		if (menu->rows[i].kind == MENU_SECTION && lines[column] > 0.0f)
			lines[column] += 0.4f;
		lines[column] += 1.0f;
		i++;
	}
	tallest = 0.0f;
	column = 0;
	while (column < MENU_COLUMNS)
	{
		tallest = fmaxf(tallest, lines[column]);
		column++;
	}
	return (tallest);
}

static float	column_width(const Menu *menu, int column)
{
	float	widest = 0.0f;
	float	needed;
	int		i;

	i = 0;
	while (i < menu->rowCount)
	{
		if (menu->rows[i].column == column)
		{
			needed = ui_text_width(menu->rows[i].label, 1.0f) + 2.0f * FONT_ADVANCE;
			if (menu->rows[i].kind == MENU_VALUE)
				needed += (MENU_VALUE_CHARS + 2.0f * MENU_BUTTON_CHARS) * FONT_ADVANCE + MENU_BUTTON_GAP;
			widest = fmaxf(widest, needed);
		}
		i++;
	}
	return (widest);
}

static int	used_columns(const Menu *menu)
{
	int	used;
	int	column;

	used = 0;
	column = 0;
	while (column < MENU_COLUMNS)
	{
		if (column_width(menu, column) > 0.0f)
			used++;
		column++;
	}
	if (used == 0)
		used = 1;
	return (used);
}

static float	header_width(const Menu *menu)
{
	float	title = ui_text_width(MENU_TITLE, 1.0f);
	float	note = fmaxf(ui_text_width(MENU_NOTE_BACK, 1.0f), ui_text_width(MENU_NOTE_DIRTY, 1.0f));

	if (menu->page == MENU_PAGE_MODELS)
	{
		title = ui_text_width(MENU_MODELS_TITLE, 1.0f);
		note = ui_text_width(MENU_NOTE_MODELS, 1.0f);
	}
	return (title + note + 4.0f * FONT_ADVANCE + 2.0f * MENU_PADDING);
}

static void	place_panel(Menu *menu, float width, float height)
{
	float	natural_width = (float)(used_columns(menu) + 1) * MENU_PADDING;
	float	natural_height = (content_lines(menu) + 1.5f) * MENU_LINE_PIXELS + 2.0f * MENU_PADDING;
	int		column;

	column = 0;
	while (column < MENU_COLUMNS)
	{
		natural_width += column_width(menu, column);
		column++;
	}
	natural_width = fmaxf(natural_width, header_width(menu));
	menu->panelWidth = width * MENU_PANEL_RATIO;
	menu->panelHeight = height * MENU_PANEL_RATIO;
	menu->scale = fmaxf(MENU_MIN_SCALE, fminf(menu->panelWidth / natural_width, menu->panelHeight / natural_height));
	menu->panelWidth = fminf(width, fmaxf(menu->panelWidth, natural_width * menu->scale));
	menu->panelHeight = fminf(height, fmaxf(menu->panelHeight, natural_height * menu->scale));
	menu->panelX = (width - menu->panelWidth) * 0.5f;
	menu->panelY = (height - menu->panelHeight) * 0.5f;
	menu->padding = MENU_PADDING * menu->scale;
	menu->lineHeight = (menu->panelHeight - 2.0f * menu->padding) / (content_lines(menu) + 1.5f);
	menu->lineHeight = clampf(menu->lineHeight, MENU_LINE_PIXELS * menu->scale, MENU_LINE_PIXELS * menu->scale * MENU_MAX_STRETCH);
}

// The spare width is shared by the used columns; an empty column gets no width and no gap.
static void	place_columns(const Menu *menu, float *column_widths, float *column_x)
{
	float	spare = menu->panelWidth - (float)(used_columns(menu) + 1) * menu->padding;
	float	next_x = menu->panelX + menu->padding;
	int		column;

	column = 0;
	while (column < MENU_COLUMNS)
	{
		column_widths[column] = column_width(menu, column) * menu->scale;
		spare -= column_widths[column];
		column++;
	}
	column = 0;
	while (column < MENU_COLUMNS)
	{
		column_x[column] = next_x;
		if (column_widths[column] > 0.0f)
		{
			column_widths[column] += fmaxf(0.0f, spare / (float)used_columns(menu));
			next_x += column_widths[column] + menu->padding;
		}
		column++;
	}
}

static void	place_row(const Menu *menu, MenuRow *row, float x, float y, float width)
{
	row->x = x;
	row->y = y;
	row->width = width;
	row->height = menu->lineHeight;
	row->buttonWidth = MENU_BUTTON_CHARS * FONT_ADVANCE * menu->scale;
	row->plusX = row->x + row->width - row->buttonWidth;
	row->minusX = row->plusX - row->buttonWidth - MENU_BUTTON_GAP * menu->scale;
}

void	menu_layout(Menu *menu, float width, float height)
{
	float	column_widths[MENU_COLUMNS];
	float	column_x[MENU_COLUMNS];
	float	column_y[MENU_COLUMNS];
	float	top;
	int		column;
	int		i;

	place_panel(menu, width, height);
	place_columns(menu, column_widths, column_x);
	top = menu->panelY + menu->padding + 1.5f * menu->lineHeight;
	column = 0;
	while (column < MENU_COLUMNS)
	{
		column_y[column] = top;
		column++;
	}
	i = 0;
	while (i < menu->rowCount)
	{
		column = menu->rows[i].column;
		if (menu->rows[i].kind == MENU_SECTION && column_y[column] > top)
			column_y[column] += 0.4f * menu->lineHeight;
		place_row(menu, &menu->rows[i], column_x[column], column_y[column], column_widths[column]);
		column_y[column] += menu->lineHeight;
		i++;
	}
}
