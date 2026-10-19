#include "newton.h"

// The menu in the 2D overlay: panel, title line with the page note, then every row in the rectangles of the layout.

static const Vec3	g_panel = {0.07f, 0.08f, 0.11f};
static const Vec3	g_panel_edge = {0.35f, 0.40f, 0.50f};
static const Vec3	g_section = {1.00f, 0.80f, 0.25f};
static const Vec3	g_label = {0.85f, 0.88f, 0.92f};
static const Vec3	g_value = {0.55f, 0.85f, 1.00f};
static const Vec3	g_hint = {0.62f, 0.66f, 0.72f};
static const Vec3	g_button = {0.18f, 0.22f, 0.30f};
static const Vec3	g_button_hot = {0.30f, 0.45f, 0.65f};
static const Vec3	g_selected = {0.14f, 0.18f, 0.26f};

// Top of a glyph centred in a row, so every text of the row shares one baseline.
static float	text_y(float y, float height, float scale)
{
	return (y + (height - (float)FONT_GLYPH_SIZE * scale) * 0.5f);
}

static void	format_value(const MenuRow *row, char *text, size_t size)
{
	if (row->decimals == 0)
		snprintf(text, size, "%.0f", (double)*row->value);
	else if (row->decimals == 1)
		snprintf(text, size, "%.1f", (double)*row->value);
	else
		snprintf(text, size, "%.2f", (double)*row->value);
}

static void	draw_button(Ui *ui, const MenuRow *row, float x, float width, const char *label, float scale, int hot)
{
	Vec3	fill = g_button;

	if (hot)
		fill = g_button_hot;
	ui_rect(ui, x, row->y + MENU_BUTTON_INSET * scale, width, row->height - 2.0f * MENU_BUTTON_INSET * scale, fill);
	ui_text(ui, label, x + (width - ui_text_width(label, scale)) * 0.5f, text_y(row->y, row->height, scale), scale, g_label);
}

static void	draw_value_row(const Menu *menu, Ui *ui, const MenuRow *row, int index)
{
	char	text[32];
	float	y = text_y(row->y, row->height, menu->scale);

	ui_text(ui, row->label, row->x + 3.0f * menu->scale, y, menu->scale, g_label);
	format_value(row, text, sizeof(text));
	ui_text_right(ui, text, row->minusX - MENU_BUTTON_GAP * menu->scale, y, menu->scale, g_value);
	draw_button(ui, row, row->minusX, row->buttonWidth, "-", menu->scale, menu->hoverRow == index && menu->hoverPart == 0);
	draw_button(ui, row, row->plusX, row->buttonWidth, "+", menu->scale, menu->hoverRow == index && menu->hoverPart == 1);
}

static void	draw_row(const Menu *menu, Ui *ui, int index)
{
	const MenuRow	*row = &menu->rows[index];
	float			y = text_y(row->y, row->height, menu->scale);

	if (menu->selected == index && menu_is_selectable(row))
		ui_rect(ui, row->x, row->y, row->width, row->height, g_selected);
	if (row->kind == MENU_SECTION)
		ui_text(ui, row->label, row->x, y, menu->scale, g_section);
	else if (row->kind == MENU_HINT)
		ui_text(ui, row->label, row->x + 3.0f * menu->scale, y, menu->scale, g_hint);
	else if (row->kind == MENU_VALUE)
		draw_value_row(menu, ui, row, index);
	else
		draw_button(ui, row, row->x, row->width, row->label, menu->scale, menu->hoverRow == index);
}

static void	draw_header(const Menu *menu, Ui *ui)
{
	const char	*title = MENU_TITLE;
	const char	*note = MENU_NOTE_BACK;
	Vec3		note_color = g_hint;
	float		y = text_y(menu->panelY + menu->padding, menu->lineHeight, menu->scale);

	if (menu->page == MENU_PAGE_MODELS)
	{
		title = MENU_MODELS_TITLE;
		note = MENU_NOTE_MODELS;
	}
	else if (menu->dirty)
	{
		note = MENU_NOTE_DIRTY;
		note_color = g_section;
	}
	ui_text(ui, title, menu->panelX + menu->padding, y, menu->scale, g_section);
	ui_text_right(ui, note, menu->panelX + menu->panelWidth - menu->padding, y, menu->scale, note_color);
}

void	menu_draw(const Menu *menu, Ui *ui)
{
	int	i;

	ui_rect(ui, menu->panelX, menu->panelY, menu->panelWidth, menu->panelHeight, g_panel);
	ui_border(ui, menu->panelX, menu->panelY, menu->panelWidth, menu->panelHeight, 2.0f * menu->scale, g_panel_edge);
	draw_header(menu, ui);
	i = 0;
	while (i < menu->rowCount)
	{
		draw_row(menu, ui, i);
		i++;
	}
}
