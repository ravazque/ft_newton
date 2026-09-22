#include "newton.h"

/* Draws the menu into the 2D overlay with the rectangles menu_layout computed: panel, title
 * line (which turns into the "unapplied edits" reminder), then every row. */

static const Vec3	g_panel = {0.07f, 0.08f, 0.11f};
static const Vec3	g_panel_edge = {0.35f, 0.40f, 0.50f};
static const Vec3	g_section = {1.00f, 0.80f, 0.25f};
static const Vec3	g_label = {0.85f, 0.88f, 0.92f};
static const Vec3	g_value = {0.55f, 0.85f, 1.00f};
static const Vec3	g_hint = {0.62f, 0.66f, 0.72f};
static const Vec3	g_button = {0.18f, 0.22f, 0.30f};
static const Vec3	g_button_hot = {0.30f, 0.45f, 0.65f};
static const Vec3	g_selected = {0.14f, 0.18f, 0.26f};

/* Top of a glyph vertically centred in a row, so every text of a row shares one baseline. */
static float	text_y(float y, float h, float scale)
{
	return (y + (h - (float)FONT_GLYPH_SIZE * scale) * 0.5f);
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

static void	draw_button(Ui *ui, const MenuRow *row, float x, float w, const char *label, float scale, int hot)
{
	Vec3	fill = g_button;

	if (hot)
		fill = g_button_hot;
	ui_rect(ui, x, row->y + MENU_BUTTON_INSET * scale, w, row->h - 2.0f * MENU_BUTTON_INSET * scale, fill);
	ui_text(ui, label, x + (w - ui_text_width(label, scale)) * 0.5f, text_y(row->y, row->h, scale), scale, g_label);
}

static void	draw_value_row(const Menu *m, Ui *ui, const MenuRow *row, int index)
{
	char	text[32];
	float	y = text_y(row->y, row->h, m->scale);

	ui_text(ui, row->label, row->x + 3.0f * m->scale, y, m->scale, g_label);
	format_value(row, text, sizeof(text));
	ui_text_right(ui, text, row->minusX - MENU_BUTTON_GAP * m->scale, y, m->scale, g_value);
	draw_button(ui, row, row->minusX, row->buttonW, "-", m->scale, m->hoverRow == index && m->hoverPart == 0);
	draw_button(ui, row, row->plusX, row->buttonW, "+", m->scale, m->hoverRow == index && m->hoverPart == 1);
}

static void	draw_row(const Menu *m, Ui *ui, int index)
{
	const MenuRow	*row = &m->rows[index];
	float			y = text_y(row->y, row->h, m->scale);

	if (m->selected == index && menu_is_selectable(row))
		ui_rect(ui, row->x, row->y, row->w, row->h, g_selected);
	if (row->kind == MENU_SECTION)
		ui_text(ui, row->label, row->x, y, m->scale, g_section);
	else if (row->kind == MENU_HINT)
		ui_text(ui, row->label, row->x + 3.0f * m->scale, y, m->scale, g_hint);
	else if (row->kind == MENU_VALUE)
		draw_value_row(m, ui, row, index);
	else
		draw_button(ui, row, row->x, row->w, row->label, m->scale, m->hoverRow == index);
}

static void	draw_header(const Menu *m, Ui *ui)
{
	const char	*note = MENU_NOTE_BACK;
	Vec3		color = g_hint;
	float		y = text_y(m->panelY + m->pad, m->line, m->scale);

	if (m->dirty)
	{
		note = MENU_NOTE_DIRTY;
		color = g_section;
	}
	ui_text(ui, MENU_TITLE, m->panelX + m->pad, y, m->scale, g_section);
	ui_text_right(ui, note, m->panelX + m->panelW - m->pad, y, m->scale, color);
}

void	menu_draw(const Menu *m, Ui *ui)
{
	int	i;

	ui_rect(ui, m->panelX, m->panelY, m->panelW, m->panelH, g_panel);
	ui_border(ui, m->panelX, m->panelY, m->panelW, m->panelH, 2.0f * m->scale, g_panel_edge);
	draw_header(m, ui);
	i = 0;
	while (i < m->rowCount)
	{
		draw_row(m, ui, i);
		i++;
	}
}
