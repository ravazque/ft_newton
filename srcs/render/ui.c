#include "newton.h"

/*
 * 2D overlay for the menu, in window pixels from the top-left corner. Panels and
 * glyph pixels are all colored quads batched in one vertex buffer and drawn with
 * a single glDrawArrays, reusing the 3D shader in per-vertex color mode under an
 * orthographic projection.
*/

/* Window pixels -> clip space, y flipped so it grows downward like a page. */
static Mat4	ui_projection(float width, float height)
{
	Mat4	m = mat4_identity();

	m.m[0] = 2.0f / width;
	m.m[5] = -2.0f / height;
	m.m[12] = -1.0f;
	m.m[13] = 1.0f;
	return (m);
}

int	ui_init(Ui *ui)
{
	GLsizei	stride;

	memset(ui, 0, sizeof(*ui));
	ui->vertices = malloc(sizeof(float) * UI_VERTEX_FLOATS * UI_MAX_VERTICES);
	if (!ui->vertices)
		return (fprintf(stderr, "Ui: out of memory\n"), 0);
	glGenVertexArrays(1, &ui->vao);
	glGenBuffers(1, &ui->vbo);
	glBindVertexArray(ui->vao);
	glBindBuffer(GL_ARRAY_BUFFER, ui->vbo);
	glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(sizeof(float) * UI_VERTEX_FLOATS * UI_MAX_VERTICES), NULL, GL_DYNAMIC_DRAW);
	stride = UI_VERTEX_FLOATS * (GLsizei)sizeof(float);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void *)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void *)(3 * sizeof(float)));
	glBindVertexArray(0);
	return (1);
}

void	ui_destroy(Ui *ui)
{
	free(ui->vertices);
	ui->vertices = NULL;
	if (ui->vbo)
		glDeleteBuffers(1, &ui->vbo);
	if (ui->vao)
		glDeleteVertexArrays(1, &ui->vao);
	ui->vbo = 0;
	ui->vao = 0;
}

void	ui_begin(Ui *ui, float width, float height)
{
	ui->vertexCount = 0;
	ui->width = width;
	ui->height = height;
}

static void	push_vertex(Ui *ui, float x, float y, Vec3 color)
{
	float	*v;

	if (ui->vertexCount >= UI_MAX_VERTICES)
		return ;
	v = ui->vertices + (size_t)ui->vertexCount * UI_VERTEX_FLOATS;
	v[0] = x;
	v[1] = y;
	v[2] = 0.0f;
	v[3] = color.x;
	v[4] = color.y;
	v[5] = color.z;
	ui->vertexCount++;
}

/* Two triangles; the color rides in the normal slot so the 3D vertex layout is reused. */
void	ui_rect(Ui *ui, float x, float y, float w, float h, Vec3 color)
{
	if (ui->vertexCount + 6 > UI_MAX_VERTICES)
		return ;
	push_vertex(ui, x, y, color);
	push_vertex(ui, x + w, y, color);
	push_vertex(ui, x + w, y + h, color);
	push_vertex(ui, x, y, color);
	push_vertex(ui, x + w, y + h, color);
	push_vertex(ui, x, y + h, color);
}

void	ui_border(Ui *ui, float x, float y, float w, float h, float t, Vec3 color)
{
	ui_rect(ui, x, y, w, t, color);
	ui_rect(ui, x, y + h - t, w, t, color);
	ui_rect(ui, x, y + t, t, h - 2.0f * t, color);
	ui_rect(ui, x + w - t, y + t, t, h - 2.0f * t, color);
}

/* One quad per set bit of the 8x8 glyph. Edges are snapped to whole screen
 * pixels, so any scale stays sharp and every copy of a letter looks the same. */
static void	draw_glyph(Ui *ui, char c, float x, float y, float scale, Vec3 color)
{
	const unsigned char	*rows = font_glyph(c);
	float				left;
	float				top;
	int					row;
	int					col;

	x = roundf(x);
	y = roundf(y);
	row = 0;
	while (row < FONT_GLYPH_SIZE)
	{
		top = roundf((float)row * scale);
		col = 0;
		while (col < FONT_GLYPH_SIZE)
		{
			left = roundf((float)col * scale);
			if (rows[row] & (1u << col))
				ui_rect(ui, x + left, y + top, roundf((float)(col + 1) * scale) - left, roundf((float)(row + 1) * scale) - top, color);
			col++;
		}
		row++;
	}
}

void	ui_text(Ui *ui, const char *text, float x, float y, float scale, Vec3 color)
{
	int	i;

	i = 0;
	while (text[i])
	{
		if (text[i] != ' ')
			draw_glyph(ui, text[i], x + (float)i * FONT_ADVANCE * scale, y, scale, color);
		i++;
	}
}

void	ui_text_right(Ui *ui, const char *text, float right, float y, float scale, Vec3 color)
{
	ui_text(ui, text, right - ui_text_width(text, scale), y, scale, color);
}

float	ui_text_width(const char *text, float scale)
{
	return ((float)strlen(text) * FONT_ADVANCE * scale);
}

/* Uploads the batch and draws it over the scene, depth test off. */
void	ui_end(Ui *ui, Renderer *r)
{
	if (ui->vertexCount == 0)
		return ;
	glDisable(GL_DEPTH_TEST);
	shader_use(&r->shader);
	shader_set_mat4(&r->shader, "uProjection", ui_projection(ui->width, ui->height));
	shader_set_mat4(&r->shader, "uView", mat4_identity());
	shader_set_mat4(&r->shader, "uModel", mat4_identity());
	shader_set_int(&r->shader, "uLit", UI_FLAT_COLOR_MODE);
	glBindVertexArray(ui->vao);
	glBindBuffer(GL_ARRAY_BUFFER, ui->vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(sizeof(float) * UI_VERTEX_FLOATS * (size_t)ui->vertexCount), ui->vertices);
	glDrawArrays(GL_TRIANGLES, 0, ui->vertexCount);
	glBindVertexArray(0);
	glEnable(GL_DEPTH_TEST);
}
