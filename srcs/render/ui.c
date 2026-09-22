#include "newton.h"

/*
 * Immediate-mode 2D drawing for the menu: rectangles and text, in window
 * pixels with the origin at the top-left corner.
 *
 * Everything is batched into one dynamic vertex buffer and sent to the GPU in
 * a single draw call per frame. A glyph pixel and a panel are the same thing
 * here - a colored quad - so the whole overlay costs one buffer upload and
 * one glDrawArrays, with the 3D shader reused in flat mode (uLit = 0) under
 * an orthographic projection.
*/

/* Screen pixels -> clip space. Maps x to [-1, 1] and flips y so that y grows
 * downward, which is how a text layout is naturally written. */
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

/* The one primitive everything else is made of: two triangles. The color
 * rides in the normal slot, so the vertex layout is the 3D one and the
 * fragment shader reads it as a flat color. */
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

/* A frame of four thin rectangles, used to outline panels and buttons. */
void	ui_border(Ui *ui, float x, float y, float w, float h, float t, Vec3 color)
{
	ui_rect(ui, x, y, w, t, color);
	ui_rect(ui, x, y + h - t, w, t, color);
	ui_rect(ui, x, y + t, t, h - 2.0f * t, color);
	ui_rect(ui, x + w - t, y + t, t, h - 2.0f * t, color);
}

/* Draws one glyph as one quad per set bit. 'scale' is the size of a font
 * pixel, so the text size is always a whole multiple of the 8x8 cell and
 * never blurs. */
static void	draw_glyph(Ui *ui, char c, float x, float y, float scale, Vec3 color)
{
	const unsigned char	*rows = font_glyph(c);
	int					row;
	int					col;

	row = 0;
	while (row < FONT_GLYPH_SIZE)
	{
		col = 0;
		while (col < FONT_GLYPH_SIZE)
		{
			if (rows[row] & (1u << col))
				ui_rect(ui, x + (float)col * scale, y + (float)row * scale, scale, scale, color);
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

/* Right-aligned text, for the numeric column of the menu. */
void	ui_text_right(Ui *ui, const char *text, float right, float y, float scale, Vec3 color)
{
	ui_text(ui, text, right - ui_text_width(text, scale), y, scale, color);
}

float	ui_text_width(const char *text, float scale)
{
	return ((float)strlen(text) * FONT_ADVANCE * scale);
}

/* Uploads the frame's quads and draws them over the scene: no depth test (the
 * overlay always wins) and the shader in flat mode. */
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
