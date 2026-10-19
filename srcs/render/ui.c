#include "newton.h"

// The 2D overlay, in window pixels from the top-left: every rectangle and glyph pixel batched into one draw call.

static Mat4	ui_projection(float width, float height)
{
	Mat4	projection = mat4_identity();

	projection.cells[0] = 2.0f / width;
	projection.cells[5] = -2.0f / height;
	projection.cells[12] = -1.0f;
	projection.cells[13] = 1.0f;
	return (projection);
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

// The colour travels in the normal slot, so the 3D vertex layout and shader serve the overlay too.
static void	push_vertex(Ui *ui, float x, float y, Vec3 color)
{
	float	*vertex;

	if (ui->vertexCount >= UI_MAX_VERTICES)
		return ;
	vertex = ui->vertices + (size_t)ui->vertexCount * UI_VERTEX_FLOATS;
	vertex[0] = x;
	vertex[1] = y;
	vertex[2] = 0.0f;
	vertex[3] = color.x;
	vertex[4] = color.y;
	vertex[5] = color.z;
	ui->vertexCount++;
}

void	ui_rect(Ui *ui, float x, float y, float width, float height, Vec3 color)
{
	if (ui->vertexCount + 6 > UI_MAX_VERTICES)
		return ;
	push_vertex(ui, x, y, color);
	push_vertex(ui, x + width, y, color);
	push_vertex(ui, x + width, y + height, color);
	push_vertex(ui, x, y, color);
	push_vertex(ui, x + width, y + height, color);
	push_vertex(ui, x, y + height, color);
}

void	ui_border(Ui *ui, float x, float y, float width, float height, float thickness, Vec3 color)
{
	ui_rect(ui, x, y, width, thickness, color);
	ui_rect(ui, x, y + height - thickness, width, thickness, color);
	ui_rect(ui, x, y + thickness, thickness, height - 2.0f * thickness, color);
	ui_rect(ui, x + width - thickness, y + thickness, thickness, height - 2.0f * thickness, color);
}

static void	draw_glyph(Ui *ui, char character, float x, float y, float scale, Vec3 color)
{
	const unsigned char	*rows = font_glyph(character);
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

void	ui_end(Ui *ui, Renderer *renderer)
{
	if (ui->vertexCount == 0)
		return ;
	glDisable(GL_DEPTH_TEST);
	shader_use(&renderer->shader);
	shader_set_mat4(&renderer->shader, "uProjection", ui_projection(ui->width, ui->height));
	shader_set_mat4(&renderer->shader, "uView", mat4_identity());
	shader_set_mat4(&renderer->shader, "uModel", mat4_identity());
	shader_set_int(&renderer->shader, "uLit", UI_FLAT_COLOR_MODE);
	glBindVertexArray(ui->vao);
	glBindBuffer(GL_ARRAY_BUFFER, ui->vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(sizeof(float) * UI_VERTEX_FLOATS * (size_t)ui->vertexCount), ui->vertices);
	glDrawArrays(GL_TRIANGLES, 0, ui->vertexCount);
	glBindVertexArray(0);
	glEnable(GL_DEPTH_TEST);
}
