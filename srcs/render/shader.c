#include "newton.h"

// The GLSL program of shaders/: compiled, linked, and its uniforms set by name.

static unsigned int	compile_stage(unsigned int type, const char *source, const char *label)
{
	unsigned int	stage;
	int				compiled;
	char			log[1024];

	stage = glCreateShader(type);
	glShaderSource(stage, 1, &source, NULL);
	glCompileShader(stage);
	compiled = 0;
	glGetShaderiv(stage, GL_COMPILE_STATUS, &compiled);
	if (!compiled)
	{
		glGetShaderInfoLog(stage, sizeof(log), NULL, log);
		fprintf(stderr, "Shader compile error (%s):\n%s\n", label, log);
	}
	return (stage);
}

int	shader_load(Shader *shader, const char *vertex_path, const char *fragment_path)
{
	char			*vertex_source;
	char			*fragment_source;
	unsigned int	vertex_stage;
	unsigned int	fragment_stage;
	int				linked;
	char			log[1024];

	vertex_source = file_read(vertex_path);
	fragment_source = file_read(fragment_path);
	if (!vertex_source || !fragment_source)
		return (free(vertex_source), free(fragment_source), 0);
	vertex_stage = compile_stage(GL_VERTEX_SHADER, vertex_source, vertex_path);
	fragment_stage = compile_stage(GL_FRAGMENT_SHADER, fragment_source, fragment_path);
	free(vertex_source);
	free(fragment_source);
	shader->program = glCreateProgram();
	glAttachShader(shader->program, vertex_stage);
	glAttachShader(shader->program, fragment_stage);
	glLinkProgram(shader->program);
	linked = 0;
	glGetProgramiv(shader->program, GL_LINK_STATUS, &linked);
	if (!linked)
	{
		glGetProgramInfoLog(shader->program, sizeof(log), NULL, log);
		fprintf(stderr, "Shader link error:\n%s\n", log);
	}
	glDeleteShader(vertex_stage);
	glDeleteShader(fragment_stage);
	return (linked != 0);
}

void	shader_use(const Shader *shader)
{
	glUseProgram(shader->program);
}

void	shader_set_mat4(const Shader *shader, const char *name, Mat4 value)
{
	glUniformMatrix4fv(glGetUniformLocation(shader->program, name), 1, GL_FALSE, value.cells);
}

void	shader_set_vec3(const Shader *shader, const char *name, Vec3 value)
{
	glUniform3f(glGetUniformLocation(shader->program, name), value.x, value.y, value.z);
}

void	shader_set_int(const Shader *shader, const char *name, int value)
{
	glUniform1i(glGetUniformLocation(shader->program, name), value);
}
