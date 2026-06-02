
#include "newton.h"

/* Reads a whole text file into a malloc'd, NUL-terminated buffer. */
static char	*read_file(const char *path)
{
	FILE	*f;
	long	size;
	char	*buf;
	size_t	rd;

	f = fopen(path, "rb");
	if (!f)
		return (fprintf(stderr, "Shader: cannot open %s\n", path), NULL);
	fseek(f, 0, SEEK_END);
	size = ftell(f);
	fseek(f, 0, SEEK_SET);
	if (size < 0)
		return (fclose(f), NULL);
	buf = malloc((size_t)size + 1);
	if (!buf)
		return (fclose(f), NULL);
	rd = fread(buf, 1, (size_t)size, f);
	buf[rd] = '\0';
	fclose(f);
	return (buf);
}

static unsigned int	compile_stage(unsigned int type, const char *src, const char *label)
{
	unsigned int	shader;
	int				ok;
	char			log[1024];

	shader = glCreateShader(type);
	glShaderSource(shader, 1, &src, NULL);
	glCompileShader(shader);
	ok = 0;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
	if (!ok)
	{
		glGetShaderInfoLog(shader, sizeof(log), NULL, log);
		fprintf(stderr, "Shader compile error (%s):\n%s\n", label, log);
	}
	return (shader);
}

int	shader_load(Shader *sh, const char *vert_path, const char *frag_path)
{
	char			*vsrc;
	char			*fsrc;
	unsigned int	vs;
	unsigned int	fs;
	int				ok;

	vsrc = read_file(vert_path);
	fsrc = read_file(frag_path);
	if (!vsrc || !fsrc)
		return (free(vsrc), free(fsrc), 0);
	vs = compile_stage(GL_VERTEX_SHADER, vsrc, vert_path);
	fs = compile_stage(GL_FRAGMENT_SHADER, fsrc, frag_path);
	free(vsrc);
	free(fsrc);
	sh->program = glCreateProgram();
	glAttachShader(sh->program, vs);
	glAttachShader(sh->program, fs);
	glLinkProgram(sh->program);
	ok = 0;
	glGetProgramiv(sh->program, GL_LINK_STATUS, &ok);
	if (!ok)
	{
		char	log[1024];

		glGetProgramInfoLog(sh->program, sizeof(log), NULL, log);
		fprintf(stderr, "Shader link error:\n%s\n", log);
	}
	glDeleteShader(vs);
	glDeleteShader(fs);
	return (ok != 0);
}

void	shader_use(const Shader *sh)
{
	glUseProgram(sh->program);
}

void	shader_set_mat4(const Shader *sh, const char *name, Mat4 value)
{
	/* GL_FALSE: our matrices are already column-major, no transpose needed. */
	glUniformMatrix4fv(glGetUniformLocation(sh->program, name), 1, GL_FALSE, value.m);
}

void	shader_set_vec3(const Shader *sh, const char *name, Vec3 value)
{
	glUniform3f(glGetUniformLocation(sh->program, name), value.x, value.y, value.z);
}

void	shader_set_float(const Shader *sh, const char *name, float value)
{
	glUniform1f(glGetUniformLocation(sh->program, name), value);
}

void	shader_set_int(const Shader *sh, const char *name, int value)
{
	glUniform1i(glGetUniformLocation(sh->program, name), value);
}
