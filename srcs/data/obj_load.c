#include "newton.h"

// Wavefront .obj reader: v and f lines kept in a first pass (other keywords skipped), faces resolved in a second.

static int	is_blank(char character)
{
	return (character == ' ' || character == '\t' || character == '\r' || character == '\v' || character == '\f');
}

char	*obj_token(char **cursor)
{
	char	*text = *cursor;
	char	*start;

	while (is_blank(*text))
		text++;
	if (*text == '\0')
	{
		*cursor = text;
		return (NULL);
	}
	start = text;
	while (*text && !is_blank(*text))
		text++;
	if (*text)
	{
		*text = '\0';
		text++;
	}
	*cursor = text;
	return (start);
}

int	obj_error(const ObjParser *parser, int line, const char *message, const char *token)
{
	fprintf(stderr, "%s", parser->path);
	if (line)
		fprintf(stderr, ":%d", line);
	fprintf(stderr, ": %s", message);
	if (token)
		fprintf(stderr, " '%s'", token);
	fprintf(stderr, "\n");
	return (0);
}

static void	join_continued_lines(char *text)
{
	while (*text)
	{
		if (text[0] == '\\' && text[1] == '\n')
			memset(text, ' ', 2);
		else if (text[0] == '\\' && text[1] == '\r' && text[2] == '\n')
			memset(text, ' ', 3);
		text++;
	}
}

static char	*cut_line(char *line)
{
	char	*next;
	char	*comment;

	next = strchr(line, '\n');
	if (next)
	{
		*next = '\0';
		next++;
	}
	else
		next = line + strlen(line);
	comment = strchr(line, '#');
	if (comment)
		*comment = '\0';
	return (next);
}

static int	parse_vertex(ObjMesh *obj, ObjParser *parser, char *cursor, int line)
{
	float	coordinates[3];
	char	*token;
	char	*end;
	Vec3	*grown;
	int		axis;

	axis = 0;
	while (axis < 3)
	{
		token = obj_token(&cursor);
		if (!token)
			return (obj_error(parser, line, "a vertex needs x, y and z", NULL));
		coordinates[axis] = strtof(token, &end);
		if (end == token || *end || !isfinite(coordinates[axis]))
			return (obj_error(parser, line, "invalid vertex coordinate", token));
		axis++;
	}
	if (obj->positionCount == INT_MAX)
		return (obj_error(parser, line, "too many vertices", NULL));
	grown = array_reserve(obj->positions, &obj->positionCapacity, obj->positionCount + 1, sizeof(Vec3));
	if (!grown)
		return (0);
	obj->positions = grown;
	obj->positions[obj->positionCount] = vec3(coordinates[0], coordinates[1], coordinates[2]);
	obj->positionCount++;
	return (1);
}

static int	keep_face(ObjMesh *obj, ObjParser *parser, char *cursor, int line)
{
	ObjFace	*grown;

	if (parser->faceCount == INT_MAX)
		return (obj_error(parser, line, "too many faces", NULL));
	grown = array_reserve(parser->faces, &parser->faceCapacity, parser->faceCount + 1, sizeof(ObjFace));
	if (!grown)
		return (0);
	parser->faces = grown;
	parser->faces[parser->faceCount].text = cursor;
	parser->faces[parser->faceCount].line = line;
	parser->faces[parser->faceCount].verticesBefore = obj->positionCount;
	parser->faceCount++;
	return (1);
}

static int	first_pass(ObjMesh *obj, ObjParser *parser, char *line)
{
	char	*next;
	char	*cursor;
	char	*keyword;
	int		line_number;
	int		ok;

	ok = 1;
	line_number = 1;
	while (ok && *line)
	{
		next = cut_line(line);
		cursor = line;
		keyword = obj_token(&cursor);
		if (keyword && strcmp(keyword, "v") == 0)
			ok = parse_vertex(obj, parser, cursor, line_number);
		else if (keyword && strcmp(keyword, "f") == 0)
			ok = keep_face(obj, parser, cursor, line_number);
		line = next;
		line_number++;
	}
	return (ok);
}

// A face with a doubled area under OBJ_DEGENERATE_EPSILON times the squared model diagonal is dropped.
static int	second_pass(ObjMesh *obj, ObjParser *parser)
{
	Vec3	bounds_min = obj->positions[0];
	Vec3	bounds_max = obj->positions[0];
	float	diagonal;
	int		i;

	i = 1;
	while (i < obj->positionCount)
	{
		bounds_min = vec3_min(bounds_min, obj->positions[i]);
		bounds_max = vec3_max(bounds_max, obj->positions[i]);
		i++;
	}
	diagonal = vec3_length(vec3_sub(bounds_max, bounds_min));
	obj->minArea = OBJ_DEGENERATE_EPSILON * diagonal * diagonal;
	i = 0;
	while (i < parser->faceCount)
	{
		if (!obj_parse_face(obj, parser, &parser->faces[i]))
			return (0);
		i++;
	}
	if (obj->triangleCount == 0)
		return (obj_error(parser, 0, "every face is degenerate (no area)", NULL));
	return (1);
}

int	obj_load(const char *path, ObjMesh *obj)
{
	ObjParser	parser;
	char		*text;
	int			ok;

	memset(obj, 0, sizeof(*obj));
	memset(&parser, 0, sizeof(parser));
	parser.path = path;
	text = file_read(path);
	if (!text)
		return (0);
	join_continued_lines(text);
	ok = first_pass(obj, &parser, text);
	if (ok && (parser.faceCount == 0 || obj->positionCount == 0))
		ok = obj_error(&parser, 0, "no faces", NULL);
	if (ok)
		ok = second_pass(obj, &parser);
	free(text);
	free(parser.faces);
	free(parser.polygon);
	free(parser.remaining);
	free(parser.projected);
	if (!ok)
		obj_free(obj);
	return (ok);
}

void	obj_free(ObjMesh *obj)
{
	free(obj->positions);
	free(obj->corners);
	memset(obj, 0, sizeof(*obj));
}
