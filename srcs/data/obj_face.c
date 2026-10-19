#include "newton.h"

// One f line: corners written v, v/vt, v//vn or v/vt/vn; only v is used, but every part must be well formed.

static int	skip_optional_index(const char *text, const char **end)
{
	char	*stop;

	(void)strtol(text, &stop, 10);
	*end = stop;
	return (stop != text);
}

static int	parse_corner(const char *token, long *index)
{
	char		*stop;
	const char	*cursor;

	errno = 0;
	*index = strtol(token, &stop, 10);
	if (stop == token || errno == ERANGE)
		return (0);
	if (*stop == '\0')
		return (1);
	if (*stop != '/')
		return (0);
	cursor = stop + 1;
	if (*cursor != '/')
	{
		if (!skip_optional_index(cursor, &cursor))
			return (0);
		if (*cursor == '\0')
			return (1);
		if (*cursor != '/')
			return (0);
	}
	return (skip_optional_index(cursor + 1, &cursor) && *cursor == '\0');
}

// 1..n count from the start of the file, -1 and below count back from the face's own line.
static int	resolve_index(const ObjMesh *obj, const ObjFace *face, long index, int *vertex)
{
	long	back;

	if (index > 0 && index <= (long)obj->positionCount)
	{
		*vertex = (int)(index - 1);
		return (1);
	}
	if (index >= 0)
		return (0);
	back = -index;
	if (back > (long)face->verticesBefore)
		return (0);
	*vertex = (int)(face->verticesBefore - back);
	return (1);
}

static int	add_corner(ObjMesh *obj, ObjParser *parser, const ObjFace *face, const char *token)
{
	int		*grown;
	long	index;

	grown = array_reserve(parser->polygon, &parser->polygonCapacity, parser->polygonCount + 1, sizeof(int));
	if (!grown)
		return (0);
	parser->polygon = grown;
	if (!parse_corner(token, &index))
		return (obj_error(parser, face->line, "invalid face corner", token));
	if (!resolve_index(obj, face, index, &parser->polygon[parser->polygonCount]))
		return (obj_error(parser, face->line, "vertex index out of range", token));
	parser->polygonCount++;
	return (1);
}

int	obj_parse_face(ObjMesh *obj, ObjParser *parser, const ObjFace *face)
{
	char	*cursor = face->text;
	char	*token;

	parser->polygonCount = 0;
	token = obj_token(&cursor);
	while (token)
	{
		if (!add_corner(obj, parser, face, token))
			return (0);
		token = obj_token(&cursor);
	}
	if (parser->polygonCount < 3)
		return (obj_error(parser, face->line, "a face needs 3 vertices", NULL));
	return (obj_triangulate(obj, parser, parser->polygonCount));
}
