#include "newton.h"

/*
 * Strict reader of the object files: the header CSV_HEADER, then one row per
 * property holding one number, three numbers or one word ("shape,box,,").
 * Trailing empty fields may be left out, blank lines and CRLF are tolerated;
 * anything else stops the load with "file:line: reason" on stderr.
 * Reading typed values and range checks is csv_values.c.
*/

static int	fail(const CsvFile *c, int line, const char *what)
{
	fprintf(stderr, "%s:%d: %s\n", c->path, line, what);
	return (0);
}

static char	*trim(char *s)
{
	char	*end;

	while (*s == ' ' || *s == '\t')
		s++;
	end = s + strlen(s);
	while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
		end--;
	*end = '\0';
	return (s);
}

/* Splits on commas in place, trimming each field; a count equal to max means "too many". */
static int	split_fields(char *line, char **fields, int max)
{
	char	*comma;
	int		n;

	n = 0;
	while (n < max)
	{
		comma = strchr(line, ',');
		if (comma)
			*comma = '\0';
		fields[n++] = trim(line);
		if (!comma)
			return (n);
		line = comma + 1;
	}
	return (n);
}

/* Plain decimal only (strtof alone would take nan, inf and hex): 1 number, 0 word, -1
 * malformed or overflowing. */
static int	parse_number(const char *s, float *out)
{
	const char	*p = s;
	char		*end;

	if (*p == '\0')
		return (0);
	while (*p != '\0')
	{
		if (!isdigit((unsigned char)*p) && *p != '+' && *p != '-' && *p != '.' && *p != 'e' && *p != 'E')
			return (0);
		p++;
	}
	*out = strtof(s, &end);
	if (*end != '\0' || !isfinite(*out))
		return (-1);
	return (1);
}

/* The fields after the property, trailing empty ones dropped: one number, three numbers or one word. */
static int	parse_values(CsvFile *c, CsvEntry *e, char **fields, int n)
{
	int	i;
	int	parsed;

	while (n > 1 && fields[n - 1][0] == '\0')
		n--;
	n--;
	e->count = 0;
	e->word[0] = '\0';
	if (n == 0)
		return (fail(c, e->line, "missing value"));
	if (n == 2)
		return (fail(c, e->line, "expected one number, three numbers or one word"));
	i = 0;
	while (i < n)
	{
		if (fields[1 + i][0] == '\0')
			return (fail(c, e->line, "empty value between two values"));
		parsed = parse_number(fields[1 + i], &e->v[i]);
		if (parsed < 0)
			return (fail(c, e->line, "malformed number"));
		if (parsed == 0)
		{
			if (n != 1)
				return (fail(c, e->line, "expected three numbers"));
			strncpy(e->word, fields[1], CSV_KEY_LEN - 1);
			e->word[CSV_KEY_LEN - 1] = '\0';
			return (1);
		}
		i++;
	}
	e->count = n;
	return (1);
}

const CsvEntry	*csv_find(const CsvFile *c, const char *key)
{
	int	i;

	i = 0;
	while (i < c->count)
	{
		if (strcmp(c->entries[i].key, key) == 0)
			return (&c->entries[i]);
		i++;
	}
	return (NULL);
}

static int	parse_row(CsvFile *c, char *line, int lineno)
{
	char		*fields[5];
	char		what[CSV_KEY_LEN + 32];
	CsvEntry	*e;
	int			n;

	if (c->count == CSV_MAX_ENTRIES)
		return (fail(c, lineno, "too many rows"));
	e = &c->entries[c->count];
	e->line = lineno;
	n = split_fields(line, fields, 5);
	if (n == 5)
		return (fail(c, lineno, "expected at most 4 fields: property,value1,value2,value3"));
	if (fields[0][0] == '\0')
		return (fail(c, lineno, "missing property name"));
	if (strlen(fields[0]) >= CSV_KEY_LEN)
		return (fail(c, lineno, "property name too long"));
	if (csv_find(c, fields[0]))
		return (snprintf(what, sizeof(what), "duplicated property '%s'", fields[0]), fail(c, lineno, what));
	if (!parse_values(c, e, fields, n))
		return (0);
	strcpy(e->key, fields[0]);
	c->count++;
	return (1);
}

int	csv_load(CsvFile *c, const char *path)
{
	FILE	*f;
	char	line[CSV_LINE_LEN];
	int		lineno;

	c->count = 0;
	c->errors = 0;
	strncpy(c->path, path, CSV_PATH_LEN - 1);
	c->path[CSV_PATH_LEN - 1] = '\0';
	f = fopen(path, "r");
	if (!f)
		return (fprintf(stderr, "%s: cannot open this object file (%s)\n", path, strerror(errno)), 0);
	if (!fgets(line, sizeof(line), f) || strcmp(trim(line), CSV_HEADER) != 0)
		return (fclose(f), fail(c, 1, "first line must be the header '" CSV_HEADER "'"));
	lineno = 1;
	while (fgets(line, sizeof(line), f))
	{
		lineno++;
		if (strlen(line) == sizeof(line) - 1 && line[sizeof(line) - 2] != '\n')
			return (fclose(f), fail(c, lineno, "line too long"));
		if (trim(line)[0] == '\0')
			continue ;
		if (!parse_row(c, line, lineno))
			return (fclose(f), 0);
	}
	fclose(f);
	return (1);
}
