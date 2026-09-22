#include "newton.h"

/*
 * Strict CSV reader for the object files. A file is a header line
 *   property,value1,value2,value3
 * followed by one row per property: a name and then one number, three
 * numbers (a vector or a color) or one word (e.g. "shape,box,,"). Trailing
 * empty fields may be omitted, blank lines and CRLF endings are tolerated;
 * anything else is reported with its line number and the load fails.
 * Getters never fall back to a default: a missing or badly shaped property
 * is reported and counted in 'errors', so a consumer reads everything it
 * needs and checks the count once at the end (every problem shows at once).
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

/* Splits on commas in place, trimming each field. Returns the field count
 * capped at max (a count equal to max means "too many"). */
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

/* Plain decimal notation only: strtof would also accept "nan", "inf" and
 * hexadecimal, none of which is a sensible property value. Returns 1 for a
 * number, 0 for a token that is not numeric at all (a word), and -1 for one
 * made of numeric characters that still fails ("1.2.3", or an overflow to
 * infinity), which is always a mistake. */
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

/* The values of a row: the fields after the property, with the trailing
 * empty ones dropped. One number, three numbers, or one word. */
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

static const CsvEntry	*find(const CsvFile *c, const char *key)
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
	if (find(c, fields[0]))
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

/* Every property must be one the consumer knows: a misspelt name would
 * otherwise be silently ignored and its value replaced by nothing. */
int	csv_reject_unknown(CsvFile *c, const char **allowed, int allowed_count)
{
	int	i;
	int	j;
	int	known;

	i = 0;
	while (i < c->count)
	{
		known = 0;
		j = 0;
		while (j < allowed_count && !known)
			known = (strcmp(c->entries[i].key, allowed[j++]) == 0);
		if (!known)
		{
			fprintf(stderr, "%s:%d: unknown property '%s'\n", c->path, c->entries[i].line, c->entries[i].key);
			c->errors++;
		}
		i++;
	}
	return (c->errors == 0);
}

static const CsvEntry	*require(CsvFile *c, const char *key, int count, const char *shape)
{
	const CsvEntry	*e = find(c, key);

	if (!e)
	{
		fprintf(stderr, "%s: missing property '%s'\n", c->path, key);
		c->errors++;
		return (NULL);
	}
	if (e->count != count)
	{
		fprintf(stderr, "%s:%d: '%s' expects %s\n", c->path, e->line, key, shape);
		c->errors++;
		return (NULL);
	}
	return (e);
}

float	csv_get_float(CsvFile *c, const char *key)
{
	const CsvEntry	*e = require(c, key, 1, "one number");

	if (!e)
		return (0.0f);
	return (e->v[0]);
}

Vec3	csv_get_vec3(CsvFile *c, const char *key)
{
	const CsvEntry	*e = require(c, key, 3, "three numbers");

	if (!e)
		return (vec3(0.0f, 0.0f, 0.0f));
	return (vec3(e->v[0], e->v[1], e->v[2]));
}

const char	*csv_get_word(CsvFile *c, const char *key)
{
	const CsvEntry	*e = require(c, key, 0, "one word");

	if (!e)
		return ("");
	return (e->word);
}

/* Range checks share the error counter, so a consumer validates every value
 * and then decides once. The line number comes from the row that set it. */
void	csv_error(CsvFile *c, const char *key, const char *what)
{
	const CsvEntry	*e = find(c, key);
	int				line = 0;

	if (e)
		line = e->line;
	fprintf(stderr, "%s:%d: '%s' must be %s\n", c->path, line, key, what);
	c->errors++;
}

int	csv_expect_positive(CsvFile *c, const char *key, float v)
{
	if (v > 0.0f)
		return (1);
	return (csv_error(c, key, "positive"), 0);
}

int	csv_expect_min(CsvFile *c, const char *key, float v, float lo)
{
	char	what[64];

	if (v >= lo)
		return (1);
	snprintf(what, sizeof(what), ">= %g", (double)lo);
	return (csv_error(c, key, what), 0);
}

int	csv_expect_range(CsvFile *c, const char *key, float v, float lo, float hi)
{
	char	what[64];

	if (v >= lo && v <= hi)
		return (1);
	snprintf(what, sizeof(what), "within [%g, %g]", (double)lo, (double)hi);
	return (csv_error(c, key, what), 0);
}

int	csv_expect_positive_vec3(CsvFile *c, const char *key, Vec3 v)
{
	if (v.x > 0.0f && v.y > 0.0f && v.z > 0.0f)
		return (1);
	return (csv_error(c, key, "three positive numbers"), 0);
}

int	csv_expect_range_vec3(CsvFile *c, const char *key, Vec3 v, float lo, float hi)
{
	char	what[64];

	if (v.x >= lo && v.x <= hi && v.y >= lo && v.y <= hi && v.z >= lo && v.z <= hi)
		return (1);
	snprintf(what, sizeof(what), "three numbers within [%g, %g]", (double)lo, (double)hi);
	return (csv_error(c, key, what), 0);
}
