#include "newton.h"

/* Typed access to a loaded object file. Nothing falls back to a default: a missing, misshapen
 * or out-of-range value is reported with its line and counted in c->errors, so a loader reads
 * and checks everything, then decides once. */

/* A misspelt property would otherwise be silently ignored. */
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
	const CsvEntry	*e = csv_find(c, key);

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

void	csv_error(CsvFile *c, const char *key, const char *what)
{
	const CsvEntry	*e = csv_find(c, key);
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
