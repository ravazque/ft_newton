#include "newton.h"

// Typed values of a loaded object file: a missing, misshapen or out-of-range value is reported and counted in errors.

int	csv_reject_unknown(CsvFile *csv, const char **allowed, int allowed_count)
{
	int	known;
	int	entry;
	int	key;

	entry = 0;
	while (entry < csv->count)
	{
		known = 0;
		key = 0;
		while (key < allowed_count && !known)
		{
			known = (strcmp(csv->entries[entry].key, allowed[key]) == 0);
			key++;
		}
		if (!known)
		{
			fprintf(stderr, "%s:%d: unknown property '%s'\n", csv->path, csv->entries[entry].line, csv->entries[entry].key);
			csv->errors++;
		}
		entry++;
	}
	return (csv->errors == 0);
}

static const CsvEntry	*require(CsvFile *csv, const char *key, int value_count, const char *expected)
{
	const CsvEntry	*entry = csv_find(csv, key);

	if (!entry)
	{
		fprintf(stderr, "%s: missing property '%s'\n", csv->path, key);
		csv->errors++;
		return (NULL);
	}
	if (entry->valueCount != value_count)
	{
		fprintf(stderr, "%s:%d: '%s' expects %s\n", csv->path, entry->line, key, expected);
		csv->errors++;
		return (NULL);
	}
	return (entry);
}

float	csv_get_float(CsvFile *csv, const char *key)
{
	const CsvEntry	*entry = require(csv, key, 1, "one number");

	if (!entry)
		return (0.0f);
	return (entry->values[0]);
}

Vec3	csv_get_vec3(CsvFile *csv, const char *key)
{
	const CsvEntry	*entry = require(csv, key, 3, "three numbers");

	if (!entry)
		return (vec3(0.0f, 0.0f, 0.0f));
	return (vec3(entry->values[0], entry->values[1], entry->values[2]));
}

const char	*csv_get_word(CsvFile *csv, const char *key)
{
	const CsvEntry	*entry = require(csv, key, 0, "one word");

	if (!entry)
		return ("");
	return (entry->word);
}

void	csv_error(CsvFile *csv, const char *key, const char *expected)
{
	const CsvEntry	*entry = csv_find(csv, key);
	int				line = 0;

	if (entry)
		line = entry->line;
	fprintf(stderr, "%s:%d: '%s' must be %s\n", csv->path, line, key, expected);
	csv->errors++;
}

int	csv_expect_positive(CsvFile *csv, const char *key, float value)
{
	if (value > 0.0f)
		return (1);
	csv_error(csv, key, "positive");
	return (0);
}

int	csv_expect_min(CsvFile *csv, const char *key, float value, float low)
{
	char	expected[64];

	if (value >= low)
		return (1);
	snprintf(expected, sizeof(expected), ">= %g", (double)low);
	csv_error(csv, key, expected);
	return (0);
}

int	csv_expect_range(CsvFile *csv, const char *key, float value, float low, float high)
{
	char	expected[64];

	if (value >= low && value <= high)
		return (1);
	snprintf(expected, sizeof(expected), "within [%g, %g]", (double)low, (double)high);
	csv_error(csv, key, expected);
	return (0);
}

int	csv_expect_positive_vec3(CsvFile *csv, const char *key, Vec3 value)
{
	if (value.x > 0.0f && value.y > 0.0f && value.z > 0.0f)
		return (1);
	csv_error(csv, key, "three positive numbers");
	return (0);
}

int	csv_expect_range_vec3(CsvFile *csv, const char *key, Vec3 value, float low, float high)
{
	char	expected[64];

	if (value.x >= low && value.x <= high && value.y >= low && value.y <= high && value.z >= low && value.z <= high)
		return (1);
	snprintf(expected, sizeof(expected), "three numbers within [%g, %g]", (double)low, (double)high);
	csv_error(csv, key, expected);
	return (0);
}
