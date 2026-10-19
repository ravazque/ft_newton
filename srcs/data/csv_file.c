#include "newton.h"

// Strict reader of the object files: the CSV_HEADER line, then rows of one number, three numbers or one word.

static int	fail(const CsvFile *csv, int line, const char *reason)
{
	fprintf(stderr, "%s:%d: %s\n", csv->path, line, reason);
	return (0);
}

static char	*trim(char *text)
{
	char	*end;

	while (*text == ' ' || *text == '\t')
		text++;
	end = text + strlen(text);
	while (end > text && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
		end--;
	*end = '\0';
	return (text);
}

// In place, each field trimmed; a count equal to max_fields means the line has too many.
static int	split_fields(char *line, char **fields, int max_fields)
{
	char	*comma;
	int		count;

	count = 0;
	while (count < max_fields)
	{
		comma = strchr(line, ',');
		if (comma)
			*comma = '\0';
		fields[count] = trim(line);
		count++;
		if (!comma)
			return (count);
		line = comma + 1;
	}
	return (count);
}

static int	parse_number(const char *text, float *number)
{
	const char	*cursor = text;
	char		*end;

	if (*cursor == '\0')
		return (0);
	while (*cursor != '\0')
	{
		if (!isdigit((unsigned char)*cursor) && *cursor != '+' && *cursor != '-' && *cursor != '.' && *cursor != 'e' && *cursor != 'E')
			return (0);
		cursor++;
	}
	*number = strtof(text, &end);
	if (*end != '\0' || !isfinite(*number))
		return (-1);
	return (1);
}

static int	parse_values(CsvFile *csv, CsvEntry *entry, char **fields, int field_count)
{
	int	value_count;
	int	parsed;
	int	i;

	while (field_count > 1 && fields[field_count - 1][0] == '\0')
		field_count--;
	value_count = field_count - 1;
	entry->valueCount = 0;
	entry->word[0] = '\0';
	if (value_count == 0)
		return (fail(csv, entry->line, "missing value"));
	if (value_count == 2)
		return (fail(csv, entry->line, "expected one number, three numbers or one word"));
	i = 0;
	while (i < value_count)
	{
		if (fields[1 + i][0] == '\0')
			return (fail(csv, entry->line, "empty value between two values"));
		parsed = parse_number(fields[1 + i], &entry->values[i]);
		if (parsed < 0)
			return (fail(csv, entry->line, "malformed number"));
		if (parsed == 0 && value_count != 1)
			return (fail(csv, entry->line, "expected three numbers"));
		if (parsed == 0)
		{
			snprintf(entry->word, CSV_KEY_LEN, "%s", fields[1]);
			return (1);
		}
		i++;
	}
	entry->valueCount = value_count;
	return (1);
}

const CsvEntry	*csv_find(const CsvFile *csv, const char *key)
{
	int	i;

	i = 0;
	while (i < csv->count)
	{
		if (strcmp(csv->entries[i].key, key) == 0)
			return (&csv->entries[i]);
		i++;
	}
	return (NULL);
}

static int	parse_row(CsvFile *csv, char *line, int line_number)
{
	char		*fields[5];
	char		reason[CSV_KEY_LEN + 32];
	CsvEntry	*entry;
	int			field_count;

	if (csv->count == CSV_MAX_ENTRIES)
		return (fail(csv, line_number, "too many rows"));
	entry = &csv->entries[csv->count];
	entry->line = line_number;
	field_count = split_fields(line, fields, 5);
	if (field_count == 5)
		return (fail(csv, line_number, "expected at most 4 fields: property,value1,value2,value3"));
	if (fields[0][0] == '\0')
		return (fail(csv, line_number, "missing property name"));
	if (strlen(fields[0]) >= CSV_KEY_LEN)
		return (fail(csv, line_number, "property name too long"));
	if (csv_find(csv, fields[0]))
	{
		snprintf(reason, sizeof(reason), "duplicated property '%s'", fields[0]);
		return (fail(csv, line_number, reason));
	}
	if (!parse_values(csv, entry, fields, field_count))
		return (0);
	snprintf(entry->key, CSV_KEY_LEN, "%s", fields[0]);
	csv->count++;
	return (1);
}

int	csv_load(CsvFile *csv, const char *path)
{
	FILE	*file;
	char	line[CSV_LINE_LEN];
	int		line_number;

	csv->count = 0;
	csv->errors = 0;
	snprintf(csv->path, CSV_PATH_LEN, "%s", path);
	file = fopen(path, "r");
	if (!file)
		return (fprintf(stderr, "%s: cannot open this object file (%s)\n", path, strerror(errno)), 0);
	if (!fgets(line, sizeof(line), file) || strcmp(trim(line), CSV_HEADER) != 0)
		return (fclose(file), fail(csv, 1, "first line must be the header '" CSV_HEADER "'"));
	line_number = 1;
	while (fgets(line, sizeof(line), file))
	{
		line_number++;
		if (strlen(line) == sizeof(line) - 1 && line[sizeof(line) - 2] != '\n')
			return (fclose(file), fail(csv, line_number, "line too long"));
		if (trim(line)[0] != '\0' && !parse_row(csv, line, line_number))
			return (fclose(file), 0);
	}
	fclose(file);
	return (1);
}
