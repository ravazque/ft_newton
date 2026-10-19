#include "newton.h"

// The optional [width height] arguments: digits only and within the window bounds, otherwise the default size.

static int	is_number(const char *argument)
{
	int	i;

	if (argument[0] == '\0')
		return (0);
	i = 0;
	while (argument[i] != '\0')
	{
		if (argument[i] < '0' || argument[i] > '9')
			return (0);
		i++;
	}
	return (1);
}

// Stops reading as soon as the value passes the maximum, so a long run of digits cannot overflow.
static int	within_bounds(const char *digits, int min_value, int max_value, int *value)
{
	int	i;

	*value = 0;
	i = 0;
	while (digits[i] != '\0')
	{
		*value = *value * 10 + (digits[i] - '0');
		if (*value > max_value)
			return (0);
		i++;
	}
	return (*value >= min_value);
}

static int	read_dimension(const char *argument, int min_value, int max_value, int fallback, const char *label)
{
	int	value;

	if (!is_number(argument))
	{
		fprintf(stderr, "%s '%s' is not a valid number; using default %d\n", label, argument, fallback);
		return (fallback);
	}
	if (!within_bounds(argument, min_value, max_value, &value))
	{
		fprintf(stderr, "%s '%s' is out of range [%d..%d]; using default %d\n", label, argument, min_value, max_value, fallback);
		return (fallback);
	}
	return (value);
}

void	arguments_read(int argc, char **argv, Camera *camera)
{
	if (argc == 1)
		return ;
	if (argc == 3)
	{
		camera->windowWidth = read_dimension(argv[1], WIN_WIDTH_MIN, WIN_WIDTH_MAX, WIN_WIDTH, "Width");
		camera->windowHeight = read_dimension(argv[2], WIN_HEIGHT_MIN, WIN_HEIGHT_MAX, WIN_HEIGHT, "Height");
		return ;
	}
	fprintf(stderr, "Usage: %s [width height]\n", argv[0]);
	exit(1);
}
