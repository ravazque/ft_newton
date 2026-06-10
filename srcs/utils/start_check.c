
#include "newton.h"

static int	is_number(const char *argv)
{
	int	i;

	i = 0;
	if (argv[i] == '\0')
		return (0);
	while (argv[i] != '\0')
	{
		if (argv[i] < '0' || argv[i] > '9')
			return (0);
		i++;
	}
	return (1);
}

static int	parse_dim(const char *s, int min, int max, int *clamped)
{
	int	v;
	int	i;

	v = 0;
	i = 0;
	while (s[i] != '\0')
	{
		v = v * 10 + (s[i] - '0');
		if (v > max)
			return (*clamped = 1, max);
		i++;
	}
	if (v < min)
		return (*clamped = 1, min);
	return (v);
}

static int	resolve_dim(const char *arg, int min, int max, int def, const char *label)
{
	int	value;
	int	clamped;

	if (!is_number(arg))
	{
		printf("%s '%s' is not a valid number; using default %d\n", label, arg, def);
		return (def);
	}
	clamped = 0;
	value = parse_dim(arg, min, max, &clamped);
	if (clamped)
	{
		printf("%s '%s' is out of range [%d..%d]; using default %d\n", label, arg, min, max, def);
		return (def);
	}
	return (value);
}

static void	set_resolution(Camera *camera, const char *w_arg, const char *h_arg)
{
	camera->win_width = resolve_dim(w_arg, WIN_WIDTH_MIN, WIN_WIDTH_MAX, WIN_WIDTH, "Width");
	camera->win_height = resolve_dim(h_arg, WIN_HEIGHT_MIN, WIN_HEIGHT_MAX, WIN_HEIGHT, "Height");
}

void	check_input(int argc, char **argv, Camera *camera)
{
	if (argc == 1)
		return ;
	else if (argc == 3)
		set_resolution(camera, argv[1], argv[2]);
	else if (argc == 4)
		return ;
	else
	{
		printf("Usage: %s <width> <height>\n", argv[0]);
		exit(1);
	}
}
