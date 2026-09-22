#include "newton.h"

int	main(int argc, char *argv[])
{
	Game	g;

	if (!game_init(&g, argc, argv))
		return (1);
	game_run(&g);
	game_shutdown(&g);
	return (0);
}
