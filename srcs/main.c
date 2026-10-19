#include "newton.h"

int	main(int argc, char *argv[])
{
	Game	game;

	if (!game_init(&game, argc, argv))
		return (1);
	game_run(&game);
	game_shutdown(&game);
	return (0);
}
