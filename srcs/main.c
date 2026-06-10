
#include "newton.h"

int	main(int argc, char *argv[])
{
	Game g;

	if (game_init(&g, argc, argv))
		game_run(&g);
	else
		return (fprintf(stderr, "Error!\n"), 1);
	window_destroy(&g.window);
	return (0);
}
