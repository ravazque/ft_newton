
#include "newton.h"

/*
 * [TODO] The simulation orchestrator. world_init/destroy/clear are provided
 * (memory housekeeping); world_add_body/remove_body/step are yours.
 * See docs/info.md 5 and 6.
*/

void	world_init(World *w)
{
	memset(w, 0, sizeof(*w));
	w->gravity = vec3(0.0f, GRAVITY_Y, 0.0f);
	w->solverIterations = SOLVER_ITERATIONS;
}

void	world_destroy(World *w)
{
	free(w->bodies);
	free(w->pairs);
	free(w->contacts);
	memset(w, 0, sizeof(*w));
}

int	world_add_body(World *w, RigidBody body)
{
	/* TODO: grow w->bodies when bodyCount == bodyCapacity (e.g. double the
	 * capacity with realloc), append 'body', and return its index (handle).
	 * This is the easy way to spawn MANY objects on demand. */
	(void)w;
	(void)body;
	return (-1);
}

void	world_remove_body(World *w, int handle)
{
	/* TODO: remove the body at 'handle' (swap-with-last is fine; note it
	 * changes indices). */
	(void)w;
	(void)handle;
}

void	world_clear(World *w)
{
	w->bodyCount = 0;
}

void	world_step(World *w, float dt)
{
	/* TODO: one fixed step of the full pipeline:
	 *   1) for each awake body: integrator_integrate(body, w->gravity, dt);
	 *      then rb_clear_accumulators(body)
	 *   2) broadphase_compute_pairs(w)
	 *   3) narrowphase_generate_contacts(w)
	 *   4) resolver_resolve(w, dt)
	 *   5) sleeping: rest bodies stop integrating (no infinite bounce)
	 */
	(void)w;
	(void)dt;
}
