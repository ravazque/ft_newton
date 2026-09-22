#include "newton.h"

/*
 * What is in the world: the four object files loaded into definitions, the
 * starting scene built from them (ground, trebuchet, one pyramid), and the
 * material of existing bodies re-stamped when a definition changes. Files whose
 * starting scene would overlap or leave the pyramid off the ground are refused.
*/

static void	report_missing(void)
{
	fprintf(stderr, "%s, %s, %s and %s are all required and must be valid\n", ASSET_APPLE, ASSET_BLOCK, ASSET_GROUND, ASSET_TREBUCHET);
}

/* Builds the starting scene aside: the pyramid must not have to be lifted (it would sit inside the
 * trebuchet or the ground) and every block must stand over the ground square. */
int	scene_check_start(const ObjectDef *ground, const ObjectDef *block, const Trebuchet *trebuchet)
{
	World		w;
	Trebuchet	t = *trebuchet;
	float		lift;
	int			first;
	int			ok;

	world_init(&w);
	world_add_body(&w, objectdef_make_body(ground, vec3(0.0f, 0.0f, 0.0f)));
	trebuchet_build(&t, &w);
	first = w.bodyCount;
	lift = structure_spawn_pyramid(&w, block, vec3(STRUCTURE_X, 0.0f, 0.0f), START_PYRAMID_ROWS);
	ok = (lift == 0.0f);
	if (!ok)
		fprintf(stderr, "%s, %s, %s: the starting pyramid at x=%g would overlap the trebuchet or sink into the ground\n", ASSET_BLOCK, ASSET_TREBUCHET, ASSET_GROUND, (double)STRUCTURE_X);
	while (ok && first < w.bodyCount)
		ok = collider_plane_covers(&w.bodies[0].collider, w.bodies[first++].position);
	if (lift == 0.0f && !ok)
		fprintf(stderr, "%s, %s: the starting pyramid at x=%g is not over the ground square\n", ASSET_BLOCK, ASSET_GROUND, (double)STRUCTURE_X);
	world_destroy(&w);
	return (ok);
}

/* Loaded into temporaries first, so a broken file leaves the running definitions untouched. */
int	scene_load_assets(Game *g)
{
	ObjectDef	apple;
	ObjectDef	block;
	ObjectDef	ground;
	Trebuchet	trebuchet;

	if (!objectdef_load(&apple, ASSET_APPLE, KIND_APPLE) || !objectdef_load(&block, ASSET_BLOCK, KIND_BLOCK) || !objectdef_load(&ground, ASSET_GROUND, KIND_GROUND))
		return (report_missing(), 0);
	if (!trebuchet_load(&trebuchet, ASSET_TREBUCHET, &apple))
		return (report_missing(), 0);
	if (ground.shape != SHAPE_PLANE || apple.shape != SHAPE_SPHERE || block.shape != SHAPE_BOX)
		return (fprintf(stderr, "%s must be a plane, %s a sphere and %s a box\n", ASSET_GROUND, ASSET_APPLE, ASSET_BLOCK), 0);
	if (!scene_check_start(&ground, &block, &trebuchet))
		return (0);
	g->appleDef = apple;
	g->blockDef = block;
	g->groundDef = ground;
	g->trebuchetDef = trebuchet;
	return (1);
}

/* The starting scene; everything spawned or fired before is gone. */
void	scene_build(Game *g)
{
	world_clear(&g->world);
	g->trebuchet = g->trebuchetDef;
	world_add_body(&g->world, objectdef_make_body(&g->groundDef, vec3(0.0f, 0.0f, 0.0f)));
	trebuchet_build(&g->trebuchet, &g->world);
	structure_spawn_pyramid(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), START_PYRAMID_ROWS);
	g->paused = 0;
	g->hud.refresh = 1;
}

/* material = (friction, elasticity, rolling resistance); the mass goes through rb_set_mass so
 * the inertia follows. */
static void	restamp_bodies(World *w, ObjectKind kind, float mass, Vec3 material, Vec3 color)
{
	RigidBody	*b;
	int			i;

	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		if (b->kind == kind)
		{
			b->friction = material.x;
			b->elasticity = material.y;
			b->rollingResistance = material.z;
			b->color = color;
			if (b->invMass > 0.0f)
				rb_set_mass(b, mass);
		}
		i++;
	}
}

static Vec3	def_material(const ObjectDef *d)
{
	return (vec3(d->friction, d->elasticity, d->rollingResistance));
}

/* Mass and material (not geometry) of the current definitions onto every existing body. */
void	scene_apply_definitions(Game *g)
{
	Vec3	trebuchet_material = vec3(g->trebuchet.friction, g->trebuchet.elasticity, 0.0f);

	restamp_bodies(&g->world, KIND_APPLE, g->appleDef.mass, def_material(&g->appleDef), g->appleDef.color);
	restamp_bodies(&g->world, KIND_BLOCK, g->blockDef.mass, def_material(&g->blockDef), g->blockDef.color);
	restamp_bodies(&g->world, KIND_GROUND, 0.0f, def_material(&g->groundDef), g->groundDef.color);
	restamp_bodies(&g->world, KIND_TREBUCHET, 0.0f, trebuchet_material, g->trebuchet.color);
	g->trebuchet.projectileMass = g->appleDef.mass;
	g->trebuchet.projectileRadius = g->appleDef.radius;
	g->trebuchet.spawnDistance = trebuchet_spawn_distance(&g->trebuchet);
}
