#include "newton.h"

// ragdoll.csv <-> RagdollDef: height, total mass and material of the figure; the layout of its parts is fixed.

static const char	*g_ragdoll_keys[] = {"height", "mass", "friction", "elasticity", "color"};

int	ragdoll_load(RagdollDef *ragdoll, const char *path)
{
	CsvFile	csv;

	memset(ragdoll, 0, sizeof(*ragdoll));
	if (!csv_load(&csv, path))
		return (0);
	csv_reject_unknown(&csv, g_ragdoll_keys, 5);
	ragdoll->height = csv_get_float(&csv, "height");
	ragdoll->mass = csv_get_float(&csv, "mass");
	ragdoll->friction = csv_get_float(&csv, "friction");
	ragdoll->elasticity = csv_get_float(&csv, "elasticity");
	ragdoll->color = csv_get_vec3(&csv, "color");
	if (csv.errors)
		return (0);
	csv_expect_range(&csv, "height", ragdoll->height, RAGDOLL_HEIGHT_MIN, RAGDOLL_HEIGHT_MAX);
	csv_expect_range(&csv, "mass", ragdoll->mass, RAGDOLL_MASS_MIN, RAGDOLL_MASS_MAX);
	csv_expect_range(&csv, "friction", ragdoll->friction, 0.0f, FRICTION_MAX);
	csv_expect_range(&csv, "elasticity", ragdoll->elasticity, 0.0f, 1.0f);
	csv_expect_range_vec3(&csv, "color", ragdoll->color, 0.0f, 1.0f);
	return (csv.errors == 0);
}

int	ragdoll_save(const RagdollDef *ragdoll, const char *path)
{
	FILE	*file = fopen(path, "w");

	if (!file)
		return (fprintf(stderr, "%s: cannot write file\n", path), 0);
	fprintf(file, "%s\n", CSV_HEADER);
	fprintf(file, "height,%g,,\nmass,%g,,\n", (double)ragdoll->height, (double)ragdoll->mass);
	fprintf(file, "friction,%g,,\nelasticity,%g,,\ncolor,%g,%g,%g\n", (double)ragdoll->friction, (double)ragdoll->elasticity, (double)ragdoll->color.x, (double)ragdoll->color.y, (double)ragdoll->color.z);
	if (fclose(file) != 0)
		return (fprintf(stderr, "%s: write failed\n", path), 0);
	return (1);
}
