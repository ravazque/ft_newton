#include "newton.h"

/*
 * trebuchet.csv <-> Trebuchet: the machine's geometry, its launch settings and its
 * material. Strict like every object file, and the launch values must start inside
 * the range the live controls keep them in.
*/

static const char	*g_trebuchet_keys[] = {
	"position", "base_size", "frame_height", "arm_length", "arm_thickness", "arm_angle", "counterweight_size",
	"launch_speed", "launch_angle", "friction", "elasticity", "color"};

static void	read_properties(Trebuchet *t, CsvFile *csv)
{
	t->position = csv_get_vec3(csv, "position");
	t->baseSize = csv_get_vec3(csv, "base_size");
	t->frameHeight = csv_get_float(csv, "frame_height");
	t->armLength = csv_get_float(csv, "arm_length");
	t->armThickness = csv_get_float(csv, "arm_thickness");
	t->armAngle = csv_get_float(csv, "arm_angle");
	t->counterweightSize = csv_get_vec3(csv, "counterweight_size");
	t->launchSpeed = csv_get_float(csv, "launch_speed");
	t->launchAngle = csv_get_float(csv, "launch_angle");
	t->friction = csv_get_float(csv, "friction");
	t->elasticity = csv_get_float(csv, "elasticity");
	t->color = csv_get_vec3(csv, "color");
}

static void	validate(const Trebuchet *t, CsvFile *csv)
{
	csv_expect_positive_vec3(csv, "base_size", t->baseSize);
	csv_expect_positive(csv, "frame_height", t->frameHeight);
	csv_expect_positive(csv, "arm_length", t->armLength);
	csv_expect_positive(csv, "arm_thickness", t->armThickness);
	csv_expect_range(csv, "arm_angle", t->armAngle, 0.0f, ARM_ANGLE_MAX);
	csv_expect_positive_vec3(csv, "counterweight_size", t->counterweightSize);
	csv_expect_range(csv, "launch_speed", t->launchSpeed, LAUNCH_SPEED_MIN, LAUNCH_SPEED_MAX);
	csv_expect_range(csv, "launch_angle", t->launchAngle, LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX);
	csv_expect_min(csv, "friction", t->friction, 0.0f);
	csv_expect_range(csv, "elasticity", t->elasticity, 0.0f, 1.0f);
	csv_expect_range_vec3(csv, "color", t->color, 0.0f, 1.0f);
	if (t->frameHeight <= t->baseSize.y)
		csv_error(csv, "frame_height", "above the sill, so the arm can turn");
}

/* The apple definition gives the projectile's mass and radius (the radius sets how far apples
 * spawn from the arm). */
int	trebuchet_load(Trebuchet *t, const char *path, const ObjectDef *apple)
{
	CsvFile	csv;

	memset(t, 0, sizeof(*t));
	if (!csv_load(&csv, path))
		return (0);
	csv_reject_unknown(&csv, g_trebuchet_keys, 12);
	read_properties(t, &csv);
	t->projectileMass = apple->mass;
	t->projectileRadius = apple->radius;
	if (csv.errors)
		return (0);
	validate(t, &csv);
	if (csv.errors)
		return (0);
	if (t->projectileMass < PROJECTILE_MASS_MIN)
		return (fprintf(stderr, "%s: the projectile mass (%g, from %s) must be >= %g\n", path, (double)t->projectileMass, ASSET_APPLE, (double)PROJECTILE_MASS_MIN), 0);
	t->launchPoint = trebuchet_launch_point(t);
	t->spawnDistance = trebuchet_spawn_distance(t);
	return (1);
}

/* Same layout as the shipped file; the launch values written are the ones in effect. */
int	trebuchet_save(const Trebuchet *t, const char *path)
{
	FILE	*f = fopen(path, "w");

	if (!f)
		return (fprintf(stderr, "%s: cannot write file\n", path), 0);
	fprintf(f, "%s\n", CSV_HEADER);
	fprintf(f, "position,%g,%g,%g\n", (double)t->position.x, (double)t->position.y, (double)t->position.z);
	fprintf(f, "base_size,%g,%g,%g\n", (double)t->baseSize.x, (double)t->baseSize.y, (double)t->baseSize.z);
	fprintf(f, "frame_height,%g,,\n", (double)t->frameHeight);
	fprintf(f, "arm_length,%g,,\narm_thickness,%g,,\narm_angle,%g,,\n", (double)t->armLength, (double)t->armThickness, (double)t->armAngle);
	fprintf(f, "counterweight_size,%g,%g,%g\n", (double)t->counterweightSize.x, (double)t->counterweightSize.y, (double)t->counterweightSize.z);
	fprintf(f, "launch_speed,%g,,\nlaunch_angle,%g,,\n", (double)t->launchSpeed, (double)t->launchAngle);
	fprintf(f, "friction,%g,,\nelasticity,%g,,\ncolor,%g,%g,%g\n", (double)t->friction, (double)t->elasticity, (double)t->color.x, (double)t->color.y, (double)t->color.z);
	if (fclose(f) != 0)
		return (fprintf(stderr, "%s: write failed\n", path), 0);
	return (1);
}
