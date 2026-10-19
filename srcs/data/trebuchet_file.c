#include "newton.h"

// trebuchet.csv <-> Trebuchet: geometry, launch settings and material; the launch values start inside the live ranges.

static const char	*g_trebuchet_keys[] = {
	"position", "base_size", "frame_height", "arm_length", "arm_thickness", "arm_angle", "counterweight_size",
	"launch_speed", "launch_angle", "friction", "elasticity", "color"};

static void	read_properties(Trebuchet *trebuchet, CsvFile *csv)
{
	trebuchet->position = csv_get_vec3(csv, "position");
	trebuchet->baseSize = csv_get_vec3(csv, "base_size");
	trebuchet->frameHeight = csv_get_float(csv, "frame_height");
	trebuchet->armLength = csv_get_float(csv, "arm_length");
	trebuchet->armThickness = csv_get_float(csv, "arm_thickness");
	trebuchet->armAngle = csv_get_float(csv, "arm_angle");
	trebuchet->counterweightSize = csv_get_vec3(csv, "counterweight_size");
	trebuchet->launchSpeed = csv_get_float(csv, "launch_speed");
	trebuchet->launchAngle = csv_get_float(csv, "launch_angle");
	trebuchet->friction = csv_get_float(csv, "friction");
	trebuchet->elasticity = csv_get_float(csv, "elasticity");
	trebuchet->color = csv_get_vec3(csv, "color");
}

static void	validate(const Trebuchet *trebuchet, CsvFile *csv)
{
	csv_expect_positive_vec3(csv, "base_size", trebuchet->baseSize);
	csv_expect_positive(csv, "frame_height", trebuchet->frameHeight);
	csv_expect_positive(csv, "arm_length", trebuchet->armLength);
	csv_expect_positive(csv, "arm_thickness", trebuchet->armThickness);
	csv_expect_range(csv, "arm_angle", trebuchet->armAngle, 0.0f, ARM_ANGLE_MAX);
	csv_expect_positive_vec3(csv, "counterweight_size", trebuchet->counterweightSize);
	csv_expect_range(csv, "launch_speed", trebuchet->launchSpeed, LAUNCH_SPEED_MIN, LAUNCH_SPEED_MAX);
	csv_expect_range(csv, "launch_angle", trebuchet->launchAngle, LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX);
	csv_expect_min(csv, "friction", trebuchet->friction, 0.0f);
	csv_expect_range(csv, "elasticity", trebuchet->elasticity, 0.0f, 1.0f);
	csv_expect_range_vec3(csv, "color", trebuchet->color, 0.0f, 1.0f);
	if (trebuchet->frameHeight <= trebuchet->baseSize.y)
		csv_error(csv, "frame_height", "above the sill, so the arm can turn");
}

// The projectile's mass and radius come from the apple definition.
int	trebuchet_load(Trebuchet *trebuchet, const char *path, const ObjectDef *apple)
{
	CsvFile	csv;

	memset(trebuchet, 0, sizeof(*trebuchet));
	if (!csv_load(&csv, path))
		return (0);
	csv_reject_unknown(&csv, g_trebuchet_keys, 12);
	read_properties(trebuchet, &csv);
	trebuchet->projectileMass = apple->mass;
	trebuchet->projectileRadius = apple->radius;
	if (csv.errors)
		return (0);
	validate(trebuchet, &csv);
	if (csv.errors)
		return (0);
	if (trebuchet->projectileMass < PROJECTILE_MASS_MIN)
	{
		fprintf(stderr, "%s: the projectile mass (%g, from %s) must be >= %g\n", path, (double)trebuchet->projectileMass, ASSET_APPLE, (double)PROJECTILE_MASS_MIN);
		return (0);
	}
	trebuchet->launchPoint = trebuchet_launch_point(trebuchet);
	trebuchet->spawnDistance = trebuchet_spawn_distance(trebuchet);
	return (1);
}

int	trebuchet_save(const Trebuchet *trebuchet, const char *path)
{
	FILE	*file = fopen(path, "w");

	if (!file)
		return (fprintf(stderr, "%s: cannot write file\n", path), 0);
	fprintf(file, "%s\n", CSV_HEADER);
	fprintf(file, "position,%g,%g,%g\n", (double)trebuchet->position.x, (double)trebuchet->position.y, (double)trebuchet->position.z);
	fprintf(file, "base_size,%g,%g,%g\n", (double)trebuchet->baseSize.x, (double)trebuchet->baseSize.y, (double)trebuchet->baseSize.z);
	fprintf(file, "frame_height,%g,,\n", (double)trebuchet->frameHeight);
	fprintf(file, "arm_length,%g,,\narm_thickness,%g,,\narm_angle,%g,,\n", (double)trebuchet->armLength, (double)trebuchet->armThickness, (double)trebuchet->armAngle);
	fprintf(file, "counterweight_size,%g,%g,%g\n", (double)trebuchet->counterweightSize.x, (double)trebuchet->counterweightSize.y, (double)trebuchet->counterweightSize.z);
	fprintf(file, "launch_speed,%g,,\nlaunch_angle,%g,,\n", (double)trebuchet->launchSpeed, (double)trebuchet->launchAngle);
	fprintf(file, "friction,%g,,\nelasticity,%g,,\ncolor,%g,%g,%g\n", (double)trebuchet->friction, (double)trebuchet->elasticity, (double)trebuchet->color.x, (double)trebuchet->color.y, (double)trebuchet->color.z);
	if (fclose(file) != 0)
		return (fprintf(stderr, "%s: write failed\n", path), 0);
	return (1);
}
