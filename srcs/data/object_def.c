#include "newton.h"

// One object file <-> one ObjectDef: every property of its shape present, nothing else, every value in range.

static const char	*g_sphere_keys[] = {"shape", "radius", "mass", "friction", "elasticity", "rolling_resistance", "color"};
static const char	*g_box_keys[] = {"shape", "size", "mass", "friction", "elasticity", "color"};
static const char	*g_plane_keys[] = {"shape", "normal", "offset", "extent", "friction", "elasticity", "color"};
static const char	*g_mesh_keys[] = {"shape", "size", "density", "friction", "elasticity", "color"};

static int	parse_shape(const char *word, ShapeType *shape)
{
	if (strcmp(word, "sphere") == 0)
		*shape = SHAPE_SPHERE;
	else if (strcmp(word, "box") == 0)
		*shape = SHAPE_BOX;
	else if (strcmp(word, "plane") == 0)
		*shape = SHAPE_PLANE;
	else if (strcmp(word, "mesh") == 0)
		*shape = SHAPE_MESH;
	else
		return (0);
	return (1);
}

static void	read_shape_properties(ObjectDef *def, CsvFile *csv)
{
	if (def->shape == SHAPE_SPHERE)
	{
		csv_reject_unknown(csv, g_sphere_keys, 7);
		def->radius = csv_get_float(csv, "radius");
		def->mass = csv_get_float(csv, "mass");
		def->rollingResistance = csv_get_float(csv, "rolling_resistance");
	}
	else if (def->shape == SHAPE_BOX)
	{
		csv_reject_unknown(csv, g_box_keys, 6);
		def->size = csv_get_vec3(csv, "size");
		def->mass = csv_get_float(csv, "mass");
	}
	else if (def->shape == SHAPE_PLANE)
	{
		csv_reject_unknown(csv, g_plane_keys, 7);
		def->normal = csv_get_vec3(csv, "normal");
		def->offset = csv_get_float(csv, "offset");
		def->extent = csv_get_float(csv, "extent");
	}
	else
	{
		csv_reject_unknown(csv, g_mesh_keys, 6);
		def->modelSize = csv_get_float(csv, "size");
		def->density = csv_get_float(csv, "density");
	}
}

static void	validate(const ObjectDef *def, CsvFile *csv)
{
	if (def->shape == SHAPE_SPHERE)
	{
		csv_expect_range(csv, "radius", def->radius, PROJECTILE_RADIUS_MIN, BODY_SIZE_MAX * 0.5f);
		csv_expect_range(csv, "rolling_resistance", def->rollingResistance, 0.0f, ROLLING_RESISTANCE_MAX);
	}
	if (def->shape == SHAPE_BOX)
		csv_expect_range_vec3(csv, "size", def->size, BODY_SIZE_MIN, BODY_SIZE_MAX);
	if (def->shape == SHAPE_SPHERE || def->shape == SHAPE_BOX)
		csv_expect_range(csv, "mass", def->mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX);
	if (def->shape == SHAPE_PLANE)
	{
		csv_expect_positive(csv, "extent", def->extent);
		if (vec3_length_sq(def->normal) == 0.0f)
			csv_error(csv, "normal", "a non-zero vector");
	}
	if (def->shape == SHAPE_MESH)
	{
		csv_expect_range(csv, "size", def->modelSize, MODEL_SIZE_MIN, MODEL_SIZE_MAX);
		csv_expect_range(csv, "density", def->density, MODEL_DENSITY_MIN, MODEL_DENSITY_MAX);
	}
	csv_expect_range(csv, "friction", def->friction, 0.0f, FRICTION_MAX);
	csv_expect_range(csv, "elasticity", def->elasticity, 0.0f, 1.0f);
	csv_expect_range_vec3(csv, "color", def->color, 0.0f, 1.0f);
}

int	object_def_load(ObjectDef *def, const char *path, ObjectKind kind)
{
	CsvFile		csv;
	const char	*shape;

	memset(def, 0, sizeof(*def));
	def->kind = kind;
	if (!csv_load(&csv, path))
		return (0);
	shape = csv_get_word(&csv, "shape");
	if (csv.errors)
		return (0);
	if (!parse_shape(shape, &def->shape))
		return (fprintf(stderr, "%s: unknown shape '%s' (expected sphere, box, plane or mesh)\n", path, shape), 0);
	read_shape_properties(def, &csv);
	def->friction = csv_get_float(&csv, "friction");
	def->elasticity = csv_get_float(&csv, "elasticity");
	def->color = csv_get_vec3(&csv, "color");
	if (csv.errors)
		return (0);
	validate(def, &csv);
	if (csv.errors)
		return (0);
	def->normal = vec3_normalized(def->normal);
	return (1);
}

// A sphere, box or plane; a model body also needs its surface and is built by model_spawn.
RigidBody	object_def_make_body(const ObjectDef *def, Vec3 position)
{
	RigidBody	body = rigidbody_make();

	body.kind = def->kind;
	body.position = position;
	if (def->shape == SHAPE_SPHERE)
		body.collider = collider_sphere(def->radius);
	else if (def->shape == SHAPE_BOX)
		body.collider = collider_box(vec3_scale(def->size, 0.5f));
	else
		body.collider = collider_plane(def->normal, def->offset, def->extent);
	body.friction = def->friction;
	body.elasticity = def->elasticity;
	body.rollingResistance = def->rollingResistance;
	body.color = def->color;
	if (def->shape == SHAPE_PLANE)
		rigidbody_make_static(&body);
	else
		rigidbody_set_mass(&body, def->mass);
	return (body);
}

int	object_def_save(const ObjectDef *def, const char *path)
{
	FILE	*file = fopen(path, "w");

	if (!file)
		return (fprintf(stderr, "%s: cannot write file\n", path), 0);
	fprintf(file, "%s\n", CSV_HEADER);
	if (def->shape == SHAPE_SPHERE)
		fprintf(file, "shape,sphere,,\nradius,%g,,\nmass,%g,,\nrolling_resistance,%g,,\n", (double)def->radius, (double)def->mass, (double)def->rollingResistance);
	else if (def->shape == SHAPE_BOX)
		fprintf(file, "shape,box,,\nsize,%g,%g,%g\nmass,%g,,\n", (double)def->size.x, (double)def->size.y, (double)def->size.z, (double)def->mass);
	else if (def->shape == SHAPE_PLANE)
		fprintf(file, "shape,plane,,\nnormal,%g,%g,%g\noffset,%g,,\nextent,%g,,\n", (double)def->normal.x, (double)def->normal.y, (double)def->normal.z, (double)def->offset, (double)def->extent);
	else
		fprintf(file, "shape,mesh,,\nsize,%g,,\ndensity,%g,,\n", (double)def->modelSize, (double)def->density);
	fprintf(file, "friction,%g,,\nelasticity,%g,,\ncolor,%g,%g,%g\n", (double)def->friction, (double)def->elasticity, (double)def->color.x, (double)def->color.y, (double)def->color.z);
	if (fclose(file) != 0)
		return (fprintf(stderr, "%s: write failed\n", path), 0);
	return (1);
}
