#include "newton.h"

/*
 * One object file (apples, block, ground) <-> one ObjectDef <-> the bodies built
 * from it. Loading is strict: every property of the shape must be present,
 * nothing else may appear and every value must be in its physical range.
*/

static const char	*g_sphere_keys[] = {"shape", "radius", "mass", "friction", "elasticity", "rolling_resistance", "color"};
static const char	*g_box_keys[] = {"shape", "size", "mass", "friction", "elasticity", "color"};
static const char	*g_plane_keys[] = {"shape", "normal", "offset", "extent", "friction", "elasticity", "color"};

static int	parse_shape(const char *word, ShapeType *out)
{
	if (strcmp(word, "sphere") == 0)
		*out = SHAPE_SPHERE;
	else if (strcmp(word, "box") == 0)
		*out = SHAPE_BOX;
	else if (strcmp(word, "plane") == 0)
		*out = SHAPE_PLANE;
	else
		return (0);
	return (1);
}

/* Only a sphere rolls, so only it has a rolling resistance; a plane is static, so it has no mass. */
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
	else
	{
		csv_reject_unknown(csv, g_plane_keys, 7);
		def->normal = csv_get_vec3(csv, "normal");
		def->offset = csv_get_float(csv, "offset");
		def->extent = csv_get_float(csv, "extent");
	}
}

static void	validate(const ObjectDef *def, CsvFile *csv)
{
	if (def->shape == SHAPE_SPHERE)
		csv_expect_positive(csv, "radius", def->radius);
	if (def->shape == SHAPE_SPHERE)
		csv_expect_range(csv, "rolling_resistance", def->rollingResistance, 0.0f, ROLLING_RESISTANCE_MAX);
	if (def->shape == SHAPE_BOX)
		csv_expect_positive_vec3(csv, "size", def->size);
	if (def->shape == SHAPE_PLANE)
		csv_expect_positive(csv, "extent", def->extent);
	if (def->shape == SHAPE_PLANE && vec3_length_sq(def->normal) == 0.0f)
		csv_error(csv, "normal", "a non-zero vector");
	if (def->shape != SHAPE_PLANE)
		csv_expect_positive(csv, "mass", def->mass);
	csv_expect_min(csv, "friction", def->friction, 0.0f);
	csv_expect_range(csv, "elasticity", def->elasticity, 0.0f, 1.0f);
	csv_expect_range_vec3(csv, "color", def->color, 0.0f, 1.0f);
}

int	objectdef_load(ObjectDef *def, const char *path, ObjectKind kind)
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
		return (fprintf(stderr, "%s: unknown shape '%s' (expected sphere, box or plane)\n", path, shape), 0);
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

RigidBody	objectdef_make_body(const ObjectDef *def, Vec3 position)
{
	RigidBody	b;

	b = rb_make();
	b.kind = def->kind;
	b.position = position;
	if (def->shape == SHAPE_SPHERE)
		b.collider = collider_sphere(def->radius);
	else if (def->shape == SHAPE_BOX)
		b.collider = collider_box(vec3_scale(def->size, 0.5f));
	else
		b.collider = collider_plane(def->normal, def->offset, def->extent);
	b.friction = def->friction;
	b.elasticity = def->elasticity;
	b.rollingResistance = def->rollingResistance;
	b.color = def->color;
	if (def->shape == SHAPE_PLANE)
		rb_make_static(&b);
	else
		rb_set_mass(&b, def->mass);
	return (b);
}

/* Same layout as the shipped files: the shape's own properties, then the shared material. */
int	objectdef_save(const ObjectDef *def, const char *path)
{
	FILE	*f = fopen(path, "w");

	if (!f)
		return (fprintf(stderr, "%s: cannot write file\n", path), 0);
	fprintf(f, "%s\n", CSV_HEADER);
	if (def->shape == SHAPE_SPHERE)
		fprintf(f, "shape,sphere,,\nradius,%g,,\nmass,%g,,\nrolling_resistance,%g,,\n", (double)def->radius, (double)def->mass, (double)def->rollingResistance);
	else if (def->shape == SHAPE_BOX)
		fprintf(f, "shape,box,,\nsize,%g,%g,%g\nmass,%g,,\n", (double)def->size.x, (double)def->size.y, (double)def->size.z, (double)def->mass);
	else
		fprintf(f, "shape,plane,,\nnormal,%g,%g,%g\noffset,%g,,\nextent,%g,,\n", (double)def->normal.x, (double)def->normal.y, (double)def->normal.z, (double)def->offset, (double)def->extent);
	fprintf(f, "friction,%g,,\nelasticity,%g,,\ncolor,%g,%g,%g\n", (double)def->friction, (double)def->elasticity, (double)def->color.x, (double)def->color.y, (double)def->color.z);
	if (fclose(f) != 0)
		return (fprintf(stderr, "%s: write failed\n", path), 0);
	return (1);
}
