#include "newton.h"

/*
 * An ObjectDef is the bridge between one assets/objects CSV file and a
 * RigidBody: the file says what a kind of object is (shape, size, mass,
 * friction, restitution, color); objectdef_make_body stamps it at a position.
 * Loading is strict: every property of the shape must be present, nothing
 * else may appear, and every value must be inside its physical range.
*/

static const char	*g_sphere_keys[] = {"shape", "radius", "mass", "friction", "restitution", "color"};
static const char	*g_box_keys[] = {"shape", "size", "mass", "friction", "restitution", "color"};
static const char	*g_plane_keys[] = {"shape", "normal", "offset", "extent", "friction", "restitution", "color"};

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

/* The properties that only one shape has. Planes are infinite and static,
 * so they carry no mass; 'extent' is just the size of the square drawn. */
static void	read_shape_properties(ObjectDef *def, CsvFile *csv)
{
	if (def->shape == SHAPE_SPHERE)
	{
		csv_reject_unknown(csv, g_sphere_keys, 6);
		def->radius = csv_get_float(csv, "radius");
		def->mass = csv_get_float(csv, "mass");
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

/* Every check runs, so one bad file reports all its problems at once. */
static void	validate(const ObjectDef *def, CsvFile *csv)
{
	if (def->shape == SHAPE_SPHERE)
		csv_expect_positive(csv, "radius", def->radius);
	if (def->shape == SHAPE_BOX)
		csv_expect_positive_vec3(csv, "size", def->size);
	if (def->shape == SHAPE_PLANE)
	{
		csv_expect_positive(csv, "extent", def->extent);
		if (vec3_length_sq(def->normal) == 0.0f)
			csv_error(csv, "normal", "a non-zero vector");
	}
	if (def->shape != SHAPE_PLANE)
		csv_expect_positive(csv, "mass", def->mass);
	csv_expect_min(csv, "friction", def->friction, 0.0f);
	csv_expect_range(csv, "restitution", def->restitution, 0.0f, 1.0f);
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
	def->restitution = csv_get_float(&csv, "restitution");
	def->color = csv_get_vec3(&csv, "color");
	if (csv.errors)
		return (0);
	validate(def, &csv);
	if (csv.errors)
		return (0);
	def->normal = vec3_normalized(def->normal);
	return (1);
}

/* Planes are always static: an infinite surface has no meaningful mass. */
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
		b.collider = collider_plane(def->normal, def->offset);
	b.friction = def->friction;
	b.restitution = def->restitution;
	b.color = def->color;
	if (def->shape == SHAPE_PLANE)
		rb_make_static(&b);
	else
		rb_set_mass(&b, def->mass);
	return (b);
}

/* Writes the definition back in the file's own format, so a value tuned in
 * the menu survives a restart. The order matches the shipped files: what the
 * shape needs first, then the material every shape shares. */
int	objectdef_save(const ObjectDef *def, const char *path)
{
	FILE	*f = fopen(path, "w");

	if (!f)
		return (fprintf(stderr, "%s: cannot write file\n", path), 0);
	fprintf(f, "%s\n", CSV_HEADER);
	if (def->shape == SHAPE_SPHERE)
		fprintf(f, "shape,sphere,,\nradius,%g,,\nmass,%g,,\n", (double)def->radius, (double)def->mass);
	else if (def->shape == SHAPE_BOX)
		fprintf(f, "shape,box,,\nsize,%g,%g,%g\nmass,%g,,\n", (double)def->size.x, (double)def->size.y, (double)def->size.z, (double)def->mass);
	else
		fprintf(f, "shape,plane,,\nnormal,%g,%g,%g\noffset,%g,,\nextent,%g,,\n", (double)def->normal.x, (double)def->normal.y, (double)def->normal.z, (double)def->offset, (double)def->extent);
	fprintf(f, "friction,%g,,\nrestitution,%g,,\ncolor,%g,%g,%g\n", (double)def->friction, (double)def->restitution, (double)def->color.x, (double)def->color.y, (double)def->color.z);
	if (fclose(f) != 0)
		return (fprintf(stderr, "%s: write failed\n", path), 0);
	return (1);
}
