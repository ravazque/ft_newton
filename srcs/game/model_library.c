#include "newton.h"

// Every .obj of MODEL_DIR, loaded once at start-up in name order; a file that cannot be used is reported and skipped.

static int	compare_names(const void *a, const void *b)
{
	return (strcmp(*(char *const *)a, *(char *const *)b));
}

static int	is_model_file(const char *file_name)
{
	size_t	length = strlen(file_name);
	size_t	extension = strlen(MODEL_EXTENSION);

	return (length > extension && strcmp(file_name + length - extension, MODEL_EXTENSION) == 0);
}

static void	free_names(char **names, int count)
{
	int	i;

	i = 0;
	while (i < count)
	{
		free(names[i]);
		i++;
	}
	free(names);
}

// The model file names of the directory, sorted; returns how many, -1 on failure.
static int	list_model_files(char ***names)
{
	DIR				*directory = opendir(MODEL_DIR);
	struct dirent	*entry;
	char			**grown;
	int				capacity;
	int				count;

	*names = NULL;
	if (!directory)
		return (fprintf(stderr, "%s: cannot open the model directory (%s)\n", MODEL_DIR, strerror(errno)), 0);
	capacity = 0;
	count = 0;
	entry = readdir(directory);
	while (entry)
	{
		if (is_model_file(entry->d_name))
		{
			grown = array_reserve(*names, &capacity, count + 1, sizeof(char *));
			if (!grown)
				return (closedir(directory), free_names(*names, count), -1);
			*names = grown;
			(*names)[count] = strdup(entry->d_name);
			if (!(*names)[count])
				return (closedir(directory), free_names(*names, count), -1);
			count++;
		}
		entry = readdir(directory);
	}
	closedir(directory);
	qsort(*names, (size_t)count, sizeof(char *), compare_names);
	return (count);
}

static void	load_model(ModelLibrary *library, const char *file_name, float size)
{
	char	path[CSV_PATH_LEN];
	char	name[MODEL_NAME_LEN];
	size_t	name_length = strlen(file_name) - strlen(MODEL_EXTENSION);
	int		index = library->count;

	if (name_length >= MODEL_NAME_LEN || strlen(MODEL_DIR) + strlen(file_name) >= CSV_PATH_LEN)
	{
		fprintf(stderr, "%s%s: file name too long, model skipped\n", MODEL_DIR, file_name);
		return ;
	}
	snprintf(path, sizeof(path), "%s%s", MODEL_DIR, file_name);
	if (!mesh_shape_load(&library->shapes[index], path, size, index))
	{
		fprintf(stderr, "%s: model skipped\n", path);
		return ;
	}
	memcpy(name, file_name, name_length);
	name[name_length] = '\0';
	memcpy(library->names[index], name, sizeof(name));
	snprintf(library->labels[index], MENU_LABEL_LEN, "%s  %d triangles", name, library->shapes[index].triangleCount);
	library->count++;
}

int	model_library_load(ModelLibrary *library, float size)
{
	char	**names;
	int		count;
	int		i;

	memset(library, 0, sizeof(*library));
	count = list_model_files(&names);
	if (count < 0)
		return (0);
	i = 0;
	while (i < count && library->count < MODEL_MAX)
	{
		load_model(library, names[i], size);
		i++;
	}
	if (i < count)
		fprintf(stderr, "%s: only the first %d models are loaded, %d files ignored\n", MODEL_DIR, MODEL_MAX, count - i);
	if (library->count == 0)
		printf("%s: no model loaded, keys 6 to 8 have nothing to place\n", MODEL_DIR);
	free_names(names, count);
	return (1);
}

void	model_library_free(ModelLibrary *library)
{
	int	i;

	i = 0;
	while (i < library->count)
	{
		mesh_shape_free(&library->shapes[i]);
		i++;
	}
	library->count = 0;
	library->selected = 0;
}

void	model_library_resize(ModelLibrary *library, float size)
{
	int	i;

	i = 0;
	while (i < library->count)
	{
		mesh_shape_rescale(&library->shapes[i], size);
		i++;
	}
}

void	model_library_step(ModelLibrary *library, int direction)
{
	if (library->count == 0)
		return ;
	library->selected = (library->selected + direction + library->count) % library->count;
}
