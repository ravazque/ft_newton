#include "newton.h"

// A whole regular file in a NUL-terminated buffer the caller frees: the shaders and the model files.

static char	*fail(FILE *file, char *buffer, const char *path, const char *reason)
{
	fprintf(stderr, "%s: %s\n", path, reason);
	free(buffer);
	if (file)
		fclose(file);
	return (NULL);
}

char	*file_read(const char *path)
{
	FILE		*file;
	struct stat	status;
	char		*buffer;

	file = fopen(path, "rb");
	if (!file)
		return (fail(NULL, NULL, path, strerror(errno)));
	if (fstat(fileno(file), &status) != 0 || !S_ISREG(status.st_mode))
		return (fail(file, NULL, path, "not a regular file"));
	buffer = malloc((size_t)status.st_size + 1);
	if (!buffer)
		return (fail(file, NULL, path, "out of memory"));
	if (fread(buffer, 1, (size_t)status.st_size, file) != (size_t)status.st_size)
		return (fail(file, buffer, path, "cannot read the file"));
	buffer[status.st_size] = '\0';
	fclose(file);
	return (buffer);
}
