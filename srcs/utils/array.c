#include "newton.h"

// Growable arrays: the capacity doubles, so n appends cost O(n) in total; on failure the array is left untouched.

void	*array_reserve(void *data, int *capacity, int need, size_t element_size)
{
	int		new_capacity;
	void	*grown;

	if (need <= *capacity)
		return (data);
	new_capacity = *capacity;
	if (new_capacity < 16)
		new_capacity = 16;
	while (new_capacity < need && new_capacity <= INT_MAX / 2)
		new_capacity *= 2;
	if (new_capacity < need || (size_t)new_capacity > SIZE_MAX / element_size)
		return (fprintf(stderr, "Out of memory\n"), NULL);
	grown = realloc(data, (size_t)new_capacity * element_size);
	if (!grown)
		return (fprintf(stderr, "Out of memory\n"), NULL);
	*capacity = new_capacity;
	return (grown);
}
