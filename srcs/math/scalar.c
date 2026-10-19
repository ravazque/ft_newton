#include "newton.h"

float	clampf(float value, float low, float high)
{
	if (value < low)
		return (low);
	if (value > high)
		return (high);
	return (value);
}
