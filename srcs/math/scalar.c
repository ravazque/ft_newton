#include "newton.h"

/* Scalar helpers shared by the collision, menu and control code. */

float	clampf(float v, float lo, float hi)
{
	if (v < lo)
		return (lo);
	if (v > hi)
		return (hi);
	return (v);
}
