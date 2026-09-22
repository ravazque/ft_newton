#include "newton.h"

/*
 * Contact manifolds of the box contacts: Sutherland-Hodgman clipping of a face
 * polygon [F16], and the reduction of many candidate points to at most
 * MAX_CONTACTS_PER_PAIR well spread ones. Keeping only the deepest points would
 * cluster them on one side and make a resting box rock.
*/

/* Keeps the part of the polygon behind the plane dot(n, x) = off, in place. */
int	manifold_clip(Vec3 *poly, int count, Vec3 n, float off)
{
	Vec3	out[12];
	Vec3	cur;
	Vec3	nxt;
	float	dc;
	float	dn;
	int		kept;
	int		i;

	kept = 0;
	i = 0;
	while (i < count)
	{
		cur = poly[i];
		nxt = poly[(i + 1) % count];
		dc = vec3_dot(n, cur) - off;
		dn = vec3_dot(n, nxt) - off;
		if (dc <= 0.0f)
			out[kept++] = cur;
		if ((dc < 0.0f && dn > 0.0f) || (dc > 0.0f && dn < 0.0f))
			out[kept++] = vec3_add(cur, vec3_scale(vec3_sub(nxt, cur), dc / (dc - dn)));   /* [F16] */
		i++;
	}
	memcpy(poly, out, (size_t)kept * sizeof(Vec3));
	return (kept);
}

/* Best untaken candidate: mode 0 deepest, 1 farthest from p0, 2 largest signed area with p0 p1 on 'side'. */
static int	pick_point(const Contact *cand, int n, const int *taken, Vec3 axis, Vec3 p0, Vec3 p1, int mode, int side)
{
	float	best_score = -1.0f;
	float	score;
	int		best = -1;
	int		i;

	i = 0;
	while (i < n)
	{
		if (!taken[i])
		{
			if (mode == 0)
				score = cand[i].penetration;
			else if (mode == 1)
				score = vec3_length_sq(vec3_sub(cand[i].point, p0));
			else
				score = (float)side * vec3_dot(axis, vec3_cross(vec3_sub(p1, p0), vec3_sub(cand[i].point, p0)));
			if (score > best_score)
			{
				best_score = score;
				best = i;
			}
		}
		i++;
	}
	if (mode == 2 && best_score <= 0.000001f)
		return (-1);
	return (best);
}

/* Deepest point, the one farthest from it, then the largest triangle on each side of those two. */
int	manifold_reduce(const Contact *cand, int n, Vec3 axis, Contact *out)
{
	int		taken[12] = {0};
	int		chosen[4];
	int		count;
	int		side;
	int		k;

	if (n <= MAX_CONTACTS_PER_PAIR)
	{
		memcpy(out, cand, (size_t)n * sizeof(Contact));
		return (n);
	}
	chosen[0] = pick_point(cand, n, taken, axis, cand[0].point, cand[0].point, 0, 1);
	taken[chosen[0]] = 1;
	chosen[1] = pick_point(cand, n, taken, axis, cand[chosen[0]].point, cand[chosen[0]].point, 1, 1);
	taken[chosen[1]] = 1;
	count = 2;
	side = 1;
	while (count < 4 && side >= -1)
	{
		k = pick_point(cand, n, taken, axis, cand[chosen[0]].point, cand[chosen[1]].point, 2, side);
		if (k >= 0)
		{
			taken[k] = 1;
			chosen[count++] = k;
		}
		side -= 2;
	}
	k = 0;
	while (k < count)
	{
		out[k] = cand[chosen[k]];
		k++;
	}
	return (count);
}
