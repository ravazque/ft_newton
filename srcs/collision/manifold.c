#include "newton.h"

// Face clipping [F16] and the reduction of many contact candidates to four spread ones, so a resting body does not rock.

int	manifold_clip(Vec3 *polygon, int count, Vec3 normal, float offset)
{
	Vec3	kept_points[12];
	Vec3	current;
	Vec3	next;
	float	current_side;
	float	next_side;
	int		kept;
	int		i;

	kept = 0;
	i = 0;
	while (i < count)
	{
		current = polygon[i];
		next = polygon[(i + 1) % count];
		current_side = vec3_dot(normal, current) - offset;
		next_side = vec3_dot(normal, next) - offset;
		if (current_side <= 0.0f)
			kept_points[kept++] = current;
		if ((current_side < 0.0f && next_side > 0.0f) || (current_side > 0.0f && next_side < 0.0f))
			kept_points[kept++] = vec3_add(current, vec3_scale(vec3_sub(next, current), current_side / (current_side - next_side)));   // [F16]
		i++;
	}
	memcpy(polygon, kept_points, (size_t)kept * sizeof(Vec3));
	return (kept);
}

static int	is_chosen(const int *chosen, int chosen_count, int candidate)
{
	int	k;

	k = 0;
	while (k < chosen_count)
	{
		if (chosen[k] == candidate)
			return (1);
		k++;
	}
	return (0);
}

static int	pick_deepest(const Contact *candidates, int count, const int *chosen, int chosen_count)
{
	float	best_score = -1.0f;
	int		best = -1;
	int		i;

	i = 0;
	while (i < count)
	{
		if (!is_chosen(chosen, chosen_count, i) && candidates[i].penetration > best_score + MANIFOLD_DEPTH_TOLERANCE)
		{
			best_score = candidates[i].penetration;
			best = i;
		}
		i++;
	}
	return (best);
}

static int	pick_farthest(const Contact *candidates, int count, const int *chosen, int chosen_count, Vec3 from)
{
	float	best_score = -1.0f;
	float	score;
	int		best = -1;
	int		i;

	i = 0;
	while (i < count)
	{
		score = vec3_length_sq(vec3_sub(candidates[i].point, from));
		if (!is_chosen(chosen, chosen_count, i) && score > best_score)
		{
			best_score = score;
			best = i;
		}
		i++;
	}
	return (best);
}

// The candidate spanning the largest triangle with the first two picks, on the given side of them; -1 if none does.
static int	pick_widest(const Contact *candidates, int count, const int *chosen, int chosen_count, Vec3 axis, int side)
{
	Vec3	start = candidates[chosen[0]].point;
	Vec3	end = candidates[chosen[1]].point;
	float	best_score = -1.0f;
	float	score;
	int		best = -1;
	int		i;

	i = 0;
	while (i < count)
	{
		score = (float)side * vec3_dot(axis, vec3_cross(vec3_sub(end, start), vec3_sub(candidates[i].point, start)));
		if (!is_chosen(chosen, chosen_count, i) && score > best_score)
		{
			best_score = score;
			best = i;
		}
		i++;
	}
	if (best_score <= 0.000001f)
		return (-1);
	return (best);
}

int	manifold_reduce(const Contact *candidates, int count, Vec3 axis, Contact *out)
{
	int		chosen[4] = {-1, -1, -1, -1};
	int		chosen_count;
	int		side;
	int		pick;

	if (count <= MAX_CONTACTS_PER_PAIR)
	{
		memcpy(out, candidates, (size_t)count * sizeof(Contact));
		return (count);
	}
	chosen[0] = pick_deepest(candidates, count, chosen, 0);
	chosen[1] = pick_farthest(candidates, count, chosen, 1, candidates[chosen[0]].point);
	chosen_count = 2;
	side = 1;
	while (chosen_count < 4 && side >= -1)
	{
		pick = pick_widest(candidates, count, chosen, chosen_count, axis, side);
		if (pick >= 0)
			chosen[chosen_count++] = pick;
		side -= 2;
	}
	pick = 0;
	while (pick < chosen_count)
	{
		out[pick] = candidates[chosen[pick]];
		pick++;
	}
	return (chosen_count);
}

void	manifold_add_candidate(ContactCandidates *set, Vec3 point, Vec3 normal, float depth)
{
	int	slot;
	int	i;

	slot = set->count;
	if (set->count == MESH_CANDIDATES)
	{
		slot = 0;
		i = 1;
		while (i < set->count)
		{
			if (set->contacts[i].penetration < set->contacts[slot].penetration)
				slot = i;
			i++;
		}
		if (set->contacts[slot].penetration + MANIFOLD_DEPTH_TOLERANCE >= depth)
			return ;
	}
	else
		set->count++;
	set->contacts[slot].point = point;
	set->contacts[slot].normal = normal;
	set->contacts[slot].penetration = depth;
}

int	manifold_reduce_candidates(const ContactCandidates *set, Contact *out)
{
	int	deepest;
	int	i;

	if (set->count == 0)
		return (0);
	deepest = 0;
	i = 1;
	while (i < set->count)
	{
		if (set->contacts[i].penetration > set->contacts[deepest].penetration + MANIFOLD_DEPTH_TOLERANCE)
			deepest = i;
		i++;
	}
	return (manifold_reduce(set->contacts, set->count, set->contacts[deepest].normal, out));
}
