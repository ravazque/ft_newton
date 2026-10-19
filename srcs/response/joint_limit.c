#include "newton.h"

// Locks and limits of a joint as angular rows [F33] [F34]: cone rows 0 swing and 1 twist, hinge rows 0 and 1 locks and 2 angle.

static float	twist_angle(Vec3 axis_a, Vec3 axis_b, Vec3 reference_a, Vec3 reference_b)
{
	Vec3	aligned = quat_rotate(quat_from_to(axis_b, axis_a), reference_b);

	return (atan2f(vec3_dot(vec3_cross(reference_a, aligned), axis_a), vec3_dot(reference_a, aligned)));   // [F33]
}

static void	set_row(JointRow *row, Vec3 axis, float lower_bound, float upper_bound)
{
	row->axis = axis;
	row->lowerBound = lower_bound;
	row->upperBound = upper_bound;
	row->error = 0.0f;
	row->bias = 0.0f;
	row->active = 1;
}

// A limit acts from JOINT_LIMIT_MARGIN before it: the joint may still close the gap left in one step, but not cross it.
static void	lower_limit(JointRow *row, Vec3 axis, float gap, float dt)
{
	set_row(row, axis, 0.0f, INFINITY);
	row->error = fmaxf(-gap, 0.0f);
	row->bias = -fmaxf(gap, 0.0f) / dt;   // [F34]
}

static void	upper_limit(JointRow *row, Vec3 axis, float gap, float dt)
{
	set_row(row, axis, -INFINITY, 0.0f);
	row->error = fminf(gap, 0.0f);
	row->bias = fmaxf(gap, 0.0f) / dt;   // [F34]
}

static void	range_row(JointRow *row, Vec3 axis, float angle, const Joint *joint, float dt)
{
	if (angle - joint->angleMin < JOINT_LIMIT_MARGIN)
		lower_limit(row, axis, angle - joint->angleMin, dt);
	else if (joint->angleMax - angle < JOINT_LIMIT_MARGIN)
		upper_limit(row, axis, joint->angleMax - angle, dt);
}

static void	measure_cone(const RigidBody *body_a, const RigidBody *body_b, const Joint *joint, JointRow *rows, float dt)
{
	Vec3	twist_axis_a = quat_rotate(body_a->orientation, joint->axisA);
	Vec3	twist_axis_b = quat_rotate(body_b->orientation, joint->axisB);
	float	swing = acosf(clampf(vec3_dot(twist_axis_a, twist_axis_b), -1.0f, 1.0f));   // [F33]
	float	twist = twist_angle(twist_axis_a, twist_axis_b, quat_rotate(body_a->orientation, joint->referenceA), quat_rotate(body_b->orientation, joint->referenceB));

	if (joint->swing - swing < JOINT_LIMIT_MARGIN)
		upper_limit(&rows[0], vec3_normalized(vec3_cross(twist_axis_a, twist_axis_b)), joint->swing - swing, dt);
	range_row(&rows[1], vec3_normalized(vec3_add(twist_axis_a, twist_axis_b)), twist, joint, dt);
}

static void	measure_hinge(const RigidBody *body_a, const RigidBody *body_b, const Joint *joint, JointRow *rows, float dt)
{
	Vec3	hinge_a = quat_rotate(body_a->orientation, joint->axisA);
	Vec3	hinge_b = quat_rotate(body_b->orientation, joint->axisB);
	Vec3	misalignment = vec3_cross(hinge_b, hinge_a);
	Vec3	across_u;
	Vec3	across_v;
	float	angle;

	across_u = vec3_normalized(vec3_cross(hinge_a, vec3(0.0f, 1.0f, 0.0f)));
	if (fabsf(hinge_a.y) > 0.9f)
		across_u = vec3_normalized(vec3_cross(hinge_a, vec3(1.0f, 0.0f, 0.0f)));
	across_v = vec3_cross(hinge_a, across_u);
	set_row(&rows[0], across_u, -INFINITY, INFINITY);   // [F34] lock
	rows[0].error = vec3_dot(misalignment, across_u);
	set_row(&rows[1], across_v, -INFINITY, INFINITY);
	rows[1].error = vec3_dot(misalignment, across_v);
	angle = twist_angle(hinge_a, hinge_b, quat_rotate(body_a->orientation, joint->referenceA), quat_rotate(body_b->orientation, joint->referenceB));
	range_row(&rows[2], hinge_a, angle, joint, dt);
}

static void	measure(const World *world, const Joint *joint, JointRow *rows)
{
	const RigidBody	*body_a = &world->bodies[joint->bodyA];
	const RigidBody	*body_b = &world->bodies[joint->bodyB];
	int				i;

	i = 0;
	while (i < JOINT_ROWS)
		rows[i++].active = 0;
	if (joint->type == JOINT_CONE)
		measure_cone(body_a, body_b, joint, rows, world->dt);
	else
		measure_hinge(body_a, body_b, joint, rows, world->dt);
}

static float	row_mass(const RigidBody *body_a, const RigidBody *body_b, Vec3 axis)
{
	float	inverse = vec3_dot(axis, mat3_mul_vec3(body_a->inverseInertiaWorld, axis)) + vec3_dot(axis, mat3_mul_vec3(body_b->inverseInertiaWorld, axis));   // [F34]

	if (inverse <= 0.0f)
		return (0.0f);
	return (1.0f / inverse);
}

static void	apply_spin(RigidBody *body_a, RigidBody *body_b, Vec3 axis, float impulse)
{
	rigidbody_apply_angular_impulse(body_a, vec3_scale(axis, -impulse));
	rigidbody_apply_angular_impulse(body_b, vec3_scale(axis, impulse));
}

void	joint_limit_prepare(World *world, Joint *joint)
{
	RigidBody	*body_a = &world->bodies[joint->bodyA];
	RigidBody	*body_b = &world->bodies[joint->bodyB];
	JointRow	rows[JOINT_ROWS];
	int			i;

	if (!body_a->awake && !body_b->awake)
		return ;
	measure(world, joint, rows);
	i = 0;
	while (i < JOINT_ROWS)
	{
		rows[i].impulse = 0.0f;
		if (rows[i].active && joint->rows[i].active)
			rows[i].impulse = clampf(joint->rows[i].impulse, rows[i].lowerBound, rows[i].upperBound);
		if (rows[i].active)
		{
			rows[i].mass = row_mass(body_a, body_b, rows[i].axis);
			apply_spin(body_a, body_b, rows[i].axis, rows[i].impulse);
		}
		joint->rows[i] = rows[i];
		i++;
	}
}

void	joint_limit_solve(World *world, Joint *joint)
{
	RigidBody	*body_a = &world->bodies[joint->bodyA];
	RigidBody	*body_b = &world->bodies[joint->bodyB];
	JointRow	*row;
	float		previous;
	float		relative_spin;
	int			i;

	if (!body_a->awake && !body_b->awake)
		return ;
	i = 0;
	while (i < JOINT_ROWS)
	{
		row = &joint->rows[i];
		i++;
		if (!row->active)
			continue ;
		previous = row->impulse;
		relative_spin = vec3_dot(vec3_sub(body_b->angularVelocity, body_a->angularVelocity), row->axis);
		row->impulse = clampf(previous - row->mass * (relative_spin - row->bias), row->lowerBound, row->upperBound);   // [F34]
		apply_spin(body_a, body_b, row->axis, row->impulse - previous);
	}
}

static void	turn_about_anchor(RigidBody *body_b, Vec3 anchor, Vec3 axis, float angle)
{
	Quat	rotation = quat_from_axis_angle(axis, angle);

	body_b->orientation = quat_normalized(quat_mul(rotation, body_b->orientation));
	body_b->position = vec3_add(anchor, quat_rotate(rotation, vec3_sub(body_b->position, anchor)));
	inertia_update_world(body_b);
}

void	joint_limit_project(World *world, Joint *joint)
{
	RigidBody	*body_a = &world->bodies[joint->bodyA];
	RigidBody	*body_b = &world->bodies[joint->bodyB];
	JointRow	rows[JOINT_ROWS];
	Vec3		anchor;
	float		excess;
	float		spin;
	int			i;

	if ((!body_a->awake && !body_b->awake) || body_b->inverseMass == 0.0f)
		return ;
	measure(world, joint, rows);
	anchor = vec3_add(body_a->position, quat_rotate(body_a->orientation, joint->anchorA));
	i = 0;
	while (i < JOINT_ROWS)
	{
		excess = 0.0f;
		if (rows[i].active && rows[i].error > JOINT_PROJECTION_ANGLE)
			excess = rows[i].error - JOINT_PROJECTION_ANGLE;
		else if (rows[i].active && rows[i].error < -JOINT_PROJECTION_ANGLE)
			excess = rows[i].error + JOINT_PROJECTION_ANGLE;
		if (excess != 0.0f)
		{
			turn_about_anchor(body_b, anchor, rows[i].axis, excess);   // [F38]
			spin = vec3_dot(vec3_sub(body_b->angularVelocity, body_a->angularVelocity), rows[i].axis);
			if (spin * excess < 0.0f)
				body_b->angularVelocity = vec3_sub(body_b->angularVelocity, vec3_scale(rows[i].axis, spin));   // [F38]
		}
		i++;
	}
}

void	joint_limit_correct(World *world, Joint *joint)
{
	RigidBody	*body_a = &world->bodies[joint->bodyA];
	RigidBody	*body_b = &world->bodies[joint->bodyB];
	JointRow	rows[JOINT_ROWS];
	float		turn;
	int			i;

	if (!body_a->awake && !body_b->awake)
		return ;
	measure(world, joint, rows);
	i = 0;
	while (i < JOINT_ROWS)
	{
		if (rows[i].active && fabsf(rows[i].error) > JOINT_SLOP)
		{
			turn = clampf(rows[i].error, -JOINT_MAX_CORRECTION, JOINT_MAX_CORRECTION) * JOINT_PERCENT * row_mass(body_a, body_b, rows[i].axis);   // [F35]
			correction_turn(world, joint->bodyA, vec3_scale(rows[i].axis, -turn));
			correction_turn(world, joint->bodyB, vec3_scale(rows[i].axis, turn));
		}
		i++;
	}
}
