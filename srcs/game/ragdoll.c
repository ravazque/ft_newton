#include "newton.h"

// The ragdoll: eleven parts held by ten joints, cones at neck, waist, shoulders and hips, hinges at elbows and knees.

typedef enum RagdollPart
{
	PART_PELVIS,
	PART_TORSO,
	PART_HEAD,
	PART_UPPER_ARM_LEFT,
	PART_FOREARM_LEFT,
	PART_UPPER_ARM_RIGHT,
	PART_FOREARM_RIGHT,
	PART_THIGH_LEFT,
	PART_SHIN_LEFT,
	PART_THIGH_RIGHT,
	PART_SHIN_RIGHT
}	RagdollPart;

typedef struct PartLayout
{
	Vec3	center;
	Vec3	size;
	float	massShare;
}	PartLayout;

typedef struct JointLayout
{
	RagdollPart	parent;
	RagdollPart	child;
	JointType	type;
	Vec3		anchor;
	Vec3		axisParent;
	Vec3		axisChild;
	float		swingDegrees;
	float		angleMinDegrees;
	float		angleMaxDegrees;
	int			collides;
}	JointLayout;

// A figure RAGDOLL_LAYOUT_HEIGHT tall standing at the origin, facing +Z (+X is its left); the head's size is its diameter.
static const PartLayout	g_parts[RAGDOLL_PARTS] = {
	[PART_PELVIS] = {{0.0f, 0.99f, 0.0f}, {0.34f, 0.18f, 0.20f}, 0.14f},
	[PART_TORSO] = {{0.0f, 1.32f, 0.0f}, {0.38f, 0.44f, 0.22f}, 0.32f},
	[PART_HEAD] = {{0.0f, 1.68f, 0.0f}, {0.22f, 0.22f, 0.22f}, 0.08f},
	[PART_UPPER_ARM_LEFT] = {{0.255f, 1.33f, 0.0f}, {0.09f, 0.30f, 0.09f}, 0.03f},
	[PART_FOREARM_LEFT] = {{0.255f, 1.01f, 0.0f}, {0.08f, 0.30f, 0.08f}, 0.025f},
	[PART_UPPER_ARM_RIGHT] = {{-0.255f, 1.33f, 0.0f}, {0.09f, 0.30f, 0.09f}, 0.03f},
	[PART_FOREARM_RIGHT] = {{-0.255f, 1.01f, 0.0f}, {0.08f, 0.30f, 0.08f}, 0.025f},
	[PART_THIGH_LEFT] = {{0.09f, 0.67f, 0.0f}, {0.14f, 0.42f, 0.15f}, 0.11f},
	[PART_SHIN_LEFT] = {{0.09f, 0.22f, 0.0f}, {0.11f, 0.44f, 0.12f}, 0.065f},
	[PART_THIGH_RIGHT] = {{-0.09f, 0.67f, 0.0f}, {0.14f, 0.42f, 0.15f}, 0.11f},
	[PART_SHIN_RIGHT] = {{-0.09f, 0.22f, 0.0f}, {0.11f, 0.44f, 0.12f}, 0.065f}
};

float	ragdoll_mass_share(int part)
{
	if (part < 0 || part >= RAGDOLL_PARTS)
		return (0.0f);
	return (g_parts[part].massShare);
}

static Vec3	lean(Vec3 axis, Vec3 toward, float degrees)
{
	return (vec3_add(vec3_scale(axis, cosf(DEG2RAD(degrees))), vec3_scale(toward, sinf(DEG2RAD(degrees)))));
}

static JointLayout	mirrored(JointLayout layout, RagdollPart parent, RagdollPart child)
{
	layout.parent = parent;
	layout.child = child;
	layout.anchor.x = -layout.anchor.x;
	if (layout.type == JOINT_CONE)
	{
		layout.axisParent.x = -layout.axisParent.x;
		layout.axisChild.x = -layout.axisChild.x;
	}
	return (layout);
}

static JointLayout	cone(RagdollPart parent, RagdollPart child, Vec3 anchor, Vec3 axis_parent, Vec3 axis_child, float swing, float twist)
{
	return ((JointLayout){parent, child, JOINT_CONE, anchor, axis_parent, axis_child, swing, -twist, twist, 0});
}

static JointLayout	hinge(RagdollPart parent, RagdollPart child, Vec3 anchor, Vec3 axis, float fold)
{
	return ((JointLayout){parent, child, JOINT_HINGE, anchor, axis, axis, 0.0f, 0.0f, fold, 0});
}

static void	joint_layouts(JointLayout *layouts)
{
	Vec3	up = vec3(0.0f, 1.0f, 0.0f);
	Vec3	down = vec3(0.0f, -1.0f, 0.0f);
	Vec3	forward = vec3(0.0f, 0.0f, 1.0f);
	Vec3	left = vec3(1.0f, 0.0f, 0.0f);
	Vec3	shoulder = lean(down, left, RAGDOLL_SHOULDER_TILT);

	layouts[0] = cone(PART_TORSO, PART_HEAD, vec3(0.0f, 1.555f, 0.0f), up, up, RAGDOLL_NECK_SWING, RAGDOLL_NECK_TWIST);
	layouts[1] = cone(PART_PELVIS, PART_TORSO, vec3(0.0f, 1.09f, 0.0f), lean(up, forward, RAGDOLL_WAIST_TILT), up, RAGDOLL_WAIST_SWING, RAGDOLL_WAIST_TWIST);
	layouts[2] = cone(PART_TORSO, PART_UPPER_ARM_LEFT, vec3(0.255f, 1.48f, 0.0f), shoulder, down, RAGDOLL_SHOULDER_SWING, RAGDOLL_SHOULDER_TWIST);
	layouts[2].collides = 1;
	layouts[3] = mirrored(layouts[2], PART_TORSO, PART_UPPER_ARM_RIGHT);
	layouts[4] = cone(PART_PELVIS, PART_THIGH_LEFT, vec3(0.09f, 0.89f, 0.0f), lean(down, forward, RAGDOLL_HIP_TILT), down, RAGDOLL_HIP_SWING, RAGDOLL_HIP_TWIST);
	layouts[5] = mirrored(layouts[4], PART_PELVIS, PART_THIGH_RIGHT);
	layouts[6] = hinge(PART_UPPER_ARM_LEFT, PART_FOREARM_LEFT, vec3(0.255f, 1.17f, 0.0f), vec3_neg(left), RAGDOLL_ELBOW_MAX);
	layouts[7] = mirrored(layouts[6], PART_UPPER_ARM_RIGHT, PART_FOREARM_RIGHT);
	layouts[8] = hinge(PART_THIGH_LEFT, PART_SHIN_LEFT, vec3(0.09f, 0.45f, 0.0f), left, RAGDOLL_KNEE_MAX);
	layouts[9] = mirrored(layouts[8], PART_THIGH_RIGHT, PART_SHIN_RIGHT);
}

static void	add_joints(World *world, int first_body, Vec3 origin, float scale)
{
	JointLayout	layouts[RAGDOLL_JOINTS];
	Joint		joint;
	Vec3		anchor;
	int			parent;
	int			child;
	int			i;

	joint_layouts(layouts);
	i = 0;
	while (i < RAGDOLL_JOINTS)
	{
		anchor = vec3_add(origin, vec3_scale(layouts[i].anchor, scale));
		parent = first_body + (int)layouts[i].parent;
		child = first_body + (int)layouts[i].child;
		joint = joint_make(world, parent, child, anchor, layouts[i].axisParent, layouts[i].axisChild, vec3(0.0f, 0.0f, 1.0f));
		joint.type = layouts[i].type;
		joint.swing = DEG2RAD(layouts[i].swingDegrees);
		joint.angleMin = DEG2RAD(layouts[i].angleMinDegrees);
		joint.angleMax = DEG2RAD(layouts[i].angleMaxDegrees);
		joint.collides = layouts[i].collides;
		joint_add(world, joint);
		i++;
	}
}

static RigidBody	make_part(const RagdollDef *def, int part, Vec3 origin, float scale, int ragdoll_id)
{
	const PartLayout	*layout = &g_parts[part];
	RigidBody			body = rigidbody_make();

	body.kind = KIND_RAGDOLL;
	body.ragdollId = ragdoll_id;
	body.ragdollPart = part;
	body.position = vec3_add(origin, vec3_scale(layout->center, scale));
	body.collider = collider_box(vec3_scale(layout->size, 0.5f * scale));
	if (part == PART_HEAD)
	{
		body.collider = collider_sphere(layout->size.x * 0.5f * scale);
		body.rollingResistance = RAGDOLL_HEAD_ROLLING;
	}
	body.friction = def->friction;
	body.elasticity = def->elasticity;
	body.color = def->color;
	rigidbody_set_mass(&body, def->mass * layout->massShare);
	return (body);
}

int	ragdoll_spawn(World *world, const RagdollDef *def, Vec3 origin)
{
	RigidBody	parts[RAGDOLL_PARTS];
	float		scale = def->height / RAGDOLL_LAYOUT_HEIGHT;
	float		lift;
	int			first_body;
	int			part;

	world->ragdollCount++;
	part = 0;
	while (part < RAGDOLL_PARTS)
	{
		parts[part] = make_part(def, part, origin, scale, world->ragdollCount);
		part++;
	}
	first_body = world->bodyCount;
	lift = placement_add(world, parts, RAGDOLL_PARTS);
	if (lift < 0.0f)
		return (-1);
	add_joints(world, first_body, vec3_add(origin, vec3(0.0f, lift, 0.0f)), scale);
	return (first_body);
}
