/*
	Two volumes over the same rungs, both rooted at the model's foot.

	The solid one is what the body walks into, grown well past the mesh: the
	rungs are 5 centimetres deep and nothing that thin can be pushed against,
	and the character's own capsule is 70 across, so it would sit on the
	box's edge and slide off.

	The climbable one is the reach around the rungs. Its top is where the
	climb ends, so it stops at the top rung; its depth has to reach past
	where the solid box stops the body, or the climb can never be caught.
	It starts below the foot, so standing at the bottom is already inside.
*/
#include "prefab/e32_prefab3d.h"


static const PhysicsShapeDef ladder_shapes[] = {
	{ .type = SHAPE_BOX, .box = {
		.tx = { .position = { 0.0_fp, 0.0_fp, 2.5_fp } },
		.e  = { 0.30_fp, 0.20_fp, 2.5_fp },
	}},
	{ .type = SHAPE_BOX, .box = {
		.tx     = { .position = { 0.0_fp, 0.0_fp, 2.3_fp } },
		.e      = { 0.30_fp, 0.65_fp, 2.7_fp },
		.sensor = SENSOR_CLIMBABLE,
	}},
};

static const Entity3DColliderDef ladder_collider = { ladder_shapes, 2 };

extern const Prefab3D ladder = {

	.type     = PREFAB3D_PROP,
	.model    = "rom:/models/ladder.model",
	.collider = &ladder_collider,
};
