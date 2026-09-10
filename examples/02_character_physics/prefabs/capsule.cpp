/*
	The same capsule the character wears, standing still. Its shape is written
	here by hand because a prop knows nothing about character settings: radius
	0.35 and 1.80 tall, so the segment between the two caps is 0.55 either side
	of the middle.
*/
#include "prefab/e32_prefab3d.h"


static const PhysicsShapeDef capsule_shapes[] = {
	{ .type = SHAPE_CAPSULE, .capsule = {
		.tx          = { .position = { 0.0_fp, 0.0_fp, 0.90_fp } },
		.radius      = 0.35_fp,
		.half_height = 0.55_fp,
		.friction    = 0.8_fp,
		.restitution = 0.1_fp,
	}},
};

static const Entity3DColliderDef capsule_collider = { capsule_shapes, 1 };

extern const Prefab3D capsule = {

	.type     = PREFAB3D_PROP,
	.model    = "rom:/models/capsule.model",
	.collider = &capsule_collider,
};
