/*
	A one metre cube. Physics units are metres, the same the model is
	authored in, and the box is half of that on each side.
*/
#include "prefab/e32_prefab3d.h"


static const PhysicsShapeDef cube_shapes[] = {
	{ .type = SHAPE_BOX, .box = {
		.e           = { 0.5_fp, 0.5_fp, 0.5_fp },
		.friction    = 0.8_fp,
		.restitution = 0.1_fp,
	}},
};

static const Entity3DColliderDef cube_collider = { cube_shapes, 1 };

extern const Prefab3D cube = {

	.type     = PREFAB3D_PROP,
	.model    = "rom:/models/cube.model",
	.collider = &cube_collider,
};
