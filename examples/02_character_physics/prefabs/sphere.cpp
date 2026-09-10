/*
	A one metre ball. Rounded, so the character slides off it instead of
	standing on top: the counterpart of the cube's flat faces.
*/
#include "prefab/e32_prefab3d.h"


static const PhysicsShapeDef sphere_shapes[] = {
	{ .type = SHAPE_SPHERE, .sphere = {
		.radius      = 0.5_fp,
		.friction    = 0.8_fp,
		.restitution = 0.1_fp,
	}},
};

static const Entity3DColliderDef sphere_collider = { sphere_shapes, 1 };

extern const Prefab3D sphere = {

	.type     = PREFAB3D_PROP,
	.model    = "rom:/models/sphere.model",
	.collider = &sphere_collider,
};
