/*
	The room: floor, walls, the raised platform and the mound. Its collision is
	the triangle mesh the importer builds from the same OBJ, so what you see
	is what the character walks on.
*/
#include "prefab/e32_prefab3d.h"


static const PhysicsShapeDef room_shapes[] = {
	{ .type = SHAPE_MESH, .mesh = {
		.path        = "rom:/collision/room.collision",
		.friction    = 0.9_fp,
		.restitution = 0.1_fp,
	}},
};

static const Entity3DColliderDef room_collider = { room_shapes, 1 };

/* No body: a prop without one is static, which is what a room is. */
extern const Prefab3D room = {

	.type      = PREFAB3D_PROP,
	.model     = "rom:/models/room.model",
	.subdivide = true,
	.collider  = &room_collider,
};
