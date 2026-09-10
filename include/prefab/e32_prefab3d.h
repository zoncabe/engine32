/*
	Content declaration: a model plus what its kind needs, behind a tag.
	Position, rotation and scale are not here, they belong to the placement,
	so one prefab can be placed any number of times.
*/
#ifndef ENGINE_32_PREFAB3D_H
#define ENGINE_32_PREFAB3D_H

#include <stdint.h>

#include "entity/e32_entity3d.h"

/* What a prefab of each kind is declared with: the game writes these
   defs next to the prefab, so their full types come in here. */
#include "physics/body/e32_rigid_body.h"
#include "physics/shapes/e32_physics_shape.h"
#include "physics/cloth/e32_cloth.h"
#include "character3d/e32_character3d.h"
#include "shaders/e32_water.h"

/* Not ported yet: only pointed at from here. */
struct SoundDef;


typedef enum {

	PREFAB3D_CHARACTER,
	PREFAB3D_PROP,
	PREFAB3D_CLOTH,
	PREFAB3D_WATER,

} Prefab3DType;


struct Prefab3D
{
	Prefab3DType type;
	const char *model;

	/* Big faces that span depth (floors, walls) get cut into pieces when
	   drawn, against the GPU's linear texture mapping. Off for small
	   things, where the pieces would only cost. */
	bool subdivide;

	/* Opened in the entity from create to delete. The looping ones play on
	   their own; the rest wait for whoever fires them. */
	const SoundDef *const *sound;
	uint8_t                sound_count;

	/* Solid for a prop, sensor volume for water. NULL: no collision. */
	const Entity3DColliderDef *collider;

	/* The kind the tag names. A prop without a body is static. */
	union {
		const Character3DDef *character;
		const RigidBodyDef *prop;
		const ClothDef     *cloth;
		const WaterDef     *water;
	};
};


#endif
