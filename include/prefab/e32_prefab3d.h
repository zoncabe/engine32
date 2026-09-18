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

	/* Lights this model a vertex at a time instead of once for the whole
	   object. A point light is measured from the object's origin by default,
	   so a big piece takes one direction and one fade for all of it: a room
	   lit from a lamp inside it gets no falloff at all, and a shell with the
	   lamp at its centre gets nothing. Per vertex each point gets the
	   direction and the fade that actually reach it.

	   It costs a reload of the GTE's light matrix per vertex, so it is worth
	   it on what a point light is meant to shape and wasted on the rest.
	   Nothing to pay in a scene of directional lights: there is no point to
	   measure from and the flag is ignored. */
	bool vertex_lighting;

	/* The same, per named part: one flag per name in .part, in the same
	   order. A model whose lamp needs it and whose post does not says so
	   here, and the flag above is what part 0, the unnamed remainder, and a
	   model with no parts go by. */
	const bool *part_vertex_lighting;

	/* Objects inside the model the game drives on its own, named as they are
	   named in the model, so it can show and hide each one. Left out, the
	   model is drawn whole. Up to seven names. */
	const char *const *part;

	/* Where each of those objects is drawn, in the entity's own space and in
	   the same order as the names. A part left at zero stays where it was
	   modelled. */
	const Vector3 *part_position;

	uint8_t part_count;

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
