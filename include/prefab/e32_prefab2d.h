/*
	Content declaration for the 2D scene: what it draws with plus what its
	kind needs, behind a tag. Position is not here, it belongs to the
	placement, so one prefab can be placed any number of times. Mirror of
	Prefab3D.
*/
#ifndef ENGINE_32_PREFAB2D_H
#define ENGINE_32_PREFAB2D_H

#include "graphics/e32_graphic.h"
#include "sound/e32_sound.h"
#include "character2d/e32_character2d.h"


typedef enum {

	PREFAB2D_WIDGET,
	PREFAB2D_PROP,
	PREFAB2D_CHARACTER,

} Prefab2DType;


typedef struct Prefab2D {

	Prefab2DType type;

	/* The entity copies it and loads the sprite's file. A character's is a
	   sprite sheet it picks its frames out of. */
	Graphic graphic;

	/* Opened in the entity from create to delete. The looping ones play on
	   their own; the rest wait for whoever fires them. */
	const SoundDef *const *sound;
	uint8_t                sound_count;

	/* What the kind needs. A prop carries nothing yet: collision comes with
	   the 2D physics. */
	const Character2DDef *character;

} Prefab2D;


#endif
