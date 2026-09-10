/*
	The 2D entity: an object placed on a layer of the 2D scene, the Node2D of
	Godot. Mirror of the 3D entity: where it is and what it draws with. The
	graphic is the entity's, from create to delete; whoever animates it
	writes there.

	Body and culling come with the 2D physics and a world larger than the
	screen.
*/
#ifndef ENGINE_32_ENTITY2D_H
#define ENGINE_32_ENTITY2D_H

#include "physics/math/e32_vector2.h"
#include "graphics/e32_graphic.h"
#include "sound/e32_sound.h"


typedef struct Entity2D {

	/* Pixels, kept as floats so movement stays smooth; whoever writes it
	   snaps to whole pixels when it matters. */
	Vector2 position;
	Vector2 scale;
	float   rotation;

	Graphic *graphic;

	/* The prefab's sounds, open: one per entry, in the same order, from
	   create to delete. The looping ones play for as long as the entity
	   exists; the rest wait for whoever fires them. */
	Sound   *sound;
	uint8_t  sound_count;

} Entity2D;


/* Filled by the scene from the prefab and its placement, gone after the
   load. The graphic is copied in; a sprite gets its file loaded. A zero
   scale means identity. */
typedef struct Entity2DDef {

	const Graphic *graphic;
	const SoundDef *const *sound;
	uint8_t                sound_count;

	Vector2 position;
	Vector2 scale;
	float   rotation;

} Entity2DDef;


Entity2D *entity2d_create(const Entity2DDef *def);
void      entity2d_delete(Entity2D *entity);

#endif
