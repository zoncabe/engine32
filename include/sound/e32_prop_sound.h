#ifndef ENGINE_32_PROP_SOUND_H
#define ENGINE_32_PROP_SOUND_H

#include <stdint.h>

struct PhysicsWorld;

/* What wakes a sound of a prop. A sound is declared with one of these in its
   def; the update below fires the matching ones. A new event is a new value
   here. */
typedef enum {

	PROP_SOUND_COLLISION,     /* the body's first frame against anything solid */
	PROP_SOUND_WATER_ENTRY,   /* something fell into this water surface */

} PropSoundTrigger;


/* Walks the frame's new contacts: a dynamic body that just met something
   solid fires PROP_SOUND_COLLISION on its entity, scaled by the speed the
   solver killed; a water surface something just fell into fires
   PROP_SOUND_WATER_ENTRY on its own entity, from where the body entered,
   scaled by the plunge speed. Call after physics_update. */
void propSound_update(struct PhysicsWorld *world);

#endif
