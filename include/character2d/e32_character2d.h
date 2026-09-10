#ifndef ENGINE_32_CHARACTER2D_H
#define ENGINE_32_CHARACTER2D_H

#include <stdbool.h>
#include <stdint.h>

#include "entity/e32_entity2d.h"


/* One clip: a run of consecutive frames in the entity's sheet, counted the
   way the sprite counts them, row by row from 0. The sheet is the
   entity's; the character only picks frames out of it. */
typedef struct Character2DAction {

	uint8_t first_frame;
	uint8_t frame_count;
	uint8_t fps;
	bool    loop;

} Character2DAction;

typedef struct Character2DDef {

	const Character2DAction *action;
	uint8_t                  action_count;

} Character2DDef;

typedef struct Character2D {

	const Character2DDef *def;

	/* The scene entity this character draws through. The character writes
	   its frame, flip and position from create to delete. */
	Entity2D *entity;

	Vector2 position;      /* feet, in pixels; float so movement stays smooth */
	bool    facing_left;

	uint8_t action;
	uint8_t current_frame; /* within the action, from 0 */
	float   time;          /* elapsed inside the current frame */
	bool    finished;      /* a non-looping action reached its last frame */

} Character2D;


Character2D *character2d_create(const Character2DDef *def, Entity2D *entity);
void character2d_delete(Character2D *character);

/* Restarts the action from its first frame, even if it is already playing. */
void character2d_setAction(Character2D *character, uint8_t action);

/* Advances the animation and writes the frame to the entity, anchored at
   the feet and snapped to whole pixels. */
void character2d_update(Character2D *character, float dt);

#endif
