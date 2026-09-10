/*
	The 2D character: a sprite animation driven by discrete frames. Poses
	change at the action's fps; the position moves at the game's rate and is
	snapped to whole pixels only when set on the scene entity, so the
	sprite never lands on a half pixel.
*/
#include <assert.h>
#include <malloc.h>
#include <math.h>
#include <libdragon.h>

#include "character2d/e32_character2d.h"


static const Character2DAction *character2d_currentAction(const Character2D *character)
{
	return &character->def->action[character->action];
}

/* Hands the scene entity the current frame: cell of the sheet, flip, and
   the feet position carried to the top-left corner the blit draws from. */
static void character2d_setEntity(Character2D *character)
{
	const Character2DAction *action = character2d_currentAction(character);

	Entity2D *entity = character->entity;
	Sprite   *sprite = &entity->graphic->sprite;

	sprite->frame  = action->first_frame + character->current_frame;
	sprite->flip_x = character->facing_left;

	/* Feet at the bottom center of the cell. */
	int cols = sprite->cols ? sprite->cols : 1;
	int rows = sprite->rows ? sprite->rows : 1;
	int w    = sprite->asset->width  / cols;
	int h    = sprite->asset->height / rows;

	entity->position.x = floorf(character->position.x) - w / 2;
	entity->position.y = floorf(character->position.y) - h;
}


Character2D *character2d_create(const Character2DDef *def, Entity2D *entity)
{
	assert(def && def->action_count && entity);

	Character2D *character = malloc(sizeof(Character2D));
	assert(character);

	/* The placement put the feet where the entity stands. */
	*character = (Character2D){
		.def      = def,
		.entity   = entity,
		.position = entity->position,
	};

	character2d_setAction(character, 0);
	return character;
}

void character2d_delete(Character2D *character)
{
	if (!character) return;
	free(character);
}

void character2d_setAction(Character2D *character, uint8_t action)
{
	assert(action < character->def->action_count);

	character->action        = action;
	character->current_frame = 0;
	character->time          = 0.0f;
	character->finished      = false;

	character2d_setEntity(character);
}

void character2d_update(Character2D *character, float dt)
{
	const Character2DAction *action = character2d_currentAction(character);
	float frame_time = 1.0f / action->fps;

	character->time += dt;

	while (character->time >= frame_time) {
		character->time -= frame_time;

		if (character->current_frame + 1 < action->frame_count) {
			character->current_frame++;
		} else if (action->loop) {
			character->current_frame = 0;
		} else {
			character->finished = true;
			character->time = 0.0f;
			break;
		}
	}

	character2d_setEntity(character);
}
