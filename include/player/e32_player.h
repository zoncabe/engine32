#ifndef ENGINE_32_PLAYER_H
#define ENGINE_32_PLAYER_H

#include "entity/e32_entity3d.h"
#include "character3d/e32_character3d_movement.h"
#include "control/e32_character3d_control.h"


struct Player
{
	Character3D *character;
	Entity3D *entity;
	MovementCommand cmd;

	/* Which buttons the command above is built from. Written once, with the
	   character. */
	const Character3DControlBinding *control;
};


Player *player_get(void);
void player_init(void);
/* Seats a player: the body it drives and the buttons that drive it, together. */
void player_setCharacter3D(Character3D *character, const Character3DControlBinding *control);
void player_switchCharacter3D(PlayerID id, int8_t direction);
void player_update(void);
void player_setMatrix(uint8_t fb_index);

#endif
