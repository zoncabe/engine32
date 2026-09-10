#ifndef ENGINE_32_PLAYER_CONTROL_H
#define ENGINE_32_PLAYER_CONTROL_H

#include "e32_character3d_control.h"

typedef struct Viewport Viewport;
typedef struct Player   Player;

/* Reads the buttons this player was seated with and turns them into its
   command for this frame, aimed by the camera. */
void player_setCharacter3DControl(PlayerID id, Viewport *viewport);

#endif
