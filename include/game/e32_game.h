/*
	The game as the engine sees it: a state table and the frame loop that
	runs the current state.

	PSYQo owns the loop: the application is started once and never returns,
	and calls the engine's scene once per frame. So game_run takes the
	place of game_init plus the for-ever loop of runStep: it brings the
	system up, calls the game's setup (its own inits and game_start), and
	hands the frame to PSYQo.
*/
#ifndef ENGINE_32_GAME_H
#define ENGINE_32_GAME_H

#include <stdbool.h>

#include "e32_game_states.h"
#include "control/e32_controller.h"
#include "system/e32_system.h"


struct Game
{
	GameState state;

	/* Where it is heading. A state that asks to leave sets this and plays
	   its way out; the switch happens once the screen is done. Equal to
	   state while there is nowhere to go. */
	GameState next;
};


Game *game_get(void);


/* Brings the engine up and runs the game. Never returns. 'setup' is called
   once the hardware is ready, before the first frame: the game's own inits
   and game_start go there. */
int game_run(void (*setup)());

/* One frame: time, input, the state's update and the render. PSYQo calls
   it; a game never does. */
void game_runStep(void);
void game_close(void);


#endif
