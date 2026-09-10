#ifndef ENGINE_32_GAME_STATES_H
#define ENGINE_32_GAME_STATES_H

#include <stdbool.h>
#include <stdint.h>


struct Game;
struct Scene3DDef;
struct Scene2DDef;
struct Player;

/* Index into the state table the game hands to game_start. */
typedef uint8_t GameState;

/* No state: the overlay_of sentinel. A field left out of a designated
   initializer is 0, which is a valid state, so every table entry must set
   overlay_of explicitly. */
#define GAME_STATE_NONE 0xFF


struct GameStateDef
{
	void (*update)();
	void (*bindCharacter)();
	void (*onEnter)();
	void (*onExit)();

	/* Holds the switch back while it answers false, so a state that plays
	   its way out is seen through. NULL leaves the moment it is asked. */
	bool (*canLeave)();

	/* Per-state input handling (menus, pause); NULL for none. The controller is
	   already polled: the game reads it with controller_get. */
	void (*control)();

	/* The scenes this state runs on, either or both: the 3D world and the
	   2D one drawn over it. */
	const Scene3DDef    *scene3d;
	const Scene2DDef    *scene2d;

	/* The state this one rides on top of; GAME_STATE_NONE for none.
	   Switching between an overlay and its base leaves the base untouched. */
	GameState          overlay_of;
};


/* Hands the engine the game's state table and loads the initial state.
   Runs after game_init and the game's own inits, before the first runStep. */
void game_start(const GameStateDef *states, uint8_t count, GameState initial);

const GameStateDef *gameState_get(GameState id);

void game_setState(Game *game, GameState new_state);
void game_updateState(void);


#endif
