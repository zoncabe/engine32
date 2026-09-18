#ifndef ENGINE_32_GAME_STATES_H
#define ENGINE_32_GAME_STATES_H

#include <stdbool.h>
#include <stdint.h>


struct Game;
struct Scene3DDef;
struct Scene2DDef;
struct Player;
struct ControlsDef;

/* Index into the state table the game hands to game_start. */
typedef uint8_t GameState;

/* No state: what a state index reads as when there is none. */
#define GAME_STATE_NONE 0xFF


struct GameStateDef
{
	void (*update)();
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

	/* What the game drives this state with. Wired once, after the scene is
	   loaded and before the first update: the player is seated on the body
	   its binding names, and the camera answers to the buttons that name it. */
	const ControlsDef   *controls;

	/* The table entry this one rides on top of (&states[BASE]); NULL, the
	   default, for none. Switching between an overlay and its base leaves
	   the base untouched. */
	const GameStateDef *overlay_of;
};


/* Hands the engine the game's state table and loads the initial state.
   Runs after game_init and the game's own inits, before the first runStep. */
void game_start(const GameStateDef *states, uint8_t count, GameState initial);

const GameStateDef *gameState_get(GameState id);

void game_setState(Game *game, GameState new_state);
void game_updateState(void);


#endif
