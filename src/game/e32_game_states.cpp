/*
	State machinery only: the table itself is the game's, handed over at
	game_start. The engine loads, unloads and switches whatever it is given.
*/
#include "psyqo/kernel.hh"

#include "time/e32_time.h"
#include "scene3d/e32_scene3d.h"
#ifdef ENGINE_32_SCENE2D
#include "scene2d/e32_scene2d.h"
#endif
#ifdef ENGINE_32_PLAYER
#include "player/e32_player.h"
#endif
#include "game/e32_game.h"
#include "game/e32_game_states.h"


static const GameStateDef *game_state;
static uint8_t             game_state_count;


const GameStateDef *gameState_get(GameState id)
{
	psyqo::Kernel::assert(id < game_state_count, "game: no such state");
	return &game_state[id];
}

static void gameState_load(GameState id)
{
	if (game_state[id].scene3d) {
		scene3d_load(game_state[id].scene3d);
		if (game_state[id].bindCharacter) game_state[id].bindCharacter();
	}
#ifdef ENGINE_32_SCENE2D
	if (game_state[id].scene2d) scene2d_load(game_state[id].scene2d);
#endif
	if (game_state[id].onEnter) game_state[id].onEnter();
}

static void gameState_unload(GameState id)
{
	if (game_state[id].onExit) game_state[id].onExit();
#ifdef ENGINE_32_SCENE2D
	if (game_state[id].scene2d) scene2d_unload();
#endif
	if (game_state[id].scene3d) {
#ifdef ENGINE_32_PLAYER
		player_init();
#endif
		scene3d_unload();
	}
}

static bool gameState_isOverlayPair(GameState prev, GameState next)
{
	return game_state[next].overlay_of == prev || game_state[prev].overlay_of == next;
}

/* Asking to leave is not leaving: the state names where it goes, and the
   switch happens as soon as it lets go. */
void game_setState(Game *game, GameState new_state)
{
	psyqo::Kernel::assert(new_state < game_state_count, "game: no such state");
	game->next = new_state;
}

static void gameState_settle(Game *game)
{
	if (game->next == game->state) return;

	const GameStateDef *leaving = &game_state[game->state];
	if (leaving->canLeave && !leaving->canLeave()) return;

	GameState prev      = game->state;
	GameState new_state = game->next;

	/* An overlay rides its base: the 3D world stays untouched and dropping
	   back does not re-enter the base. Only the 2D scene changes hands, and
	   it comes back the way its definition declares it. */
	if (gameState_isOverlayPair(prev, new_state)) {
		game->state = new_state;
#ifdef ENGINE_32_SCENE2D
		if (game_state[new_state].scene2d) scene2d_load(game_state[new_state].scene2d);
#endif
		if (game_state[new_state].overlay_of == prev && game_state[new_state].onEnter)
			game_state[new_state].onEnter();
		return;
	}

	/* The GPU may still be drawing the leaving scene: its primitives and
	   textures stay until it is done. */
	System::get().getGpu().waitChainIdle();
	gameState_unload(prev);
	/* An abandoned overlay takes its base state down with it. */
	if (game_state[prev].overlay_of != GAME_STATE_NONE)
		gameState_unload(game_state[prev].overlay_of);
	game->state = new_state;
	gameState_load(new_state);

	/* The load blocked for as long as it took: none of it counts as a
	   played frame, or the enter animations would swallow it as one. */
	Time::get().reset();
}

void game_start(const GameStateDef *states, uint8_t count, GameState initial)
{
	psyqo::Kernel::assert(states && initial < count, "game: bad state table");

	game_state       = states;
	game_state_count = count;

	Game *game = game_get();
	game->state = initial;
	game->next  = initial;

	gameState_load(initial);
	Time::get().reset();
}

void game_updateState(void)
{
	Game *game = game_get();

	game_state[game->state].update();
	gameState_settle(game);
}
