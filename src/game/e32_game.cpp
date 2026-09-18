#include "psyqo/scene.hh"

#include "viewport/e32_viewport.h"
#include "time/e32_time.h"
#include "scene3d/e32_scene3d.h"
#include "render/e32_render.h"
#include "graphics/e32_texture.h"
#include "control/e32_controller.h"
#include "game/e32_game.h"
#include "game/e32_game_states.h"
#include "system/e32_system.h"
#ifdef ENGINE_32_PLAYER
#include "player/e32_player.h"
#include "control/e32_player_control.h"
#endif
#ifdef ENGINE_32_SOUND
#include "sound/e32_sound.h"
#endif
#ifdef ENGINE_32_MENU
#include "menu/e32_settings.h"
#include "menu/e32_menu.h"
#endif
#ifdef ENGINE_32_PARTICLES
#include "particles/e32_particles.h"
#endif
#ifdef E32_TRACE
#include "common/hardware/counters.h"
#include "debug/e32_debug.h"

uint32_t debug_profile_state;
uint32_t debug_profile_input;
uint32_t debug_profile_setup;
uint32_t debug_profile_end;
#endif


static Game game;


Game *game_get(void) { return &game; }


/* The engine's one PSYQo scene: every frame is a runStep. */
class GameScene final : public psyqo::Scene
{
	void frame() override { game_runStep(); }
};

static GameScene game_scene;

static void (*game_setup)();


static void game_init()
{
	controller_start();

	Time::get().init();

	Viewport::get().init();

	Texture::init();

	Render::get().init();

#ifdef ENGINE_32_PARTICLES
	particles_init();
#endif

#ifdef ENGINE_32_PLAYER
	player_init();
#endif

#ifdef ENGINE_32_MENU
	settings_init();

	menuStack_init();
#endif

	/* The game's own inits and game_start (state table, initial state)
	   run after this, from its setup. */
}

/* Called by the system once the GPU is up. */
static void game_ready()
{
	game_init();

	if (game_setup) game_setup();

	System::get().pushScene(&game_scene);
}

int game_run(void (*setup)())
{
	game_setup = setup;
	return System::get().run(game_ready);
}

void game_runStep(void)
{
#ifdef ENGINE_32_SOUND
	sound_poll();
#endif

#ifdef E32_TRACE
	uint16_t t0 = (uint16_t)COUNTERS[2].value;
#endif

	Time::get().update();

	controller_poll();

#ifdef E32_TRACE
	uint16_t t1 = (uint16_t)COUNTERS[2].value;
	debug_profile_input += (uint16_t)(t1 - t0);
#endif

	game_updateState();

#ifdef E32_TRACE
	debug_profile_state += (uint16_t)((uint16_t)COUNTERS[2].value - t1);
#endif

#ifdef ENGINE_32_SOUND
	sound_update();
#endif

	Render::get().frame();
}

void game_close()
{
}
