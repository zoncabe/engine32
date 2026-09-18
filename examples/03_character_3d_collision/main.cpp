/*
	A body in a room: walk it, run it, jump it, push it into the water and up
	the ladder. Everything the character does comes from its settings and the
	solver; the engine has no movement of its own.

	The prefabs live one per file under prefabs/. What is here is the world:
	where each one stands, the light, the camera and the one state that runs
	the frame.
*/
#include "game/e32_game.h"
#include "scene3d/e32_scene3d.h"
#include "entity/e32_entity3d.h"
#include "viewport/e32_viewport.h"
#include "player/e32_player.h"
#include "control/e32_controller.h"
#include "control/e32_camera_control.h"
#include "control/e32_character3d_control.h"
#include "control/e32_player_control.h"
#include "character3d/e32_character3d.h"
#include "shaders/e32_water.h"
#include "time/e32_time.h"
#include "debug/e32_debug.h"


/* --- prefabs ---------------------------------------------------------------
	One file each, under prefabs/: the seven pieces of content, and the camera,
	the light and the fog the scene runs with.
*/

extern const Prefab3D room;

extern const Prefab3D water;
extern const Prefab3D ladder;

extern const Prefab3D capsule;
extern const Prefab3D cube;
extern const Prefab3D sphere;

extern const Prefab3D character;

extern const CameraDef camera;
extern const LightDef  light;
extern const FogDef    fog;


/* --- the scene -------------------------------------------------------------
	The room is 50 metres across, five quads of ten a side. The platform
	rises 5 in one corner, the mound peaks at 2 across from it, and the pool
	is sunk 2.5 into the middle.
*/

/* One row per placement: which prefab, then where it stands. Declaring them
   here is the whole job: the load builds each one in order, registers it in
   the physics and draws it, with nothing else to call. What is left out stays
   zero, and a zero scale means original size. */
static const Scene3DPrefab scene_prefabs[] = {

	{ &character, { 20.0_fp, -20.0_fp, 0.0_fp }, { 0.0_deg, 0.0_deg, 135.0_deg } },

	/* Three still bodies in a row, parallel to the water's edge. The cube and
	   the ball are modelled around their middle, so at double size they sit
	   a metre up to rest on the floor. The scale carries their collision
	   with it. */
	{ &capsule, {   0.0_fp, 0.0_fp, 0.0_fp } },
	{ &cube,    { -10.0_fp, 0.0_fp, 1.0_fp }, {}, { 2.0_fp, 2.0_fp, 2.0_fp } },
	{ &sphere,  {  10.0_fp, 0.0_fp, 1.0_fp }, {}, { 2.0_fp, 2.0_fp, 2.0_fp } },

	/* Halfway along the platform's east face, facing the mound, 5 tall,
	   which is exactly the climb. Stood off the wall on purpose: flush
	   against it the body meets the platform before it can reach the volume
	   it grabs, and the climb never starts. */
	{ &ladder, { -4.8_fp, -10.0_fp, 0.0_fp }, { 0.0_deg, 0.0_deg, -90.0_deg } },

	{ &room },

	/* Last on purpose: the water is transparent, so it has to blend over
	   everything already drawn. */
	{ &water, { 0.0_fp, 10.0_fp, -0.5_fp } },
};

static const Scene3DDef scene = {

	.light  = &light,
	.fog    = &fog,
	.camera = &camera,

	.prefab       = scene_prefabs,
	.prefab_count = sizeof(scene_prefabs) / sizeof(scene_prefabs[0]),
};


/* --- the state -------------------------------------------------------------
	A state is one mode of the game: it carries the scene it draws and the
	function the engine calls every frame. This one is the only mode here,
	and its update is where the body is driven and the camera follows it.
*/

static const CameraControlBinding camera_binding = {

	.player = PLAYER_1,

	.pan  = AXIS_RIGHT_X,
	.tilt = AXIS_RIGHT_Y,

	.distance_in  = BTN_L1,
	.distance_out = BTN_R1,

	.fov_in    = BTN_D_UP,
	.fov_out   = BTN_D_DOWN,
};

static const Character3DControlBinding character3d_binding = {

	.player = PLAYER_1,

	.jump   = BTN_CROSS,
	.roll   = BTN_CIRCLE,
	.sprint = BTN_SQUARE,
};

/* The scene loads its characters in placement order; this one is the only one,
   so the player declared on the binding takes the only one sitting at index 0. */
static void GameStateExample_bindCharacter(void)
{
	player_setCharacter3D(scene3d_getCharacter3D(0), &character3d_binding);
}

static void GameStateExample_update(void)
{
	Scene3D  *scene3d  = scene3d_get();
	Viewport &viewport = Viewport::get();
	Fixed     delta    = Time::get().delta;

	player_setCharacter3DControl(PLAYER_1, &viewport);
	player_update();

	water_update(delta);

	/* A character is not placed by the solver: it collides itself against the
	   world the solver just settled, and from there reaches what draws it. */
	scene3d_updateCharacters(viewport.fb_index);

	/* Model matrices are kept per framebuffer: each frame writes the one it
	   is about to draw with. */
	for (int i = 0; i < scene3d->entity_count; i++)
		entity3d_setMatrixFromBody(scene3d->entity[i], viewport.fb_index);

	/* The arm hangs off the body the player drives. */
	Vector3 center = scene3d_getCharacter3D(0)->entity->transform.position;

	CameraControl::update(&viewport.camera, &camera_binding, scene3d, delta);
	viewport.updateCamera(&center, scene3d);
	viewport.setPerspectiveCamera();

	debugUI_showFPS();

	/* Select toggles the wireframe. */
	const Controller *pad = &controller_get()[0];
	if (button_getPressed(pad, pad->pressed, BTN_SELECT) != 0)
		Render::get().setWireframe(!Render::get().wireframe());
}

enum { GAME_STATE_EXAMPLE, STATE_COUNT };

static const GameStateDef states[STATE_COUNT] = {

	[GAME_STATE_EXAMPLE] = {
		.update        = GameStateExample_update,
		.bindCharacter = GameStateExample_bindCharacter,
		.scene3d       = &scene,
		.overlay_of    = GAME_STATE_NONE,
	},
};


/* Once the hardware is up: the game's own inits, then the state table. */
static void setup()
{
	debugUI_init();

	game_start(states, STATE_COUNT, GAME_STATE_EXAMPLE);
}

int main()
{
	return game_run(setup);
}
