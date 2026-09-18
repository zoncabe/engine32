/*
	A body in a room with things in it: walk it, run it, jump it, and push it
	against them. Everything the character does comes from its settings and the
	solver; the engine has no movement of its own.

	The prefabs live one per file under prefabs/, the light, the fog and the
	camera under scene/, the button bindings under controls/. What is here is
	the placement table and the one state that runs the frame.
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
#include "time/e32_time.h"
#include "debug/e32_debug.h"


/* --- prefabs ---------------------------------------------------------------
	One file each, under prefabs/: the room, the body that walks it and the
	three things it pushes against. The camera, the light and the fog the scene
	runs with come from scene/, the bindings that drive it from controls/.
*/

extern const Prefab3D room;

extern const Prefab3D capsule;
extern const Prefab3D cube;
extern const Prefab3D sphere;

extern const Prefab3D character;

extern const CameraDef camera;
extern const LightDef  light;
extern const FogDef    fog;

extern const ControlsDef controls;


/* --- the scene -------------------------------------------------------------
	One row per placement: which prefab, then where it stands. Declaring them
	here is the whole job: the load builds each one in order, registers it in
	the physics and draws it, with nothing else to call. What is left out stays
	zero, and a zero scale means original size.
*/
static const Scene3DPrefab scene_prefabs[] = {

	{ &character, { 0.0_fp, -6.0_fp, 0.0_fp } },

	/* Three still bodies in a row. The cube and the ball are modelled around
	   their middle, so at double size they sit a metre up to rest on the
	   floor. The scale carries their collision with it. */
	{ &capsule, {   0.0_fp, 0.0_fp, 0.0_fp } },
	{ &cube,    { -10.0_fp, 0.0_fp, 1.0_fp }, {}, { 2.0_fp, 2.0_fp, 2.0_fp } },
	{ &sphere,  {  10.0_fp, 0.0_fp, 1.0_fp }, {}, { 2.0_fp, 2.0_fp, 2.0_fp } },

	{ &room },
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

static void GameStateExample_update(void)
{
	Scene3D  *scene3d  = scene3d_get();
	Viewport &viewport = Viewport::get();
	Fixed     delta    = Time::get().delta;

	player_setCharacter3DControl(PLAYER_1, &viewport);
	player_update();

	/* A character is not placed by the solver: it collides itself against the
	   world the solver just settled, and from there reaches what draws it. */
	scene3d_updateCharacters(viewport.fb_index);

	/* Model matrices are kept per framebuffer: each frame writes the one it
	   is about to draw with. */
	for (int i = 0; i < scene3d->entity_count; i++)
		entity3d_setMatrixFromBody(scene3d->entity[i], viewport.fb_index);

	/* The arm hangs off the body the player drives. */
	Vector3 center = scene3d_getCharacter3D(0)->entity->transform.position;

	CameraControl::update(&viewport.camera, viewport.camera.binding, scene3d, delta);
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
		.scene3d       = &scene,

		/* Wired once, after the scene is loaded and before the first update:
		   the player is seated on the body its binding names, and the camera
		   answers to the buttons that name it. */
		.controls      = &controls,

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
