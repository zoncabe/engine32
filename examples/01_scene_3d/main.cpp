/*
	The smallest world engine32 can put on screen: a room, a capsule standing in
	the middle of it, and a camera to look around.

	Everything here is content: the assets, where they are placed, and the one
	game state that draws them. The engine supplies the rest.
*/
#include "game/e32_game.h"
#include "scene3d/e32_scene3d.h"
#include "entity/e32_entity3d.h"
#include "viewport/e32_viewport.h"
#include "control/e32_controller.h"
#include "control/e32_camera_control.h"
#include "time/e32_time.h"
#include "debug/e32_debug.h"


/* --- prefabs ---------------------------------------------------------------
	A prefab is a model plus what its kind needs. A prop with no collider and
	no body is only drawn.
*/

static const Prefab3D room    = { .type = PREFAB3D_PROP, .model = "rom:/models/room.model", .subdivide = true };
static const Prefab3D capsule = { .type = PREFAB3D_PROP, .model = "rom:/models/capsule.model" };


/* --- the scene -------------------------------------------------------------
	What the world contains and where. Distances are meters, angles degrees.
*/

/* One row per placement: which prefab, then where it stands. Declaring them
   here is the whole job — the load builds each one in order and draws it,
   with nothing else to call. What is left out stays zero, and a zero scale
   means original size. */
static const Scene3DPrefab scene_prefabs[] = {

	{ &room,    { 0.0_fp, 0.0_fp, 0.0_fp } },
	{ &capsule, { 0.0_fp, 0.0_fp, 0.0_fp } },
};

static const CameraDef camera = {

	.type = CAMERA_TYPE_SPRING_ARM,

	.field_of_view = 60.0_deg,
	.near_clipping = 1.0_fp,
	.far_clipping  = 50.0_fp,

	.spring_arm = {
		.arm_length   = 6.0_fp,
		.side_offset  = 0.0_fp,
		.yaw          = -45.0_deg,
		.pitch        = 15.0_deg,
		.pivot_height = 1.0_fp,

		.settings = {
			.response_rate = { 10.0_fp, 10.0_fp },
			.max_velocity  = { Fixed(60.0_deg), Fixed(40.0_deg) },
			.direction     = {  1.0_fp, -1.0_fp },
			.zoom_response_rate = 6.0_fp,
			.distance_speed = 4.0_fp,
			.fov_speed      = Fixed(30.0_deg),
			.max_pitch     =  80.0_deg,
			.min_pitch     = -50.0_deg,
		},
	},
};

/* Three slots, shared between directional and point lights. They are read in
   order and cut at the first empty one, so the two left over cost nothing. */
static const LightDef light = {

	.ambient_color = {{60, 60, 70}},

	.source = {
		{ .type  = LIGHT_POINT,
		  .color = {{255, 245, 220}},
		  .point = { .position = { 0.0_fp, 0.0_fp, 7.0_fp }, .size = 25.0_fp } },
	},
};

static const FogDef fog = { .enabled = false };

static const Scene3DDef scene = {

	.light  = &light,
	.fog    = &fog,
	.camera = &camera,

	.prefab       = scene_prefabs,
	.prefab_count = sizeof(scene_prefabs) / sizeof(scene_prefabs[0]),
};


/* --- the state -------------------------------------------------------------
	A state is one mode of the game: the title menu, the match, the pause,
	the credits. One at a time. Each carries the scene it draws, the sprites
	and fonts it needs, and the function the engine calls every frame. Leaving
	a state frees all of that, and the next one loads its own.

	This table is the whole game as far as the engine is concerned: it takes
	it at startup and from then on the only thing it ever calls is the update
	of the current state. That is why there is one even here, for a single
	mode that never changes.
*/

enum { GAME_STATE_EXAMPLE, STATE_COUNT };

static Vector3 room_center = { 0.0_fp, 0.0_fp, 1.0_fp };

/* Naming the sticks and buttons is the whole job: the engine reads this
   every frame and turns, pulls back and zooms the camera on its own.
   Anything left at BTN_NONE simply never happens. */
static const CameraControlBinding camera_binding = {

	.player = PLAYER_1,

	.pan  = AXIS_RIGHT_X,
	.tilt = AXIS_RIGHT_Y,

	.distance_in  = BTN_L1,
	.distance_out = BTN_R1,

	.fov_in    = BTN_D_UP,
	.fov_out   = BTN_D_DOWN,
};

static void GameStateExample_update(void)
{
	Scene3D *scene3d = scene3d_get();
	uint8_t fb = Viewport::get().fb_index;

	/* Model matrices are kept per framebuffer: each frame writes the one it
	   is about to draw with. */
	for (int i = 0; i < scene3d->entity_count; i++)
		entity3d_setMatrixFromBody(scene3d->entity[i], fb);

	CameraControl::update(&Viewport::get().camera, &camera_binding, scene3d, Time::get().delta);
	Viewport::get().updateCamera(&room_center, scene3d);
	Viewport::get().setPerspectiveCamera();

	debugUI_showFPS();

	/* Select toggles the wireframe. */
	const Controller *pad = &controller_get()[0];
	if (button_getPressed(pad, pad->pressed, BTN_SELECT) != 0)
		Render::get().setWireframe(!Render::get().wireframe());
}

static const GameStateDef states[STATE_COUNT] = {

	[GAME_STATE_EXAMPLE] = {
		.update     = GameStateExample_update,
		.scene3d    = &scene,
		.overlay_of = GAME_STATE_NONE,
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
