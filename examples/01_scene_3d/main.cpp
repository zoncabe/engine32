/*
	The smallest world engine32 can put on screen: a room, the logo standing in
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
#include "control/e32_player_control.h"
#include "camera/e32_camera.h"
#include "physics/math/e32_trig.h"
#include "time/e32_time.h"
#include "graphics/e32_mesh.h"
#include "debug/e32_debug.h"


/* --- prefabs ---------------------------------------------------------------
	A prefab is a model plus what its kind needs. A prop with no collider and
	no body is only drawn.
*/

/* The smallest prefab there is: a type and a model, drawn as one piece. */
static const Prefab3D room = {

	.type      = PREFAB3D_PROP,
	.model     = "rom:/models/room.model",
	.subdivide = true,
};

/* ps_logo.obj is one object per colour of the logo, named after its material.
   Listing them as parts is what lets the update switch each colour off on its
   own; none of them is displaced, so they are all drawn where they were
   modelled and there are no positions to give. */
static const Prefab3D ps_logo = {

	.type  = PREFAB3D_PROP,
	.model = "rom:/models/ps_logo.model",

	.part = MESH_PARTS(
		"red",
		"green",
		"blue",
		"yellow"
	),

	.part_count = 4,
};

/* --- the scene -------------------------------------------------------------
	What the world contains and where. Distances are meters, angles degrees.
*/

/* One row per placement: which prefab, then where it stands. Declaring them
   here is the whole job — the load builds each one in order and draws it,
   with nothing else to call. What is left out stays zero, and a zero scale
   means original size. */
static const Scene3DPrefab scene_prefabs[] = {

	{ &room,    { 0.0_fp, 0.0_fp, 0.0_fp } },
	{ &ps_logo, { 0.0_fp, 0.0_fp, 1.0_fp }, { 0.0_deg, 0.0_deg, 45.0_deg } },
};

static const CameraDef camera = {

	.type = CAMERA_TYPE_SPRING_ARM,

	.field_of_view = 60.0_deg,   /* vertical lens angle, between 10 and 120 */
	.near_clipping =  1.0_fp,    /* closer than this is not drawn           */
	.far_clipping  = 50.0_fp,    /* farther than this is not drawn          */

	.spring_arm = {
		.arm_length   = 6.0_fp,      /* distance from the target, in metres        */
		.side_offset  = 0.0_fp,      /* shifts the arm sideways, for over-shoulder */
		.yaw          = 45.0_deg,   /* starting orbit angle                       */
		.pitch        = 15.0_deg,    /* starting elevation                         */
		.pivot_height = 1.0_fp,      /* raises the anchor above the target point   */

		.settings = {
			/* The arm does not jump to where the controls ask: it accelerates
			   towards it. One value per axis, yaw then pitch. */
			.response_rate = { 10.0_fp, 10.0_fp },                  /* how hard it accelerates */
			.max_velocity  = { Fixed(120.0_deg), Fixed(100.0_deg) },/* degrees per second cap  */
			.direction     = { 1.0_fp, 1.0_fp },                    /* -1 inverts that axis    */

			.zoom_response_rate = 6.0_fp,          /* how fast the arm settles at a new length */
			.distance_speed     = 4.0_fp,          /* metres per second while zooming          */
			.fov_speed          = Fixed(30.0_deg), /* degrees per second while changing fov    */

			.max_pitch =  80.0_deg,   /* how far above the target it can climb */
			.min_pitch = -50.0_deg,   /* and how far below it can drop         */
		},
	},
};

/* One button per camera action, and the camera they move. The engine reads
   this binding every frame and applies the motion itself, so the game never
   moves the camera. Any action left unset never triggers. */
static const CameraControlBinding camera_binding = {

	.player = PLAYER_1,
	.camera = &camera,

	.pan  = AXIS_RIGHT_X,
	.tilt = AXIS_RIGHT_Y,

	.distance_in  = BTN_L1,
	.distance_out = BTN_R1,

	.fov_in    = BTN_D_UP,
	.fov_out   = BTN_D_DOWN,
};

/* What this state drives with, named in its declaration further down. Each
   binding names the piece it moves, so entering the state is all it takes for
   the engine to wire them: there is nothing to bind by hand. */
static const ControlsDef controls = {

	.camera = &camera_binding,
};

/* --- the light -------------------------------------------------------------
	Ambient is the flat colour every surface keeps regardless of what reaches
	it, and the only thing lighting the faces no light hits.

	The slots are shared by every light type. They are read in order and stop
	at the first empty one, so a scene only pays for what it declares.

	A point light radiates in all directions from .position, and .size is the
	radius it reaches. This one hangs over the logo.
*/
static const LightDef light = {

	.ambient_color = {{60, 60, 70}},

	.source = {
		{ .type  = LIGHT_POINT,
		  .color = {{255, 245, 220}},
		  .point = { .position = { 0.0_fp, 0.0_fp, 4.0_fp },
		             .size = 25.0_fp } },
	},
};

/* Distance fog: a surface blends towards .color the farther it is from the
   camera, untouched up to .near and fully replaced past .far. Both in metres,
   measured along the view axis.

   Keep the range inside the camera's far plane. Fog that saturates past it
   never finishes, and geometry is cut at the plane anyway, so the room's back
   wall would pop out of a haze that never closed.

   Enabling fog also paints the background: the frame is cleared to .color, so
   what fades out in the distance matches what is behind it. With fog off the
   background is black. */
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

enum { GAMEPLAY3D, STATE_COUNT };

/* The point the camera orbits, moved with the stick. Metres per second at full
   push: the axis arrives normalised, so this is the speed itself. */
#define CAMERA_TARGET_SPEED 12.0_fp

/* Starts on the logo, which is what this example has to show. The stick walks
   it from there. */
static Vector3 camera_target = { 0.0_fp, 0.0_fp, 1.5_fp };

/* Part visibility is set, not toggled, so the game keeps the current value of
   each one. Every part is visible when the scene loads. */
static const char *const logo_part[]  = { "red", "green", "blue", "yellow" };
static const ButtonID    logo_button[] = { BTN_CROSS, BTN_CIRCLE, BTN_SQUARE, BTN_TRIANGLE };
static bool              draw_logo[]  = { true, true, true, true };

/* The state update: the only place this game's own code runs. Called once per
   frame while this state is current, after the controllers are polled and the
   physics has stepped, before the frame is drawn.

   It does two things: switches each colour of the logo on and off, and walks
   the point the camera orbits across the floor. */
static void gameplay3d_update(void)
{
	Scene3D *scene3d = scene3d_get();
	Viewport &viewport = Viewport::get();

	/* While the seams are being looked for: black hides a hole in dark
	   geometry, this does not. */
	viewport.setClearColor(0, 255, 0);

	/* The controller, already polled by the engine this frame, and how long
	   the previous frame took, in seconds. */
	const Controller *pad = &controller_get()[PLAYER_1];
	Fixed delta = Time::get().delta;

	/* Buttons are read directly here, unlike the camera above.

	   Moving a camera is engine work, so it is declared as a binding and the
	   engine does it. There is a binding type per module that moves something:
	   camera, 3D character, menu. Hiding a piece of a model belongs to no
	   module, so there is nothing to bind it to and the game reads the button
	   itself.

	   pad->pressed is the set that went down on this frame, which is what a
	   toggle needs; pad->held would fire every frame.

	   entity[1] is the logo because it is the second row of the placement
	   table. Parts are addressed by the name the prefab declared, and setting
	   a name the entity does not have does nothing. */
	for (int i = 0; i < 4; i++) {
		if (button_getPressed(pad, pad->pressed, logo_button[i]) == 0) continue;
		draw_logo[i] = !draw_logo[i];
		entity3d_setPartVisible(scene3d->entity[1], logo_part[i], draw_logo[i]);
	}

	/* Raw stick values run from -128 to 127 and never rest at exactly zero, so
	   anything under the deadzone is discarded.

	   The stick vector is then rotated by the camera's angle around the target
	   before being applied, which makes up on the stick mean away from the
	   camera at any orbit angle. A character controller does the same thing. */
	Fixed x =  axis_get(pad, AXIS_LEFT_X);
	Fixed y = -axis_get(pad, AXIS_LEFT_Y);   /* up on the stick reads negative */

	if (x.raw() != 0 || y.raw() != 0) {
		Angle angle = viewport.camera.getAngleAround(&camera_target);
		Fixed sin_a, cos_a;
		Trig::sincos(angle, &sin_a, &cos_a);

		camera_target.x += (x * cos_a + y * sin_a) * CAMERA_TARGET_SPEED * delta;
		camera_target.y += (y * cos_a - x * sin_a) * CAMERA_TARGET_SPEED * delta;
	}

	/* Model matrices are kept per framebuffer: each frame writes the one it
	   is about to draw with. */
	for (int i = 0; i < scene3d->entity_count; i++)
		entity3d_setMatrix(scene3d->entity[i], viewport.fb_index);

	/* Hands the camera the point to orbit for this frame. Reading the buttons,
	   accelerating the arm and applying the result all happen inside. */
	CameraControl::update(&viewport.camera, viewport.camera.binding, scene3d, delta);
	viewport.updateCamera(&camera_target, scene3d);
	viewport.setPerspectiveCamera();

	/* Debug lines: numbered slots down the left, the rate and the camera's arm
	   and lens down the right. Each line has to be rewritten every frame;
	   nothing persists. */
	/* Off while the frame's cost is being measured: the seven lines are some
	   160 glyphs, and the font draws a sprite per glyph, none of which the
	   render's counters see. The rate stays on, to read on screen. */
#if 0
	debugUI_set(0, "CROSS red    CIRCLE green");
	debugUI_set(1, "SQUARE blue  TRIANGLE yellow");

	debugUI_set(3, "STICK move camera target");
	debugUI_set(4, "RSTICK orbit camera");
	debugUI_set(5, "L1 R1 arm length");
	debugUI_set(6, "DPAD fov");
	debugUI_set(7, "SELECT wireframe");
#endif

	debugUI_showFPS();

	/* Select toggles the wireframe. */
	if (button_getPressed(pad, pad->pressed, BTN_SELECT) != 0)
		Render::get().setWireframe(!Render::get().wireframe());
}

static const GameStateDef states[STATE_COUNT] = {

	[GAMEPLAY3D] = {
		.update     = gameplay3d_update,
		.scene3d    = &scene,

		/* Wired once, after the scene is loaded and before the first update:
		   the camera answers to the buttons that name it. */
		.controls   = &controls,

	},
};


/* Measuring what the subdivision grid costs: two pieces along every edge
   within 24 metres, in place of the built in four within twelve. Four
   pieces is sixteen cells and twenty five grid points per face; two is
   four cells and nine points. */
static const MeshLod lod[] = { { 24.0_fp, 2 } };

/* Once the hardware is up: the game's own inits, then the state table. */
static void setup()
{
	debugUI_init();
	//mesh_setLod(lod, 1);

	game_start(states, STATE_COUNT, GAMEPLAY3D);
}

int main()
{
	return game_run(setup);
}
