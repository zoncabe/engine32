/*
	What each button does, one binding per thing that moves.

	A binding is a mapping plus a target: the player whose controller is read,
	the piece it moves, and a button or an axis per action the module offers.
	The engine reads them when the state is loaded, so nothing here is called
	from the game.

	An action left out never fires. The left stick is not bound in either of
	them: it always drives the body, and the camera takes its angle from where
	it already is.
*/
#include "control/e32_player_control.h"

#include "camera/e32_camera.h"
#include "prefab/e32_prefab3d.h"


extern const CameraDef camera;
extern const Prefab3D  character;


/* Naming the camera here is what hands it these buttons: it answers them only
   while the scene it was placed from is the one running. */
extern const CameraControlBinding camera_binding = {

	.player = PLAYER_1,
	.camera = &camera,

	.pan  = AXIS_RIGHT_X,
	.tilt = AXIS_RIGHT_Y,

	.distance_in  = BTN_L1,
	.distance_out = BTN_R1,

	.fov_in    = BTN_D_UP,
	.fov_out   = BTN_D_DOWN,
};

/* The prefab it drives is what seats the player: the scene builds one body
   from it, and that body is the one this controller moves.

   Only what this body can do. Aiming, shooting and weapon switching are left
   out, so those buttons read as never pressed. */
extern const Character3DControlBinding character3d_binding = {

	.player    = PLAYER_1,
	.character = &character,

	.jump   = BTN_CROSS,
	.sprint = BTN_SQUARE,
};

/* The state names this, and the engine wires both when it is loaded. */
extern const ControlsDef controls = {

	.camera      = &camera_binding,
	.character3d = &character3d_binding,
};
