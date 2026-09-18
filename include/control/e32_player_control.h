#ifndef ENGINE_32_PLAYER_CONTROL_H
#define ENGINE_32_PLAYER_CONTROL_H

#include "e32_character3d_control.h"
#include "e32_camera_control.h"
#include "e32_menu_control.h"

typedef struct Viewport Viewport;
typedef struct Player   Player;

struct Scene3DDef;


/* Everything this game drives with, handed over once with the state that runs
   on it, the way the scene is. Each binding names what it moves, so from here
   on the engine seats the players and points the camera by itself, every time
   a scene is loaded. */
struct ControlsDef
{
	const CameraControlBinding      *camera;
	const Character3DControlBinding *character3d;
	const MenuControlBinding        *menu;
};

/* Wires what the bindings name against what this scene declared: the seat goes
   to the character built from the prefab they point at, and the camera answers
   if it is the one they point at. Called by the engine when a state is loaded,
   with the controls that state declared. */
void controls_bind(const ControlsDef *controls, const Scene3DDef *scene);

/* Reads the buttons this player was seated with and turns them into its
   command for this frame, aimed by the camera. */
void player_setCharacter3DControl(PlayerID id, Viewport *viewport);

#endif
