#ifndef ENGINE_32_CHARACTER3D_CONTROL_H
#define ENGINE_32_CHARACTER3D_CONTROL_H

#include "e32_controller.h"
#include "character3d/e32_character3d.h"
#include "character3d/e32_character3d_movement.h"

#define PLAYER_STICK_WALK_THRESHOLD 65


/* What a body can be asked to do. What a character does not do is left out:
   an unwritten button is BTN_NONE and reads as never pressed. */
struct Character3DControlBinding
{
	/* Whose seat drives this body. */
	PlayerID player;

	ButtonID jump;
	ButtonID roll;
	ButtonID sprint;
	ButtonID aim;
	ButtonID shoot;
	ButtonID weapon_next;
	ButtonID weapon_prev;
};


struct Character3DControls
{
	bool  jump;
	bool  jump_held;
	bool  roll;
	bool  sprint;
	bool  aim;
	bool  shoot;            /* held: the bow draws while it stays down */
	bool  shoot_released;   /* the shot fires on this edge */
	bool  weapon_next;
	bool  weapon_prev;

	/* The left stick, -128..127, up positive: the hardware reports up
	   negative and it is flipped on the way in, so the reading is the one
	   this control was written against. */
	int16_t stick_x;
	int16_t stick_y;
};


/* This frame's state of the buttons the binding names, off that player's
   controller.
   The binding is the mapping and is written once; this only reads what those
   buttons are doing now. */
void character3dControls_read(Character3DControls *controls, const Character3DControlBinding *binding);
void character3dControl_update(Character3D *character, MovementCommand *cmd, const Character3DControls *controls, Angle camera_angle_around);

#endif
