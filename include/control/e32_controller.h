/*
	The pads, as the hardware hands them over. What any of it means is the
	game's to decide: it holds its own bindings and reads them off this with
	button_getPressed and axis_get.

	PSYQo's AdvancedPad drives the ports; this snapshots it once per frame so
	edges (pressed, released) exist, and so every read in a frame sees the
	same state.
*/
#ifndef ENGINE_32_CONTROLLER_H
#define ENGINE_32_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "physics/math/e32_fixed.h"


typedef enum {

	/* Unbound: reads as never pressed. Zero on purpose, so an action left
	   out of a binding comes out with no button instead of on the cross. */
	BTN_NONE,

	BTN_CROSS, BTN_CIRCLE, BTN_SQUARE, BTN_TRIANGLE,
	BTN_START, BTN_SELECT,
	BTN_D_UP, BTN_D_DOWN, BTN_D_LEFT, BTN_D_RIGHT,
	BTN_L1, BTN_R1, BTN_L2, BTN_R2, BTN_L3, BTN_R3,
	BTN_COUNT,

} ButtonID;

/* The two sticks, one axis each. Unbound reads as centred. */
typedef enum {

	AXIS_NONE,

	AXIS_LEFT_X, AXIS_LEFT_Y,
	AXIS_RIGHT_X, AXIS_RIGHT_Y,
	AXIS_COUNT,

} AxisID;

/* Centred, a stick does not rest at zero: it wanders by a few units of its
   128. Under this it reads as centred, so that wander reaches nothing. */
#define STICK_DEADZONE 12

/* Who a binding belongs to. The port and the player are the same index: the
   first controller drives the first player. */
typedef enum {

	PLAYER_1, PLAYER_2,
	PLAYER_COUNT,

} PlayerID;

/* One bit per button, in the hardware's own order. */
typedef uint16_t Buttons;

/* The sticks, -128..127 with the centre at zero; all zero on a pad
   without them. Up is negative, as the hardware reports it. */
struct ControllerInputs
{
	int8_t left_x;
	int8_t left_y;
	int8_t right_x;
	int8_t right_y;
};

/* The controller as the hardware hands it over. What any of it means is the
   game's to decide: it holds its own bindings and its own set of actions, and
   reads them off this with button_getPressed. */
struct Controller
{
	Buttons pressed;
	Buttons held;
	Buttons released;
	ControllerInputs input;

	/* The frame the left stick crosses into a direction, -1, 0 or +1 per
	   axis. A stick has no edge of its own, so this keeps one the way a
	   button's is kept: it tells pushing it now from having held it pushed
	   since before. */
	int8_t stick_pressed_x;
	int8_t stick_pressed_y;

	/* No controller in this port. An absent one reads as all zeroes, which is a
	   valid answer for an edge but not for a continuous value: the seats
	   behind the first would write their zero over whatever it set. */
	bool connected;
	bool analog;
};

#define CONTROLLER_COUNT PLAYER_COUNT

Controller *controller_get(void);
void controller_start(void);
void controller_poll(void);

/* Whether a bound button is being pushed, 0 or 1. */
Fixed button_getPressed(const Controller *controller, Buttons buttons, ButtonID id);

/* A bound axis, -1 to 1: nothing inside the dead zone, then the rest of the
   travel spread over the range. */
Fixed axis_get(const Controller *controller, AxisID id);

#endif
