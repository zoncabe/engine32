#ifndef ENGINE_32_CHARACTER3D_MOVEMENT_H
#define ENGINE_32_CHARACTER3D_MOVEMENT_H

#include <stdint.h>
#include "physics/math/e32_math.h"

struct Character3D;


#define LOCOMOTION_MIN_SPEED 0.05_fp

#define CHARACTER3D_ROTATION_SNAP_THRESHOLD 1.0_deg

#define CHARACTER3D_STRAFE_YAW_OFFSET 0.0_deg

#define CHARACTER3D_ROTATION_MODE_LERP 0
#define CHARACTER3D_ROTATION_MODE_SNAP 1

#define CHARACTER3D_JUMP_HOLD_VELOCITY_SCALE 0.96_fp
#define CHARACTER3D_JUMP_LAUNCH_VELOCITY_SCALE 0.8_fp

#define CHARACTER3D_GRAVITY -20.0_fp
#define CHARACTER3D_FALL_MAX_SPEED -15.0_fp

#define CHARACTER3D_WATER_DRAG             4.0_fp    /* vertical, per second, at full submersion */
#define CHARACTER3D_WATER_SINK_MAX_SPEED  -2.0_fp    /* fully reached at the fraction below */
#define CHARACTER3D_WATER_SINK_LIMIT_FULL  0.8_fp    /* submersion where the sink limit saturates */

/* Swim entry is the character's own swim equilibrium: sunk past the depth
   its buoyancy holds it at, the water is already carrying it, so it swims.
   A fixed threshold could sit above that depth, and then a body floating at
   its equilibrium never reaches it — it stays falling on the surface.

   Exit trails entry by this factor, the hysteresis that keeps the waves from
   flickering the state at the boundary. Exit also needs footing. */
#define CHARACTER3D_WATER_SWIM_EXIT_SCALE 0.9_fp

/* Where the body sits while climbing. The clip grips 0.26 m ahead of the
   origin, but holding at that distance buries a capsule of 0.35 radius nine
   centimetres into the rungs, and a body sunk into the ladder reads worse
   than hands closing just short of it. Stand at the radius: the capsule
   comes to rest against the plane of the rungs, where a climber belongs.
   Lower it toward 0.26 to close the hands, raise it to pull clear. */
#define CHARACTER3D_LADDER_STAND_DISTANCE 0.35_fp

/* How hard the body is pulled onto the ladder's centre line and holding
   distance, per second. Fast enough that the entry snap reads as a grab. */
#define CHARACTER3D_LADDER_ANCHOR_RATE    12.0_fp

/* How close the ground has to be for a descent to end on it. */
#define CHARACTER3D_LADDER_GROUND_REACH   0.15_fp

/* Hysteresis on the top of the volume, the same trick the swim thresholds
   use. Leaving happens on losing the volume, so without it the two edges
   are one line: a body standing on the crest has its feet exactly at the
   boundary, and a stick still asking to climb re-grabs the ladder the frame
   after it let go, over and over. Grabbing on has to happen this far below
   the top, which is out of reach of anything already standing on it. Only
   the top edge moves — the foot of the ladder is nowhere near it. */
#define CHARACTER3D_LADDER_ENTER_MARGIN   0.50_fp

/* Widest angle between the stick and the ladder's facing that still counts
   as asking to climb. Past it the stick is walking past the ladder. */
#define CHARACTER3D_LADDER_ENTER_ANGLE    70.0_deg

/* Push toward the rungs when the climb runs off the top of the volume, so
   the body steps onto the landing instead of sliding back down the face it
   was hugging. The climbable volume is what decides where the top is: the
   climb ends where the volume does. */
#define CHARACTER3D_LADDER_EXIT_SPEED     1.6_fp

enum {
	CHARACTER3D_SWIM_GAIT_IDLE,
	CHARACTER3D_SWIM_GAIT_SLOW,
	CHARACTER3D_SWIM_GAIT_FAST,
};


/* How the button becomes height. Charge holds the body down for as long as
   the crouch lasts and launches with what it gathered; snap leaves the floor
   on the press and keeps adding while the button stays down. Zero is charge,
   which is what every character did before the choice existed. */
enum JumpMode {
	JUMP_CHARGE,
	JUMP_SNAP,
};


/* No jumping state: the crouch that starts a jump runs on the ground, over
   locomotion, and the air is always a fall. */
enum MovementState {
	MOVEMENT_STATE_IDLE,
	MOVEMENT_STATE_WALKING,
	MOVEMENT_STATE_ROLLING,
	MOVEMENT_STATE_FALLING,
	MOVEMENT_STATE_SWIMMING,
	MOVEMENT_STATE_CLIMBING,
	MOVEMENT_STATE_COUNT,
	MOVEMENT_STATE_NONE
};

/* One gait phase of the WALKING state. How many and their values are up to
   the caller; the order runs from lowest to highest target_speed. */
struct Character3DGaitSettings
{
	Fixed target_speed;
	Fixed response_rate;
	Fixed rotation_response_rate;
};

struct Character3DMovementSettings
{
	Fixed idle_target_speed;
	Fixed idle_response_rate;
	Fixed idle_rotation_response_rate;

	const Character3DGaitSettings *gait;
	uint8_t gait_count;

	Fixed roll_target_speed;
	Fixed roll_launch_response_rate;
	Fixed roll_spin_response_rate;
	Fixed roll_grip_response_rate;
	Fixed roll_ground_time;
	Fixed roll_grip_time;
	Fixed roll_timer_max;

	JumpMode jump_mode;

	Fixed jump_response_rate;
	/* What the launch is worth on its own: the whole of it under snap, the
	   floor a short charge cannot go under. */
	Fixed jump_base_speed;

	/* Charge only: the crouch lasts this long, and what it gathered is
	   multiplied into the launch. */
	Fixed jump_force_multiplier;
	Fixed jump_timer_max;

	/* Snap only: the fraction of gravity the rise pays while the button is
	   held. Lower climbs higher; at 1.0 holding does nothing. It only ever
	   applies on the way up, so nobody floats down. */
	Fixed jump_hold_gravity_scale;

	/* Snap only: how long the jump still answers after the floor is gone.
	   Walking off a ledge the body coasts for this long before the fall
	   takes it, and the button launches the whole way. Charge needs none of
	   this: its own crouch is the window. */
	Fixed jump_coyote_time;

	/* How much of the ground's steering the air gets, 0 to 1. At zero the
	   body keeps the heading and the speed it left with, and the stick does
	   nothing until it lands; at one it turns and accelerates in the air
	   exactly as it would on the floor. */
	Fixed air_control;

	Fixed swim_slow_speed;
	Fixed swim_fast_speed;
	Fixed swim_response_rate;

	/* Vertical speed on a ladder and how fast it is reached. The climb clip
	   is timed against the first of them: at full speed it plays at its own
	   pace, and slower while the second is still ramping up to it. */
	Fixed climb_speed;
	Fixed climb_response_rate;

	/* Fake buoyancy: submerged fraction of the capsule where the scaled
	   gravity flips sign. The equilibrium follows the pose the clips are
	   authored at: treading water holds the head at the capsule top,
	   stroking holds the body at its middle, and speed slides the target
	   between the two so the swim rides up to the surface.

	   The swim one doubles as the swim state threshold: sunk past it the
	   water carries the body, so that is where the swim begins, and exit
	   trails it by CHARACTER3D_WATER_SWIM_EXIT_SCALE. */
	Fixed water_equilibrium_idle;
	Fixed water_equilibrium_swim;
};

struct Character3DMovementData
{
	Angle previous_yaw;
	Fixed horizontal_speed;
	Fixed roll_timer;
	Vector3 jump_initial_velocity;
	Fixed jump_force;
	Fixed jump_timer;
	/* Time since the floor was lost, counted only while the coyote window is
	   still open. Reset on every landing. */
	Fixed coyote_timer;
	Angle roll_yaw;
	bool is_grounded;
	/* Straight down from the feet, written by the collision pass. Negative
	   with no floor within reach: the animation times the landing on it. */
	Fixed floor_distance;

	bool in_water;
	Fixed submerged_fraction;   /* 0..1 of the capsule under the surface */

	/* Written by the ladder probe of the collision pass. The anchor is where
	   the body has to stand to reach the rungs: the ladder's centre line,
	   pulled out to the clip's holding distance on the side the body is on. */
	bool  on_ladder;
	Angle ladder_yaw;      /* body rotation.z that faces the rungs */
	Fixed ladder_anchor_x;
	Fixed ladder_anchor_y;
	Fixed ladder_top;      /* world z of the climbable volume's ceiling */

	uint8_t rotation_mode;
	bool strafe;
	bool strafe_locked;
	Angle strafe_yaw;

	bool aiming;
	bool charging_shoot;
	bool shooting;

	uint8_t gait;
};

struct MovementCommand
{
	Angle target_yaw;

	bool roll_triggered;

	bool jump_held;
	bool jump_triggered;

	bool strafe;
	bool strafe_locked;
	Angle strafe_yaw;

	bool aiming;
	bool charging_shoot;
	bool shooting;

	uint8_t gait;
	uint8_t swim_gait;   /* CHARACTER3D_SWIM_GAIT_*, from the stick while swimming */
	Fixed speed_scale;   /* 1.0 normal, tired_speed_scale while tired */

	/* Stick along the ladder: +1 climbs, -1 descends, 0 holds. The stick is
	   read in the ladder's own frame, so pushing at the rungs always climbs
	   whichever way the camera happens to look. */
	Fixed climb;
	bool  climb_release;   /* jump button: let go and drop */
};

struct Character3DMovement
{
	const Character3DMovementSettings *settings;
	Character3DMovementData data;
	uint8_t current;
	uint8_t locomotion;
	uint8_t next;
};

void character3d_updateMovement(Character3D *character, MovementCommand *cmd, Fixed dt);
void character3dMovement_setMode(Character3DMovement *movement, uint8_t new_mode);
bool character3dMovement_isLocomotion(uint8_t mode);

/* A crouch under way. The floor probe reads it to leave the body in locomotion
   when the ledge runs out mid-charge, so the jump it was building survives. */
bool character3dMovement_isChargingJump(const Character3D *character);

/* Still inside the coyote window: off the floor, but not for long enough that
   the jump has stopped answering. The control reads it to keep taking the
   button after the ledge. */
bool character3dMovement_isCoyoteOpen(const Character3D *character);

#endif
