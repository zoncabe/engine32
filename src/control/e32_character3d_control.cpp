#include <stddef.h>

#include "entity/e32_entity3d.h"
#include "control/e32_character3d_control.h"
#include "physics/math/e32_trig.h"


static void character3dControl_setJump(Character3D *character, MovementCommand *cmd, const Character3DControls *actions)
{
	Character3DMovement *movement = &character->movement;

	/* The button never changes the state: it only asks for a jump. The crouch
	   runs on the ground and the movement switches to the air on the impulse.
	   The coyote window counts as ground for this: the ledge is gone but the
	   ask is still taken. */
	if (actions->jump
	    && (character3dMovement_isLocomotion(movement->current)
	        || character3dMovement_isCoyoteOpen(character))
	    && movement->data.jump_timer == 0) {
		cmd->jump_held      = true;
		cmd->jump_triggered = true;
	} else if (actions->jump_held) {
		return;
	} else {
		cmd->jump_held = false;
	}
}

static void character3dControl_setRoll(Character3D *character, MovementCommand *cmd, const Character3DControls *actions)
{
	Character3DMovement *movement = &character->movement;

	/* Not while a jump is being charged: the crouch owns the body until it
	   takes off. */
	if (actions->roll && character3dMovement_isLocomotion(movement->current)
	    && movement->current != MOVEMENT_STATE_IDLE
	    && movement->data.jump_timer == 0) {
		cmd->roll_triggered = true;
		character3dMovement_setMode(movement, MOVEMENT_STATE_ROLLING);
	}
}

/* Only with both feet in ordinary locomotion: never mid-air, mid-roll, in
   the water or on a ladder, and not while a jump crouch owns the body. */
static void character3dControl_setWeaponSwitch(Character3D *character, const Character3DControls *actions)
{
	Character3DMovement *movement = &character->movement;

	if (!character3dMovement_isLocomotion(movement->current)
	    || movement->data.jump_timer != 0) return;

	if (actions->weapon_next) character3d_cycleWeapon(character, +1);
	if (actions->weapon_prev) character3d_cycleWeapon(character, -1);
}

static inline bool character3dControl_stickPushed(int16_t axis)
{
	return axis >= STICK_DEADZONE || axis <= -STICK_DEADZONE;
}

static void character3dControl_setLocomotionWithStick(Character3D *character, MovementCommand *cmd, const Character3DControls *actions, Angle camera_angle_around)
{
	Character3DMovement *movement = &character->movement;

	/* The stick's length is only ever compared, so it stays squared, in
	   the stick's own integer units. */
	int32_t stick_squared = 0;

	if (character3dControl_stickPushed(actions->stick_x) || character3dControl_stickPushed(actions->stick_y)) {
		int32_t x = actions->stick_x;
		int32_t y = actions->stick_y;
		stick_squared   = x * x + y * y;
		cmd->target_yaw = Trig::atan2(Fixed(x, 0), Fixed(-y, 0)) - camera_angle_around;
	}

	/* Swimming keeps its state; the stick only picks the swim gait. */
	if (movement->current == MOVEMENT_STATE_SWIMMING) {
		if (stick_squared == 0)    cmd->swim_gait = CHARACTER3D_SWIM_GAIT_IDLE;
		else if (actions->sprint)  cmd->swim_gait = CHARACTER3D_SWIM_GAIT_FAST;
		else                       cmd->swim_gait = CHARACTER3D_SWIM_GAIT_SLOW;
		return;
	}

	uint8_t mode = (stick_squared == 0) ? MOVEMENT_STATE_IDLE : MOVEMENT_STATE_WALKING;

	/* An action owns the current state and its gait until it ends: the stick
	   only picks the state it goes back to. */
	if (!character3dMovement_isLocomotion(movement->current)) {
		movement->locomotion = mode;
		return;
	}

	character3dMovement_setMode(movement, mode);
	if (mode == MOVEMENT_STATE_IDLE) return;

	const Character3DMovementSettings *settings = movement->settings;
	uint8_t last_gait = settings->gait_count - 1;

	if (stick_squared <= PLAYER_STICK_WALK_THRESHOLD * PLAYER_STICK_WALK_THRESHOLD)
		cmd->gait = 0;
	else if (actions->sprint && !actions->aim)
		cmd->gait = last_gait;
	else
		cmd->gait = (last_gait > 1) ? 1 : last_gait;
}

/* The aim button holds the drawn weapon at the ready; the shoot button on top
   charges the shot. The release edge already travels in the actions, left for
   the shot itself. */
static void character3dControl_setAiming(Character3D *character, MovementCommand *cmd, const Character3DControls *actions)
{
	const WeaponDef *drawn = character3d_drawnWeapon(character);
	bool charges = drawn && drawn->shoot_mode == SHOOT_CHARGE;

	cmd->aiming = drawn && actions->aim
		&& character3dMovement_isLocomotion(character->movement.current);
	cmd->charging_shoot = cmd->aiming && charges && actions->shoot;
}

static void character3dControl_setStrafe(Character3D *character, MovementCommand *cmd, const Character3DControls *actions, Angle camera_angle_around)
{
	/* The air keeps the strafe: a jump out of it must not turn the body
	   toward its run, it faces the camera until it lands. */
	uint8_t current = character->movement.current;
	cmd->strafe     = actions->aim
	               && (character3dMovement_isLocomotion(current) || current == MOVEMENT_STATE_FALLING);
	cmd->strafe_yaw = Trig::wrap(camera_angle_around + 1.0_pi + CHARACTER3D_STRAFE_YAW_OFFSET);
}

/* A ladder is asked for with the stick, never a button: pushing at the rungs
   climbs and pulling away from them descends. Both are read against the
   ladder's own facing, so which one the stick means never depends on where
   the camera happens to be. Only the release is a button, and it is the one
   that jumps everywhere else. */
static void character3dControl_setClimb(Character3D *character, MovementCommand *cmd, const Character3DControls *actions)
{
	Character3DMovement *movement = &character->movement;

	cmd->climb         = 0;
	cmd->climb_release = false;

	bool climbing = movement->current == MOVEMENT_STATE_CLIMBING;
	if (!climbing && !movement->data.on_ladder) return;

	if (climbing && actions->jump) {
		cmd->climb_release = true;
		return;
	}

	/* Already on it: the stick is read raw, up climbs and down descends. A
	   ladder is the one place the camera must not get a vote: it swings
	   around the body as it rises, and a heading built off it would turn
	   the same push into a climb or a drop depending on where it ended up. */
	if (climbing) {
		if (actions->stick_y >=  STICK_DEADZONE) cmd->climb =  1.0_fp;
		if (actions->stick_y <= -STICK_DEADZONE) cmd->climb = -1.0_fp;
		return;
	}

	if (!character3dControl_stickPushed(actions->stick_x)
	 && !character3dControl_stickPushed(actions->stick_y)) return;

	/* Grabbing on is the opposite case: walking at a ladder is what asks
	   for it, so the entry is the camera-relative heading measured against
	   the ladder's facing. target_yaw is the stick already in world space
	   and a body's rotation is the negative of the heading it walks, which
	   is what brings the two into one frame to be compared. */
	Fixed alignment = Trig::cos(cmd->target_yaw + movement->data.ladder_yaw);

	if (alignment >= Trig::cos(CHARACTER3D_LADDER_ENTER_ANGLE)) cmd->climb = 1.0_fp;
}

void character3dControls_read(Character3DControls *controls, const Character3DControlBinding *binding)
{
	const Controller *controller = &controller_get()[binding->player];

	*controls = {};
	controls->jump           = button_getPressed(controller, controller->pressed,  binding->jump) != 0;
	controls->jump_held      = button_getPressed(controller, controller->held,     binding->jump) != 0;
	controls->roll           = button_getPressed(controller, controller->pressed,  binding->roll) != 0;
	controls->sprint         = button_getPressed(controller, controller->held,     binding->sprint) != 0;
	controls->aim            = button_getPressed(controller, controller->held,     binding->aim) != 0;
	controls->shoot          = button_getPressed(controller, controller->held,     binding->shoot) != 0;
	controls->shoot_released = button_getPressed(controller, controller->released, binding->shoot) != 0;
	controls->weapon_next    = button_getPressed(controller, controller->pressed,  binding->weapon_next) != 0;
	controls->weapon_prev    = button_getPressed(controller, controller->pressed,  binding->weapon_prev) != 0;
	controls->stick_x        =  controller->input.left_x;
	controls->stick_y        = -controller->input.left_y;
}

void character3dControl_update(Character3D *character, MovementCommand *cmd, const Character3DControls *actions, Angle camera_angle_around)
{
	character3dControl_setWeaponSwitch(character, actions);
	character3dControl_setRoll(character, cmd, actions);
	character3dControl_setJump(character, cmd, actions);
	character3dControl_setStrafe(character, cmd, actions, camera_angle_around);
	character3dControl_setAiming(character, cmd, actions);
	character3dControl_setLocomotionWithStick(character, cmd, actions, camera_angle_around);

	/* After the stick: the climb is read off the heading it just wrote. */
	character3dControl_setClimb(character, cmd, actions);
}
