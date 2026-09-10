#include <stddef.h>

#include "psyqo/kernel.hh"

#include "character3d/e32_character3d.h"
#include "physics/math/e32_math_common.h"
#include "physics/math/e32_trig.h"


/* In MovementState order: idle, walking, rolling, falling, swimming, climbing. */
static const bool movement_updates_locomotion[MOVEMENT_STATE_COUNT] = {
	true, true, false, false, false, false,
};


/* current * factor + target * (1 - factor), on angles: the difference is
   what gets scaled, so the blend never crosses the wrap. */
static inline Angle angle_blend(Angle current, Angle target, Fixed factor)
{
	return current + Angle(Fixed(target - current) * (1.0_fp - factor));
}

void character3dMovement_setMode(Character3DMovement *movement, uint8_t new_mode)
{
	if (movement->current == new_mode) return;
	movement->current = new_mode;
	if (movement_updates_locomotion[new_mode]) movement->locomotion = new_mode;
}

bool character3dMovement_isLocomotion(uint8_t mode)
{
	return movement_updates_locomotion[mode];
}

static void character3dMovement_evaluateTransitions(Character3D *character)
{
	Character3DMovement *movement = &character->movement;
	if (movement->next == MOVEMENT_STATE_NONE) return;
	character3dMovement_setMode(movement, movement->next);
	movement->next = MOVEMENT_STATE_NONE;
}


static const Character3DGaitSettings *character3dMovement_getGait(const Character3D *character)
{
	const Character3DMovementSettings *settings = character->movement.settings;
	uint8_t gait = character->movement.data.gait;
	if (gait >= settings->gait_count) gait = settings->gait_count - 1;
	return &settings->gait[gait];
}

static Fixed character3dMovement_getTargetSpeed(const Character3D *character, uint8_t state)
{
	if (state == MOVEMENT_STATE_WALKING) return character3dMovement_getGait(character)->target_speed;
	return character->movement.settings->idle_target_speed;
}

static Fixed character3dMovement_getAccelerationRate(const Character3D *character, uint8_t state)
{
	if (state == MOVEMENT_STATE_WALKING) return character3dMovement_getGait(character)->response_rate;
	return character->movement.settings->idle_response_rate;
}

static Fixed character3dMovement_getRotationAccelerationRate(const Character3D *character, uint8_t state)
{
	if (state == MOVEMENT_STATE_WALKING) return character3dMovement_getGait(character)->rotation_response_rate;
	return character->movement.settings->idle_rotation_response_rate;
}

static void character3dMovement_setHorizontalVelocity(Character3D *character, Angle yaw, Fixed target_speed, Fixed response_rate, Fixed dt)
{
	KinematicBody *body = &character->body;

	Fixed s, c;
	Trig::sincos(yaw, &s, &c);
	Fixed target_vx = target_speed *  s;
	Fixed target_vy = target_speed * -c;

	Fixed factor = expNeg(response_rate * dt);
	body->velocity.x = body->velocity.x * factor + target_vx * (1.0_fp - factor);
	body->velocity.y = body->velocity.y * factor + target_vy * (1.0_fp - factor);
}

static void character3dMovement_setRotation(Character3D *character, Fixed dt)
{
	KinematicBody *body = &character->body;
	Character3DMovementData *data = &character->movement.data;

	/* Climbing already set the facing from the ladder, and its velocity is
	   the pull onto the anchor: read as a heading it would spin the body. */
	if (character->movement.current == MOVEMENT_STATE_CLIMBING) return;

	bool moving = body->velocity.x != 0 || body->velocity.y != 0;

	if (moving) {
		Vector2 horizontal_velocity = {body->velocity.x, body->velocity.y};
		data->horizontal_speed = horizontal_velocity.magnitude();
	}

	Angle facing_yaw;
	if (data->strafe) {
		/* With the weapon up the body keeps tracking the camera even planted;
		   the plain strafe leaves a standstill facing free. */
		if (!moving && !data->aiming && !data->charging_shoot) return;
		facing_yaw = data->strafe_yaw;
	} else {
		if (!moving) return;
		facing_yaw = Trig::atan2(-body->velocity.x, -body->velocity.y);

		if (data->rotation_mode == CHARACTER3D_ROTATION_MODE_SNAP) {
			body->rotation.z = Trig::wrap(facing_yaw);
			return;
		}
	}

	const Angle current_yaw = body->rotation.z;
	const Angle target_yaw  = Trig::wrapRelative(facing_yaw, current_yaw);

	if ((target_yaw - current_yaw).abs() <= CHARACTER3D_ROTATION_SNAP_THRESHOLD) {
		body->rotation.z = target_yaw;
		return;
	}

	uint8_t state = character->movement.current;
	if (state == MOVEMENT_STATE_ROLLING || state == MOVEMENT_STATE_FALLING)
		state = character->movement.locomotion;
	Fixed response_rate = character3dMovement_getRotationAccelerationRate(character, state);
	Fixed factor = expNeg(response_rate * dt);
	body->rotation.z = Trig::wrap(angle_blend(current_yaw, target_yaw, factor));
}

static void character3dMovement_updateBody(Character3D *character, Fixed dt)
{
	KinematicBody *body = &character->body;
	Character3DMovementData *data = &character->movement.data;

	data->previous_yaw = body->rotation.z;

	/* A ladder standing in water is still a ladder: the climb owns the
	   vertical, so the buoyancy must not bob the body off its rungs. */
	if (data->in_water && character->movement.current != MOVEMENT_STATE_CLIMBING) {
		const Character3DMovementSettings *settings = character->movement.settings;

		/* Stroking raises the equilibrium so the swim pose meets the surface. */
		Fixed stroke = (settings->swim_slow_speed > 0)
			? data->horizontal_speed / settings->swim_slow_speed : Fixed(0);
		if (stroke > 1.0_fp) stroke = 1.0_fp;

		Fixed equilibrium = settings->water_equilibrium_idle
			+ (settings->water_equilibrium_swim - settings->water_equilibrium_idle) * stroke;

		/* Gravity scaled by how far the capsule sits from the equilibrium
		   depth: deeper than it, the push turns upward. The drag grows with
		   the submerged body, so a dive keeps its momentum through the
		   surface and brakes progressively on the way down. */
		Fixed buoyant = CHARACTER3D_GRAVITY * (1.0_fp - data->submerged_fraction / equilibrium);
		body->velocity.z += buoyant * dt;
		body->velocity.z /= (1.0_fp + CHARACTER3D_WATER_DRAG * data->submerged_fraction * dt);

		/* Progressive sink limit: none at the splash so a dive keeps its
		   momentum, full past the saturation fraction; the pool is barely
		   deeper than the capsule, so the brake cannot wait for the drag. */
		Fixed t = data->submerged_fraction / CHARACTER3D_WATER_SINK_LIMIT_FULL;
		if (t > 1.0_fp) t = 1.0_fp;
		Fixed limit = CHARACTER3D_FALL_MAX_SPEED + (CHARACTER3D_WATER_SINK_MAX_SPEED - CHARACTER3D_FALL_MAX_SPEED) * t;
		if (body->velocity.z < limit) body->velocity.z = limit;
	}
	else if (body->acceleration.z != 0)
		body->velocity.z += body->acceleration.z * dt;

	if (body->velocity.x.abs() < LOCOMOTION_MIN_SPEED && body->velocity.y.abs() < LOCOMOTION_MIN_SPEED && body->velocity.z == 0) {
		body->velocity.x = 0;
		body->velocity.y = 0;
		data->horizontal_speed = 0;
	}

	if (body->velocity.x != 0 || body->velocity.y != 0 || body->velocity.z != 0)
		body->position.addScaledVector(body->velocity, dt);

	character3dMovement_setRotation(character, dt);
}

/* The crouch that starts a jump runs here, on the ground, over walk or idle:
   the stick still drives the body and holding the button only brakes it. The
   impulse and the switch to the air come together when the crouch ends, so a
   jump is never a state of its own — the air is always a fall. */
static void character3dMovement_setChargingJump(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	KinematicBody *body = &character->body;
	Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *settings = character->movement.settings;

	if (cmd->jump_triggered) {
		data->jump_initial_velocity = body->velocity;
		data->jump_timer    = 0;
		data->jump_force    = 0;
		cmd->jump_triggered = false;

		/* Snap: no crouch at all. The floor is left on this very frame with
		   the minimum launch, and the rest of the height is added in the air
		   for as long as the button holds. */
		if (settings->jump_mode == JUMP_SNAP) {
			body->velocity = data->jump_initial_velocity;
			body->velocity *= CHARACTER3D_JUMP_LAUNCH_VELOCITY_SCALE;
			body->velocity.z = settings->jump_base_speed;

			character->movement.next = MOVEMENT_STATE_FALLING;
			return;
		}
	}
	else if (data->jump_timer == 0) return;   /* nothing being charged */

	data->jump_timer += dt;
	if (cmd->jump_held) {
		data->jump_force += dt;
		body->velocity *= CHARACTER3D_JUMP_HOLD_VELOCITY_SCALE;
	}

	if (data->jump_timer < settings->jump_timer_max) return;

	/* Leaving the floor: the launch keeps a share of the run it came from. */
	body->velocity = data->jump_initial_velocity;
	body->velocity *= CHARACTER3D_JUMP_LAUNCH_VELOCITY_SCALE;
	body->velocity.z = data->jump_force * settings->jump_force_multiplier;
	if (body->velocity.z < settings->jump_base_speed)
		body->velocity.z = settings->jump_base_speed;

	data->jump_force = 0;
	data->jump_timer = 0;
	character->movement.next = MOVEMENT_STATE_FALLING;
}

/* Coyote time for the snap: the body falls the way it always did, but for a
   moment after the edge the button still launches. The charge needs none of
   this — its crouch holds it in locomotion until it lets go. */
bool character3dMovement_isCoyoteOpen(const Character3D *character)
{
	return !character->movement.data.is_grounded
	    && character->movement.data.coyote_timer < character->movement.settings->jump_coyote_time;
}

static void character3dMovement_setSnappingJump(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	KinematicBody *body = &character->body;
	Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *settings = character->movement.settings;

	data->coyote_timer += dt;

	if (settings->jump_mode != JUMP_SNAP
	 || !cmd->jump_triggered
	 || data->coyote_timer >= settings->jump_coyote_time) return;

	/* The run it walked off the ledge with is what it launches on: there was
	   no press on the floor to have saved one. */
	data->jump_initial_velocity = body->velocity;
	body->velocity *= CHARACTER3D_JUMP_LAUNCH_VELOCITY_SCALE;
	body->velocity.z = settings->jump_base_speed;

	/* One launch per edge: the window closes on the jump it granted. */
	data->coyote_timer  = settings->jump_coyote_time;
	cmd->jump_triggered = false;
}

/* A crouch already under way. The edge can run out mid-crouch and the jump is
   not lost for it: the body stays in locomotion and coasts, keeping the speed
   it had and taking no gravity, until the crouch finishes and launches it as
   if it had never left the ledge. */
bool character3dMovement_isChargingJump(const Character3D *character)
{
	return character->movement.settings->jump_mode == JUMP_CHARGE
	    && character->movement.data.jump_timer > 0;
}

static void character3dMovement_setLocomotion(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	uint8_t state = character->movement.current;
	character3dMovement_setHorizontalVelocity(character, cmd->target_yaw, character3dMovement_getTargetSpeed(character, state) * cmd->speed_scale, character3dMovement_getAccelerationRate(character, state), dt);
	character3dMovement_setChargingJump(character, cmd, dt);
}

/* Gravity and the terminal speed that goes with it, for anything with no floor
   under it. The collision pass clears the acceleration again on landing. */
static void character3dMovement_setGravity(KinematicBody *body, Fixed gravity_scale)
{
	body->acceleration.z = CHARACTER3D_GRAVITY * gravity_scale;
	if (body->velocity.z < CHARACTER3D_FALL_MAX_SPEED)
		body->velocity.z = CHARACTER3D_FALL_MAX_SPEED;
}

static void character3dMovement_setRolling(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *settings = character->movement.settings;

	/* The trigger is the entry mark and is consumed here: the stick yaw is
	   taken once and held until grip, and the timer starts from zero however
	   the previous roll ended: a ledge can end one before its own last phase. */
	if (cmd->roll_triggered) {
		data->roll_yaw      = cmd->target_yaw;
		data->roll_timer    = 0;
		cmd->roll_triggered = false;
	}

	/* Rolling off a ledge drops: the state is what holds to the end of the
	   clip, not the ground. */
	if (!data->is_grounded) character3dMovement_setGravity(&character->body, 1.0_fp);

	/* Out to idle, not to the locomotion state held from before the roll: the
	   control runs first next frame and the stick raises it to walk if it is
	   asking for one, the same way the floor probe turns it into a fall. */
	if (data->roll_timer >= settings->roll_timer_max) {
		character->movement.next = MOVEMENT_STATE_IDLE;
		data->roll_timer = 0;
		return;
	}

	/* Three phases off the one timer: the launch drives the stick yaw at the
	   roll's own speed, the spin holds whatever speed it reached along the
	   body's own facing, and the grip hands the steering back to the stick. */
	Angle yaw           = data->roll_yaw;
	Fixed target_speed  = settings->roll_target_speed;
	Fixed response_rate = settings->roll_launch_response_rate;

	if (data->roll_timer >= settings->roll_grip_time) {
		yaw           = cmd->target_yaw;
		target_speed  = data->horizontal_speed;
		response_rate = settings->roll_grip_response_rate;
	}
	else if (data->roll_timer >= settings->roll_ground_time) {
		yaw           = -character->body.rotation.z;
		target_speed  = data->horizontal_speed;
		response_rate = settings->roll_spin_response_rate;
	}

	character3dMovement_setHorizontalVelocity(character, yaw, target_speed, response_rate, dt);
	data->roll_timer += dt;
}

static void character3dMovement_setFalling(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	KinematicBody *body = &character->body;
	Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *settings = character->movement.settings;

	data->is_grounded = 0;

	character3dMovement_setSnappingJump(character, cmd, dt);

	/* Air control decides both halves at once: how far the heading can be
	   pulled toward the stick, and how much the speed can move toward what
	   the same stick would give on the ground. At zero neither moves and the
	   jump keeps the run that launched it. */
	Fixed target_speed = data->horizontal_speed;
	if (settings->air_control > 0) {
		Fixed ground = character->movement.locomotion == MOVEMENT_STATE_WALKING
		             ? character3dMovement_getTargetSpeed(character, MOVEMENT_STATE_WALKING) * cmd->speed_scale
		             : Fixed(0);
		target_speed += (ground - target_speed) * settings->air_control;
	}

	character3dMovement_setHorizontalVelocity(character, cmd->target_yaw, target_speed,
	                                        settings->jump_response_rate * settings->air_control, dt);

	/* Snap: holding the button makes the rise cost less gravity, so how long
	   it is held is how high it goes. Only on the way up: past the top the
	   fall is the fall, and nobody floats down. */
	Fixed gravity_scale = 1.0_fp;
	if (settings->jump_mode == JUMP_SNAP && cmd->jump_held && body->velocity.z > 0)
		gravity_scale = settings->jump_hold_gravity_scale;

	character3dMovement_setGravity(body, gravity_scale);
}

/* The vertical is not touched here: the fake buoyancy in updateBody floats,
   bobs and damps the capsule on its own. The stick only swims horizontally. */
static void character3dMovement_setSwimming(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	const Character3DMovementSettings *settings = character->movement.settings;
	KinematicBody *body = &character->body;

	/* Tired locks the fast stroke away, same as the top gait on land. */
	uint8_t swim_gait = cmd->swim_gait;
	if (cmd->speed_scale < 1.0_fp && swim_gait == CHARACTER3D_SWIM_GAIT_FAST)
		swim_gait = CHARACTER3D_SWIM_GAIT_SLOW;

	Fixed target = 0;
	if (swim_gait == CHARACTER3D_SWIM_GAIT_SLOW) target = settings->swim_slow_speed;
	if (swim_gait == CHARACTER3D_SWIM_GAIT_FAST) target = settings->swim_fast_speed;

	character3dMovement_setHorizontalVelocity(character, cmd->target_yaw, target * cmd->speed_scale, settings->swim_response_rate, dt);
	body->acceleration.z = 0;
}

/* On a ladder the state owns the body outright: no gravity, and the stick
   drives the vertical instead of a run. The horizontal is spent entirely on
   pulling the body onto the anchor the probe wrote, so an approach from the
   side slides into the ladder's centre line rather than climbing thin air. */
static void character3dMovement_setClimbing(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	KinematicBody *body = &character->body;
	Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *settings = character->movement.settings;

	body->acceleration.z = 0;

	Fixed target = cmd->climb * settings->climb_speed * cmd->speed_scale;
	Fixed factor = expNeg(settings->climb_response_rate * dt);
	body->velocity.z = body->velocity.z * factor + target * (1.0_fp - factor);

	/* The anchor is a position, not a target speed: the pull is written as
	   the velocity that closes the gap left this frame, so it dies out on
	   arrival instead of circling it. */
	Fixed pull = (1.0_fp - expNeg(CHARACTER3D_LADDER_ANCHOR_RATE * dt)) / dt;
	body->velocity.x = (data->ladder_anchor_x - body->position.x) * pull;
	body->velocity.y = (data->ladder_anchor_y - body->position.y) * pull;

	data->horizontal_speed = 0;
	data->is_grounded = 0;

	/* The rungs are what the body faces, whatever it faced walking in. */
	body->rotation.z   = data->ladder_yaw;
	data->rotation_mode = CHARACTER3D_ROTATION_MODE_SNAP;
}

/* Ladder entry and exit, from the climbable probe of the physics pass.
   Entering asks the stick to be pushing at the rungs, so walking past a
   ladder never snatches the body onto it.

   Every exit but the release is a loss of contact: the volume is what says
   where climbing is possible, and running off its top while still climbing
   is the body cresting the ladder. That exit gets a push toward the rungs so
   it lands on the ledge instead of peeling back off the face. */
static void character3dMovement_evaluateLadder(Character3D *character, MovementCommand *cmd)
{
	Character3DMovement *movement = &character->movement;
	Character3DMovementData *data = &movement->data;
	KinematicBody *body = &character->body;

	if (movement->current != MOVEMENT_STATE_CLIMBING) {
		if (!data->on_ladder || cmd->climb <= 0) return;
		if (movement->next != MOVEMENT_STATE_NONE) return;

		/* Swimming counts: a ladder in a pool is how the body gets out. */
		if (!character3dMovement_isLocomotion(movement->current)
		 && movement->current != MOVEMENT_STATE_FALLING
		 && movement->current != MOVEMENT_STATE_SWIMMING) return;

		/* Not from the crest: standing on top of the ladder leaves the feet
		   right at the volume's ceiling, and without this margin the climb
		   that just ended there grabs straight back on. */
		if (body->position.z > data->ladder_top - CHARACTER3D_LADDER_ENTER_MARGIN) return;

		/* Grabbing on kills whatever vertical the body arrived with: a fall
		   caught by a ladder would otherwise ride its own speed down past
		   the rungs while the climb slowly talks it out of it. */
		body->velocity.z = 0;
		movement->next   = MOVEMENT_STATE_CLIMBING;
		return;
	}

	if (cmd->climb_release) {
		movement->next = MOVEMENT_STATE_FALLING;
		return;
	}

	if (!data->on_ladder) {
		/* Off the top with the stick still climbing: step onto the landing.
		   The push is horizontal and the fall carries it from there, so the
		   body arrives with the landing clip it already has.

		   The heading is commandeered along with the velocity. The fall
		   steers toward whatever the stick last asked for, and on a ladder
		   the stick means up, not a direction — left to itself the body
		   would crest the top and immediately walk off wherever the camera
		   happened to be pointing, which mostly means back into the rungs
		   it just left. It holds this heading until the stick is moved. */
		if (cmd->climb > 0) {
			Fixed s, c;
			Trig::sincos(-data->ladder_yaw, &s, &c);
			body->velocity.x = CHARACTER3D_LADDER_EXIT_SPEED *  s;
			body->velocity.y = CHARACTER3D_LADDER_EXIT_SPEED * -c;
			body->velocity.z = 0;
			data->horizontal_speed = CHARACTER3D_LADDER_EXIT_SPEED;
			cmd->target_yaw        = -data->ladder_yaw;
		}
		movement->next = MOVEMENT_STATE_FALLING;
		return;
	}

	/* Back at the foot with the stick pushing down: the ground itself is
	   what says so, not the volume: its floor sits below the stair's, so
	   that someone standing at the bottom is inside it to begin with. */
	if (cmd->climb < 0 && data->floor_distance >= 0
	 && data->floor_distance <= CHARACTER3D_LADDER_GROUND_REACH)
		movement->next = movement->locomotion;
}

/* Swim entry and exit, from the water probe of the physics pass. Two separate
   questions, never one: the volume says whether there is water, and the floor
   says whether it can be walked. A body inside the water with nothing under
   its feet swims; one standing on the bottom wades, however deep the water
   reaches on it.

   How far the body sinks is not asked here on purpose. It moves with the
   buoyancy, which moves with the speed, and a state read off it changes when
   the stick does: standing still in a shallow pool would drop the swim, and
   pushing the stick would take it back, over and over. */
static void character3dMovement_evaluateWater(Character3D *character)
{
	Character3DMovement *movement = &character->movement;
	const Character3DMovementData *data = &movement->data;
	uint8_t current = movement->current;

	if (current != MOVEMENT_STATE_SWIMMING) {
		bool can_enter = character3dMovement_isLocomotion(current)
		              || current == MOVEMENT_STATE_FALLING
		              || current == MOVEMENT_STATE_ROLLING;

		if (can_enter && data->in_water && !data->is_grounded)
			movement->next = MOVEMENT_STATE_SWIMMING;
		return;
	}

	/* Out of the water, or standing on the bottom of it. Airborne, the floor
	   probe of the physics pass turns it into falling on its own. */
	if (!data->in_water || data->is_grounded)
		movement->next = movement->locomotion;
}

/* In MovementState order: idle, walking, rolling, falling, swimming, climbing. */
static void (*character3dMovement_handler[MOVEMENT_STATE_COUNT])(Character3D *, MovementCommand *, Fixed) = {
	character3dMovement_setLocomotion,
	character3dMovement_setLocomotion,
	character3dMovement_setRolling,
	character3dMovement_setFalling,
	character3dMovement_setSwimming,
	character3dMovement_setClimbing,
};

static_assert(sizeof(character3dMovement_handler) / sizeof(character3dMovement_handler[0]) == MOVEMENT_STATE_COUNT, "character3dMovement_handler must have one entry per character state");

void character3d_updateMovement(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	psyqo::Kernel::assert(character != NULL && cmd != NULL, "character: no body to move");
	psyqo::Kernel::assert(character->movement.current < MOVEMENT_STATE_COUNT, "character: bad movement state");

	character->movement.data.rotation_mode = CHARACTER3D_ROTATION_MODE_LERP;
	character->movement.data.strafe        = cmd->strafe;
	character->movement.data.strafe_locked = cmd->strafe_locked;
	character->movement.data.strafe_yaw    = cmd->strafe_yaw;
	character->movement.data.aiming         = cmd->aiming;
	character->movement.data.charging_shoot = cmd->charging_shoot;
	character->movement.data.shooting       = cmd->shooting;

	/* A scaled-down command locks the top gait away: the character stays on
	   the previous one until the scale is back at full. */
	uint8_t gait = cmd->gait;
	if (cmd->speed_scale < 1.0_fp && gait == character->movement.settings->gait_count - 1)
		gait = character->movement.settings->gait_count - 2;
	character->movement.data.gait = gait;
	character->movement.next = MOVEMENT_STATE_NONE;

	character3dMovement_handler[character->movement.current](character, cmd, dt);
	character3dMovement_updateBody(character, dt);
	character3dMovement_evaluateWater(character);
	character3dMovement_evaluateLadder(character, cmd);
	character3dMovement_evaluateTransitions(character);
}
