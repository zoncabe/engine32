/*
	Everything the body runs on. Physics units are metres, so the capsule that
	stands 180 units tall on screen is 1.8 here.

	The engine brings no movement of its own: a character is these settings and
	nothing else. What this def leaves out, the animation graph, the sounds,
	the weapons, the aim bend and the spring bones, is skipped entirely.
*/
#include "prefab/e32_prefab3d.h"


static const Character3DColliderSettings collider = {

	.radius = 0.35_fp,
	.height = 1.80_fp,
};

/* Gaits run slowest to fastest. Three of them: the stick alone picks the
   middle one and sprint reaches the top, which is the one that tires the
   body out. */
static const Character3DGaitSettings gait[] = {

	{ .target_speed = 4.0_fp, .response_rate =  8.0_fp, .rotation_response_rate = 12.0_fp },
	{ .target_speed = 6.5_fp, .response_rate =  9.0_fp, .rotation_response_rate = 10.0_fp },
	{ .target_speed = 8.5_fp, .response_rate = 10.0_fp, .rotation_response_rate =  8.0_fp },
};

static const Character3DMovementSettings movement = {

	.idle_target_speed           =  0.0_fp,
	.idle_response_rate          = 12.0_fp,
	.idle_rotation_response_rate = 10.0_fp,

	.gait       = gait,
	.gait_count = sizeof(gait) / sizeof(gait[0]),

	/* Snap: the body leaves the floor on the press and, while cross stays
	   down, the rise pays less gravity. A tap reaches a metre, holding all
	   the way up reaches two and a half, and the rise ends itself either
	   way. */
	.jump_mode               = JUMP_SNAP,
	.jump_response_rate      = 10.0_fp,
	.jump_base_speed         = 9.0_fp,

	/* Only read under charge, the other mode: how long the crouch lasts and
	   what it turns into. */
	.jump_force_multiplier = 30.0_fp,
	.jump_timer_max        = 0.25_fp,

	.jump_hold_gravity_scale = 0.7_fp,

	/* Coyote time: walking off a ledge the jump still answers for this long,
	   so arriving at the edge and pressing does not cost the jump. */
	.jump_coyote_time        = 0.15_fp,

	/* Most of the steering of the ground: the jump keeps the run that
	   launched it, but the stick still counts for something. */
	.air_control = 0.8_fp,

	.swim_slow_speed    = 2.0_fp,
	.swim_fast_speed    = 3.0_fp,
	.swim_response_rate = 4.0_fp,

	/* Vertical speed on the ladder. Left at zero the body grabs the rungs
	   and stays there, which is what an unfilled climb looks like. */
	.climb_speed         = 2.5_fp,
	.climb_response_rate = 8.0_fp,

	/* How deep the capsule floats: idle keeps the head out, swimming rides
	   a little lower. */
	.water_equilibrium_idle = 0.70_fp,
	.water_equilibrium_swim = 0.70_fp,
};

/* Stamina is read on every update, so it needs settings even where nothing
   spends it. A drain of zero never empties: the body sprints forever, which
   is what an example with no gauge on screen wants. The stats module is
   not ported yet, so its settings still read as floats. */
static const Character3DStatsSettings stats = {

	.stamina_drain_rate = 0.0f,
	.stamina_regen_rate = 0.35f,
	.tired_speed_scale  = 0.60f,
};


static const Character3DDef character3d_def = {

	.movement_settings = &movement,
	.collider_settings = &collider,
	.stats_settings    = &stats,
};

/* No collider of its own: the character builds its capsule from the radius
   and height above, and the scene gives it a body from that. */
extern const Prefab3D character = {

	.type      = PREFAB3D_CHARACTER,
	.model     = "rom:/models/capsule.model",
	.character = &character3d_def,
};
