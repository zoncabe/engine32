#include "character3d/e32_character3d.h"
#include "physics/math/e32_math_functions.h"


/* Running at the top gait or swimming fast drains stamina, anything else
   recovers it. Hitting zero flags the body tired, which caps locomotion
   and swim speed through the command until stamina is back at full. */
void character3dStats_update(Character3D *character, MovementCommand *cmd, float dt)
{
	const Character3DMovement *movement = &character->movement;
	Character3DStats *stats = &character->stats;
	const Character3DStatsSettings *settings = stats->settings;

	bool running = (movement->current == MOVEMENT_STATE_WALKING
	                && movement->data.gait == movement->settings->gait_count - 1)
	            || (movement->current == MOVEMENT_STATE_SWIMMING
	                && cmd->swim_gait == CHARACTER3D_SWIM_GAIT_FAST);

	/* Transitions run first, on the value the previous frame rendered: the
	   frame that lands on zero stays at zero on screen, and regen can only
	   move it from the next update on. */
	if (stats->stamina <= 0.0f) stats->tired = true;
	if (stats->stamina >= 1.0f) stats->tired = false;

	float rate = running && !stats->tired ? -settings->stamina_drain_rate : settings->stamina_regen_rate;
	stats->stamina = clampf(stats->stamina + rate * dt, 0.0f, 1.0f);

	cmd->speed_scale = stats->tired ? settings->tired_speed_scale : 1.0f;
}
