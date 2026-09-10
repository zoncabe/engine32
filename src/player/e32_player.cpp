#include <stddef.h>

#include "viewport/e32_viewport.h"
#include "entity/e32_entity3d.h"
#include "control/e32_character3d_control.h"
#include "character3d/e32_character3d.h"
#include "character3d/e32_character3d_movement.h"
#include "character3d/e32_character3d_animation.h"
#include "player/e32_player.h"
#include "physics/math/e32_math_functions.h"
#include "scene3d/e32_scene3d.h"
#include "time/e32_time.h"


static Player player[PLAYER_COUNT];

Player *player_get(void) { return player; }

void player_init(void)
{
	for (int i = 0; i < PLAYER_COUNT; i++)
		player[i] = {};
}

/* The seat comes from the binding: the buttons already name whose they are. */
void player_setCharacter3D(Character3D *character, const Character3DControlBinding *control)
{
	if (control == NULL) return;

	Player *seat = &player[control->player];

	seat->control   = control;
	seat->character = character;
	seat->entity = character ? character->entity : NULL;
	/* No ucode matrix state to prime: every draw loads its own matrices. */
}

/* Cycles the player through the scene's characters, in either direction. */
void player_switchCharacter3D(PlayerID id, int8_t direction)
{
	Scene3D *scene = scene3d_get();
	if (scene->character3d_count < 2) return;

	Player *seat = &player[id];

	uint8_t current = 0;
	for (uint8_t i = 0; i < scene->character3d_count; i++)
		if (scene->character[i] == seat->character) { current = i; break; }

	uint8_t next = (uint8_t)((current + scene->character3d_count + direction) % scene->character3d_count);
	Character3D *character = scene->character[next];

	/* Same buttons on the new body: switching bodies is not re-binding. */
	player_setCharacter3D(character, seat->control);

	/* Fresh command, facing where this body already faces: anything held over
	   from the previous character would spin the new one on the spot. */
	seat->cmd = {};
	seat->cmd.target_yaw = character->body.rotation.z;
}


void player_update(void)
{
	const Fixed dt = Time::get().delta;
	for (int i = 0; i < PLAYER_COUNT; i++) {
		/* Seats nobody took: a player without a body has nothing to run. */
		if (player[i].character == NULL) continue;

		character3dStats_update(player[i].character, &player[i].cmd, dt);
		character3d_updateMovement(player[i].character, &player[i].cmd, dt);
		character3d_setAnimation(player[i].character);
		character3dSound_update(player[i].character);
	}

	/* Scene3D characters nobody drives run on an empty command, so they idle
	   instead of freezing mid pose when the player switches away. */
	static MovementCommand idle_cmd;
	Scene3D *scene = scene3d_get();
	for (int i = 0; i < scene->character3d_count; i++) {
		Character3D *character = scene->character[i];

		bool driven = false;
		for (int p = 0; p < PLAYER_COUNT; p++)
			if (player[p].character == character) driven = true;
		if (driven) continue;

		/* Same pipeline as a driven body, on a controller nobody holds: the
		   released stick idles it through the control's own rule (treading
		   water if it was swimming), and idling never drains, so a body
		   left behind rests and refills on its own. */
		static const Character3DControls no_controls = {};

		idle_cmd.target_yaw = character->body.rotation.z;
		character3dControl_update(character, &idle_cmd, &no_controls, Angle());
		character3dStats_update(character, &idle_cmd, dt);
		character3d_updateMovement(character, &idle_cmd, dt);
		character3d_setAnimation(character);
		character3dSound_update(character);
	}
}

void player_setMatrix(uint8_t fb_index)
{
	for (int i = 0; i < PLAYER_COUNT; i++)
		if (player[i].entity)
			player[i].entity->mesh->setMatrix(&player[i].entity->transform, fb_index);
}
