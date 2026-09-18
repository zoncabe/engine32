#include "player/e32_player.h"
#include "control/e32_player_control.h"
#include "control/e32_character3d_control.h"
#include "control/e32_menu_control.h"
#include "control/e32_controller.h"
#include "viewport/e32_viewport.h"
#include "scene3d/e32_scene3d.h"
#include "prefab/e32_prefab3d.h"
#include "game/e32_game.h"


void controls_bind(const ControlsDef *controls, const Scene3DDef *scene)
{
	if (controls == NULL || scene == NULL) return;

	/* The buttons find the camera, not the other way around: they answer for
	   the camera they name, and this scene's takes them if it is that one. */
	if (controls->camera)
		Viewport::get().camera.binding =
			(controls->camera->camera == scene->camera) ? controls->camera : NULL;

	if (controls->character3d == NULL || controls->character3d->character == NULL) return;

	/* Characters come out of the load in placement order, so counting the
	   character rows of the declaration finds the one built from the prefab
	   the binding drives. */
	uint8_t index = 0;
	for (int i = 0; i < scene->prefab_count; i++) {

		const Prefab3D *prefab = scene->prefab[i].prefab;
		if (prefab == NULL || prefab->type != PREFAB3D_CHARACTER) continue;

		if (prefab == controls->character3d->character) {
			player_setCharacter3D(scene3d_getCharacter3D(index), controls->character3d);
			return;
		}
		index++;
	}
}


void player_setCharacter3DControl(PlayerID id, Viewport *viewport)
{
	Player *player = &player_get()[id];
	if (player->character == NULL || player->control == NULL) return;

	/* Read where it is used: what the controller is doing this frame is worth
	   nothing on the next one. */
	Character3DControls controls;
	character3dControls_read(&controls, player->control);

	character3dControl_update(
		player->character,
		&player->cmd,
		&controls,
		viewport->camera.getAngleAround(&player->character->entity->transform.position)
	);
}
