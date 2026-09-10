#include "player/e32_player.h"
#include "control/e32_player_control.h"
#include "control/e32_character3d_control.h"
#include "control/e32_menu_control.h"
#include "control/e32_controller.h"
#include "viewport/e32_viewport.h"
#include "game/e32_game.h"


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
