/*
	Weapon slots, equip and bone posing. Weapon meshes live inside the
	character model, skinned to dedicated root-level bones; posing those bones
	parks the weapon on its holster reference bone or on the hand.
*/
#include <assert.h>
#include <string.h>

#include "animation/e32_armature.h"

#include "physics/math/e32_quaternion.h"

#include "entity/e32_entity3d.h"
#include "character3d/e32_character3d.h"


void character3d_equipWeapon(Character3D *character, uint8_t slot_id, const WeaponDef *weapon)
{
	assert(slot_id < WEAPON_SLOT_COUNT);

	Character3DWeapons *weapons = &character->weapons;
	Armature *skeleton = &character->animation.main;

	uint8_t part = 0;
	for (int i = 0; i < weapons->def->mesh_count; i++) {
		if (strcmp(weapon->mesh, weapons->def->mesh[i]) == 0) {
			part = 1 + i;
			break;
		}
	}
	assert(part);   /* weapon mesh must exist in the character model */

	weapons->slot[slot_id] = (WeaponSlot){
		.weapon       = weapon,
		.rounds       = weapon->magazine_size,
		.integrity    = weapon->max_integrity,
		.part         = part,
		.bone         = (int16_t)armature_findBone(skeleton, (char *)weapon->bone),
		.holster_bone = (int16_t)armature_findBone(skeleton, (char *)weapon->holster_bone),
		.hand_bone    = (int16_t)armature_findBone(skeleton, (char *)weapon->hand_bone),
	};

	character->entity->mesh->visible |= 1u << part;
}

void character3d_unequipWeapon(Character3D *character, uint8_t slot_id)
{
	assert(slot_id < WEAPON_SLOT_COUNT);

	WeaponSlot *slot = &character->weapons.slot[slot_id];
	if (!slot->weapon) return;

	character->entity->mesh->visible &= ~(1u << slot->part);
	if (character->weapons.drawn == slot_id)
		character->weapons.drawn = CHARACTER3D_WEAPON_DRAWN_NONE;

	*slot = (WeaponSlot){0};
}

const WeaponDef *character3d_drawnWeapon(const Character3D *character)
{
	const Character3DWeapons *weapons = &character->weapons;
	if (weapons->drawn == CHARACTER3D_WEAPON_DRAWN_NONE) return NULL;
	return weapons->slot[weapons->drawn].weapon;
}

/* The ring is: unarmed, then every occupied slot in order. Empty slots are
   stepped over, so with one weapon carried both directions just toggle it
   in and out of the hand. */
void character3d_cycleWeapon(Character3D *character, int8_t dir)
{
	Character3DWeapons *weapons = &character->weapons;

	int8_t pos = (weapons->drawn == CHARACTER3D_WEAPON_DRAWN_NONE) ? -1 : (int8_t)weapons->drawn;

	for (int i = 0; i < WEAPON_SLOT_COUNT + 1; i++) {
		pos += dir;
		if (pos > WEAPON_SLOT_COUNT - 1) pos = -1;
		if (pos < -1)                    pos = WEAPON_SLOT_COUNT - 1;
		if (pos == -1) break;
		if (weapons->slot[pos].weapon) break;
	}

	weapons->drawn = (pos < 0) ? CHARACTER3D_WEAPON_DRAWN_NONE : (uint8_t)pos;
}

void character3dWeapon_setBones(Character3D *character)
{
	Character3DWeapons *weapons = &character->weapons;
	Armature *skeleton = &character->animation.main;

	for (int s = 0; s < WEAPON_SLOT_COUNT; s++) {
		WeaponSlot *slot = &weapons->slot[s];
		if (!slot->weapon || slot->bone < 0) continue;

		bool drawn = (weapons->drawn == s);
		int16_t reference     = drawn ? slot->hand_bone : slot->holster_bone;
		const Vector3 *offset_pos = drawn ? &slot->weapon->holding_position : &slot->weapon->holster_position;
		const Quaternion *offset_rot = drawn ? &slot->weapon->holding_rotation : &slot->weapon->holster_rotation;
		if (reference < 0) continue;

		Vector3 ref_pos;
		Quaternion ref_rot;
		character3d_getBonePose(skeleton, reference, &ref_pos, &ref_rot);

		Vector3 step = quaternion_rotateVector(&ref_rot, offset_pos);

		Bone *bone = &skeleton->bones[slot->bone];
		bone->position = (Vector3){
			ref_pos.x + step.x,
			ref_pos.y + step.y,
			ref_pos.z + step.z,
		};
		bone->rotation = quaternion_product(&ref_rot, offset_rot);
		bone->hasChanged = 1;
	}
}
