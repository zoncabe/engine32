#include <stddef.h>

#include "psyqo/alloc.h"
#include "psyqo/kernel.hh"

#include "animation/e32_model.h"
#include "animation/e32_armature.h"

#include "physics/math/e32_quaternion.h"
#include "entity/e32_entity3d.h"
#include "character3d/e32_character3d.h"
#include "character3d/e32_character3d_animation.h"


/* SkeletonModifierFn: the weapon posing; context is the Character3D. */
static void character3d_weaponModifier(Armature *skeleton, void *context)
{
	(void)skeleton;
	character3dWeapon_setBones((Character3D *)context);
}

Character3D *character3d_create(const Character3DDef *def, Entity3D *entity)
{
	/* The spring bone states ride in the same allocation; their only
	   references are the modifier contexts, freed with the character. */
	uint8_t spring_bones = 0;
	if (def->spring_bones)
		for (const SpringBonesDef *set = def->spring_bones; set->count; set++)
			spring_bones += set->count;

	Character3D *character = (Character3D *)psyqo_malloc(sizeof(Character3D) + spring_bones * sizeof(SpringBone));
	psyqo::Kernel::assert(character != NULL, "character: out of memory");

	*character = {};
	character->entity = entity;

	character->stats.settings = def->stats_settings;
	character->stats.stamina  = 1.0f;

	/* The body and the render transform are both in meters. */
	character->body.position = entity->transform.position;
	character->body.rotation = entity->transform.rotation;

	character->movement.settings         = def->movement_settings;
	character->movement.data.is_grounded = true;
	character->movement.current          = MOVEMENT_STATE_IDLE;

	character->animation.def = def->animation_def;

	character->weapons.def   = def->weapons_def;
	character->weapons.drawn = CHARACTER3D_WEAPON_DRAWN_NONE;

	/* No previous frame to compare against yet: a cycle of -1 crosses
	   nothing, and the body starts standing on the floor. */
	character->sound.def               = def->sound_def;
	character->sound.previous_cycle    = -1.0f;
	character->sound.previous_grounded = true;

	character->collider.init(def->collider_settings->radius,
		(def->collider_settings->height - def->collider_settings->radius * 2) * 0.5_fp);
	character->collider.setVertical(character->body.position);

	/* A def without animations (a vehicle) skips the whole graph: the mesh
	   keeps a NULL skeleton and draws through the model object path. */
	if (def->animation_def) {
		character3dAnimation_initGraph(character, def->animation_def);
		entity->mesh->skeleton = &character->animation.main;
	}

	/* Aim before the weapons: the bow has to follow a spine already bent. */
	if (def->aiming_settings) {
		character3dAim_init(character, def->aiming_settings);
		skeletonModifiers_add(&character->skeleton_modifiers, character3dAim_apply, character);
	}

	skeletonModifiers_add(&character->skeleton_modifiers, character3d_weaponModifier, character);

	if (spring_bones > 0) {
		SpringBone *spring_bone = (SpringBone *)(character + 1);
		uint8_t n = 0;

		/* Chains rely on this order: the resolved joints run root to tip,
		   so each modifier runs after the one it hangs from. */
		for (const SpringBonesDef *set = def->spring_bones; set->count; set++) {
			int16_t joint[16];
			uint8_t count = springBones_resolveChain(&character->animation.main, set, joint, 16);
			if (count > set->count) count = set->count;

			for (uint8_t i = 0; i < count; i++) {
				if (!springBone_init(&spring_bone[n], &character->animation.main, joint[i],
				                     i, set, &entity->transform))
					continue;

				skeletonModifiers_add(&character->skeleton_modifiers, springBone_apply, &spring_bone[n]);
				n++;
			}
		}
	}

	/* Part 0 = body, parts 1..N = one per weapon object, def order.
	   Only the body starts visible; equipping turns weapon bits on.
	   No weapons: the whole model is the single skinned part. No skeleton
	   either: per-object blocks, exactly what a prop gets. The skeleton is
	   already on the mesh, so the parts record against its bone palette. */
	if (def->weapons_def)
		entity->mesh->recordParts(def->weapons_def->mesh, def->weapons_def->mesh_count);
	else if (def->animation_def)
		entity->mesh->recordParts(NULL, 0);
	else
		entity->mesh->recordObjects();

	return character;
}

void character3d_getBoneModelSpacePose(const Armature *skeleton, int16_t bone, Vector3 *position, Quaternion *rotation)
{
	uint16_t chain[16];
	int depth = 0;

	uint16_t idx = (uint16_t)bone;
	while (idx != 0xFFFF && depth < 16) {
		chain[depth++] = idx;
		idx = skeleton->skeletonRef->bones[idx].parentIdx;
	}

	*position = Vector3::zero();
	*rotation = Quaternion::identity();

	for (int i = depth - 1; i >= 0; i--) {
		const Bone *b = &skeleton->bones[chain[i]];

		*position += rotation->rotate(b->position);
		*rotation  = *rotation * b->rotation;
	}
}

/* Model-space pose of a bone, composed from the local TRS chain so it is
   current-frame (bone->matrix would lag one skeleton update behind). */
void character3d_getBonePose(const Armature *skeleton, int16_t bone, Vector3 *position, Quaternion *rotation)
{
	uint16_t chain[16];
	int depth = 0;

	uint16_t idx = (uint16_t)bone;
	while (idx != 0xFFFF && depth < 16) {
		chain[depth++] = idx;
		idx = skeleton->skeletonRef->bones[idx].parentIdx;
	}

	*position = Vector3::zero();
	*rotation = Quaternion::identity();

	for (int i = depth - 1; i >= 0; i--) {
		const Bone *b = &skeleton->bones[chain[i]];

		*position += rotation->rotate(b->position);
		*rotation  = *rotation * b->rotation;
	}
}

void character3d_delete(Character3D *character)
{
	Character3DAnimation *animation = &character->animation;

	if (animation->def) {
		for (int i = 0; i < animation->def->clip_count; i++) {
			animation->clip[i].destroy();
			if (animation->clip_data[i]) psyqo_free(animation->clip_data[i]);
		}
		for (int i = 0; i < animation->def->buffer_count; i++)
			animation->buffer[i].destroy();
		animation->main.destroy();
	}

	psyqo_free(animation->clip);
	psyqo_free(animation->clip_data);
	psyqo_free(animation->clip_cooldown);
	psyqo_free(animation->buffer);
	psyqo_free(animation->node_state);
	psyqo_free(animation->node_active);
	psyqo_free(character);
}
