/*
	Camera-pitch aim bend. See character3d_aim.h for the frame math rationale.
*/
#include <fmath.h>

#include "animation/e32_armature.h"

#include "physics/math/e32_math_common.h"
#include "camera/e32_spring_arm.h"
#include "viewport/e32_viewport.h"
#include "entity/e32_entity3d.h"
#include "character3d/e32_character3d.h"


void character3dAim_init(Character3D *character, const Character3DAimingSettings *settings)
{
	Character3DAiming *aiming = &character->aiming;

	aiming->count = settings->count < CHARACTER3D_AIM_MAX_BONES
	              ? settings->count : CHARACTER3D_AIM_MAX_BONES;
	aiming->pitch_scale = settings->pitch_scale;

	for (uint8_t i = 0; i < aiming->count; i++)
		aiming->bone[i] = (int16_t)armature_findBone(&character->animation.main,
		                                                  (char *)settings->bone[i]);
}

void character3dAim_apply(Armature *skeleton, void *context)
{
	Character3D *character = context;
	Character3DAiming *aiming = &character->aiming;

	/* The hold is already half an aim: both modes bend, and the combined
	   presence keeps the weight steady through the hold-aim crossfade. */
	float hold  = character->animation.aiming_blend;
	float draw  = character->animation.charging_shoot_blend;
	float blend = 1.0f - (1.0f - hold) * (1.0f - draw);
	if (blend <= 0.0f || aiming->count == 0) return;

	const Camera *camera = &viewport_get()->camera;
	float pitch = cameraSpringArm_getPitch(camera);
	if (pitch == 0.0f) return;

	/* The bend is about the camera's horizontal right. That is the model's X
	   only while the body faces the camera, which the strafe does but nothing
	   guarantees: mid-turn, or with the strafe off while the pose fades, the
	   body sits at its own yaw. So the axis is turned by however far the body
	   is off from facing the camera, which is zero once it is. */
	float facing = cameraSpringArm_getYaw(camera) + 180.0f;
	float offset = (angle_wrap_relative(facing, character->body.rotation.z)
	                - character->body.rotation.z) * 0.01745329f;

	/* One share of the bend per vertebra. */
	float half = pitch * aiming->pitch_scale * blend / (float)aiming->count
	           * 0.5f * 0.01745329f;

	float sin_half = fm_sinf(half);
	Quaternion delta = { fm_cosf(offset) * sin_half,
	                   fm_sinf(offset) * sin_half,
	                   0.0f,
	                   fm_cosf(half) };

	for (uint8_t i = 0; i < aiming->count; i++) {
		int16_t b = aiming->bone[i];
		if (b < 0) continue;

		/* Conjugate the model-space delta into this bone's frame through the
		   parent chain as it stands right now — earlier vertebrae already
		   carry their share, so each one bends about the same world axis. */
		Vector3 parent_pos;
		Quaternion parent_rot;
		character3d_getBonePose(skeleton, (int16_t)skeleton->skeletonRef->bones[b].parentIdx,
		                      &parent_pos, &parent_rot);

		Quaternion inverse = { -parent_rot.x, -parent_rot.y, -parent_rot.z, parent_rot.w };
		Bone *bone = &skeleton->bones[b];

		Quaternion q = quaternion_product(&delta, &parent_rot);
		Quaternion local = quaternion_product(&inverse, &q);

		bone->rotation = quaternion_product(&local, &bone->rotation);
		bone->hasChanged = 1;
	}
}
