/*
 * Port of tiny3d's t3danim (Copyright (c) 2024 Max Bebök, MIT license, see
 * LICENSE). See the header for the differences from the original.
 */
#include "psyqo/alloc.h"
#include "psyqo/kernel.hh"

#include "animation/e32_animation.h"
#include "physics/math/e32_math_common.h"
#include "resource/e32_resource.h"

// Maps the input data streamed from the animation data file
struct AnimationKF
{
	uint16_t nextTime;
	uint16_t channelIdx;
	uint16_t data[2]; // can be either 1 or 2 16-bit values (scalar / quat)
};

Animation Animation::create(const Model *model, const char *name)
{
	AnimationData* animDef = model->getAnimation(name);
	psyqo::Kernel::assert(animDef != NULL, "animation: not found in model");

	Animation anim;
	anim.animRef = animDef;
	anim.targetsScalar = NULL;
	anim.targetsQuat = NULL;
	anim.time = Fixed();
	anim.speed = 1.0_fp;
	anim.nextKfSize = sizeof(AnimationKF);
	anim.file = Resource::find(animDef->filePath, &anim.fileSize);
	anim.cursor = 0;
	anim.playing = 1;
	anim.looping = 1;
	psyqo::Kernel::assert(anim.file != NULL, "animation: keyframe file not found");
	return anim;
}

static void rewind_anim(Animation *anim)
{
	for (int c = 0; c < anim->animRef->channelsScalar; c++) {
		anim->targetsScalar[c].base.timeEnd = Fixed();
	}
	for (int c = 0; c < anim->animRef->channelsQuat; c++) {
		anim->targetsQuat[c].base.timeEnd = Fixed();
	}
	anim->nextKfSize = sizeof(AnimationKF);
	anim->cursor = 0;
}

void Animation::attach(const Armature *armature)
{
	if (targetsQuat) psyqo_free(targetsQuat);

	size_t allocQuat = sizeof(AnimationTargetQuat) * animRef->channelsQuat;
	size_t allocScalar = sizeof(AnimationTargetScalar) * animRef->channelsScalar;
	uint8_t *block = (uint8_t *)psyqo_malloc(allocQuat + allocScalar); // only allocate a single block
	psyqo::Kernel::assert(block != NULL, "animation: out of memory");
	for (size_t i = 0; i < allocQuat + allocScalar; i++) block[i] = 0;
	targetsQuat = (AnimationTargetQuat*)block;
	targetsScalar = (AnimationTargetScalar*)(block + allocQuat);
	rewind_anim(this);

	uint32_t channelCount = animRef->channelsScalar + animRef->channelsQuat;

	uint32_t idxQuat = 0;
	uint32_t idxScalar = 0;
	for (uint32_t i = 0; i < channelCount; i++)
	{
		AnimationChannelMapping *channelMap = &animRef->channelMappings[i];
		Bone *bone = &armature->bones[channelMap->targetIdx];

		switch (channelMap->targetType) {
			case ANIM_TARGET_TRANSLATION:
				targetsScalar[idxScalar].targetScalar = &bone->position[channelMap->attributeIdx];
				targetsScalar[idxScalar++].base.changedFlag = &bone->hasChanged;
				break;
			case ANIM_TARGET_SCALE_XYZ:
				targetsScalar[idxScalar].targetScalar = &bone->scale[channelMap->attributeIdx];
				targetsScalar[idxScalar++].base.changedFlag = &bone->hasChanged;
				break;
			case ANIM_TARGET_ROTATION:
				targetsQuat[idxQuat].targetQuat = &bone->rotation;
				targetsQuat[idxQuat++].base.changedFlag = &bone->hasChanged;
			break;
			default: psyqo::Kernel::assert(false, "animation: unknown target type");
		}
	}
}

inline static void attach_scalar(Animation* anim, uint32_t targetIdx, Vector3* target, int32_t *updateFlag, uint8_t targetType)
{
	for (int i = 0; i < anim->animRef->channelsScalar; i++) {
		AnimationChannelMapping *channelMap = &anim->animRef->channelMappings[i + anim->animRef->channelsQuat];
		if (channelMap->targetIdx == targetIdx && channelMap->targetType == targetType) {
			anim->targetsScalar[i].targetScalar = &(*target)[channelMap->attributeIdx];
			anim->targetsScalar[i].base.changedFlag = updateFlag;
		}
	}
}

void Animation::attachPos(uint32_t targetIdx, Vector3* target, int32_t *updateFlag)
{
	attach_scalar(this, targetIdx, target, updateFlag, ANIM_TARGET_TRANSLATION);
}

void Animation::attachScale(uint32_t targetIdx, Vector3 *target, int32_t *updateFlag)
{
	attach_scalar(this, targetIdx, target, updateFlag, ANIM_TARGET_SCALE_XYZ);
}

void Animation::attachRot(uint32_t targetIdx, Quaternion *target, int32_t *updateFlag)
{
	for (int i = 0; i < animRef->channelsQuat; i++) {
		AnimationChannelMapping *channelMap = &animRef->channelMappings[i];
		if (channelMap->targetIdx == targetIdx && channelMap->targetType == ANIM_TARGET_ROTATION) {
			targetsQuat[i].targetQuat = target;
			targetsQuat[i].base.changedFlag = updateFlag;
		}
	}
}

static inline AnimationTargetBase* get_base_target(Animation *anim, uint32_t channelIdx, bool isRot)
{
	return isRot ?
		(AnimationTargetBase*)&anim->targetsQuat[channelIdx] :
		(AnimationTargetBase*)&anim->targetsScalar[channelIdx - anim->animRef->channelsQuat];
}

static inline bool load_keyframe(Animation *anim)
{
	AnimationKF kf;
	if (anim->cursor + anim->nextKfSize > anim->fileSize) return false;
	const uint8_t *src = anim->file + anim->cursor;
	uint8_t *dst = (uint8_t*)&kf;
	for (int i = 0; i < anim->nextKfSize; i++) dst[i] = src[i];
	anim->cursor += anim->nextKfSize;

	bool isLarge = kf.nextTime & 0x8000;
	anim->nextKfSize = isLarge ? sizeof(AnimationKF) : (sizeof(AnimationKF) - 2);
	kf.nextTime &= 0x7FFF;

	AnimationChannelMapping *channelMap = &anim->animRef->channelMappings[kf.channelIdx];

	bool isRot = kf.channelIdx < anim->animRef->channelsQuat;
	AnimationTargetBase *targetBase = get_base_target(anim, kf.channelIdx, isRot);

	targetBase->timeStart = targetBase->timeEnd;
	targetBase->timeEnd += ANIM_KEYFRAME_TICK * (int)kf.nextTime;
	if (kf.nextTime == 0) targetBase->timeStart -= TOLERANCE; // avoid zero-div for overlapping keyframes

	if (channelMap->targetType == ANIM_TARGET_ROTATION) {
		AnimationTargetQuat *target = (AnimationTargetQuat*)targetBase;
		target->kfCurr = target->kfNext;
		target->kfNext = Quaternion::unpacked(kf.data[0], kf.data[1]);
	} else {
		AnimationTargetScalar *target = (AnimationTargetScalar*)targetBase;
		target->kfCurr = target->kfNext;
		target->kfNext = channelMap->quantScale * (int)kf.data[0] + channelMap->quantOffset;
	}

	return true;
}

void Animation::update(Fixed deltaTime)
{
	if (!playing) return;
	int32_t updateFlag = 1;
	time += deltaTime * speed;

	if (time >= animRef->duration) {
		time -= animRef->duration;
		rewind_anim(this);
		updateFlag = 2;

		if (!looping) {
			playing = 0;
			return;
		}
	}

	uint32_t channelCount = animRef->channelsScalar + animRef->channelsQuat;
	for (uint32_t c = 0; c < channelCount; c++)
	{
		bool isRot = c < animRef->channelsQuat;
		AnimationTargetBase *target = get_base_target(this, c, isRot);

		while (time >= target->timeEnd) {
			if (!load_keyframe(this)) break;
		}

		Fixed timeDiff = target->timeEnd - target->timeStart;
		Fixed interp = (time - target->timeStart) / timeDiff;
		*target->changedFlag = updateFlag;

		if (isRot) {
			AnimationTargetQuat *t = (AnimationTargetQuat*)target;
			*t->targetQuat = t->kfCurr.nlerp(t->kfNext, interp);
		} else {
			AnimationTargetScalar *t = (AnimationTargetScalar*)target;
			*t->targetScalar = lerp(t->kfCurr, t->kfNext, interp);
		}
	}
}

void Animation::destroy()
{
	if (targetsQuat) psyqo_free(targetsQuat); // 'targetsScalar' is part of this memory-block
	targetsQuat = NULL;
	targetsScalar = NULL;
	file = NULL;
	cursor = 0;
}

void Animation::setTime(Fixed value)
{
	if (value > animRef->duration) value = animRef->duration;
	if (value < time) rewind_anim(this);
	time = value;
}
