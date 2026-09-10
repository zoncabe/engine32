/*
 * Animation instance: streams quantized keyframes from the model's animation
 * data file and writes the interpolated values into its attached targets
 * (usually an armature's bones).
 *
 * Port of tiny3d's t3danim (Copyright (c) 2024 Max Bebök, MIT license, see
 * LICENSE), modified from the original: renamed and rewired to the engine's
 * math types. The keyframe file is embedded read only, so the stream is a
 * cursor over it instead of a FILE.
 */
#ifndef ENGINE_32_ANIMATION_H
#define ENGINE_32_ANIMATION_H

#include <stddef.h>

#include "animation/e32_model.h"
#include "animation/e32_armature.h"

struct AnimationTargetBase
{
	Fixed timeStart;
	Fixed timeEnd;
	int32_t* changedFlag; /* flag to increment when target is changed */
};

struct AnimationTargetQuat
{
	AnimationTargetBase base;
	Quaternion* targetQuat; /* target to modify */
	Quaternion kfCurr; /* current keyframe value */
	Quaternion kfNext; /* next keyframe value */
};

struct AnimationTargetScalar
{
	AnimationTargetBase base;
	Fixed* targetScalar;
	Fixed kfCurr;
	Fixed kfNext;
};

struct Animation
{
	AnimationData *animRef;
	AnimationTargetQuat *targetsQuat;
	AnimationTargetScalar *targetsScalar;

	Fixed speed;
	Fixed time;

	/* The keyframe stream and where the next read starts. */
	const uint8_t *file;
	size_t fileSize;
	size_t cursor;
	int nextKfSize;
	uint8_t playing;
	uint8_t looping;


	/* Creates an animation instance from a model's animation data, by name. */
	static Animation create(const Model *model, const char* name);

	/* Attaches an animation to an armature: every channel targets its bone. */
	void attach(const Armature* armature);

	/*
	 * Attach a single position/rotation/scale target to a single channel,
	 * overriding an earlier 'attach'. 'updateFlag' is set to 1 when the
	 * target changed, 2 when the animation rolled over.
	 */
	void attachPos(uint32_t targetIdx, Vector3* target, int32_t *updateFlag);
	void attachRot(uint32_t targetIdx, Quaternion* target, int32_t *updateFlag);
	void attachScale(uint32_t targetIdx, Vector3* target, int32_t *updateFlag);

	/* Advances the animation and applies the changes to its targets. */
	void update(Fixed deltaTime);

	/*
	 * Sets the animation to a specific time.
	 * Rewinding may cause some work internally due to potential DMAs.
	 */
	void setTime(Fixed value);

	/* Current time of the animation in seconds. */
	Fixed getTime() const { return time; }

	/* Length of the animation in seconds. */
	Fixed getLength() const { return animRef->duration; }

	/*
	 * Sets the playback speed as a factor, default 1.0.
	 * Reverse playback (speed < 0) is not supported.
	 */
	void setSpeed(Fixed value) { speed = value < 0 ? Fixed() : value; }

	/* Play or pause. Works independently of setting the speed to 0. */
	void setPlaying(bool isPlaying) { playing = isPlaying; }

	/*
	 * Whether the animation is still playing. Looping animations always are
	 * once started; non-looping ones stop at the end.
	 */
	bool isPlaying() const { return playing != 0; }

	/*
	 * Loop or stop at the end. A non-looping animation needs 'setPlaying'
	 * to play again.
	 */
	void setLooping(bool loop) { looping = loop; }

	/* Frees data allocated in the animation. */
	void destroy();
};


#endif
