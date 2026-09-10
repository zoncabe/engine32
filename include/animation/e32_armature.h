/*
 * Armature: live skeleton instance built from a model's skeleton data.
 * Bone poses are blended and composed on the CPU; the resulting transforms
 * are the palette the render skins vertices with, one GTE matrix per bone.
 *
 * Port of tiny3d's t3dskeleton (Copyright (c) 2024 Max Bebök, MIT license,
 * see LICENSE), modified from the original: renamed, rewired to the engine's
 * math types, and producing the GTE's layout: a 3x3 rotation with the scale
 * folded in, plus a translation. The tiny3d segment-table mechanism for
 * buffered skeletons is gone; the current buffer is resolved on the CPU at
 * draw time.
 */
#ifndef ENGINE_32_ARMATURE_H
#define ENGINE_32_ARMATURE_H

#include "animation/e32_model.h"
#include "physics/math/e32_transform.h"

/*
 * Bone instance, part of an armature. 'matrix' gets updated by
 * 'Armature::update' if 'hasChanged' is set.
 */
struct Bone
{
	Transform matrix;   /* model space, scale folded into the rotation */
	Vector3 scale;
	Quaternion rotation;
	Vector3 position;
	int32_t hasChanged;
};

/*
 * Armature instance, constructed from a model's skeleton data.
 * This is what skinned models are drawn with.
 */
struct Armature
{
	Bone* bones;
	Transform* boneMatricesFP; /* the palette the render reads, one per bone per buffer */
	uint8_t bufferCount; /* number of matrix buffers */
	uint8_t currentBufferIdx;
	const SkeletonData* skeletonRef; /* reference to the model, defines the skeleton structure */


	/*
	 * Creates an armature instance from a model's skeleton data.
	 * It reserves multiple matrix buffers so a skeleton can be updated while
	 * the last frame is still being rendered. Only the palette is buffered,
	 * the bone data itself is not.
	 * 'bufferCount' should match the frame-buffer count.
	 */
	static Armature createBuffered(const Model *model, int bufferCount);

	static Armature create(const Model *model) { return createBuffered(model, 1); }

	/*
	 * Returns the bone matrix palette for the next draw call: the buffer
	 * that 'update' last wrote.
	 */
	const Transform *getMatrices() const
	{
		return &boneMatricesFP[currentBufferIdx * skeletonRef->boneCount];
	}

	/*
	 * Clones an armature. With 'useMatrices' false no matrices are
	 * allocated, which is useful for blending animations.
	 */
	Armature clone(bool useMatrices) const;

	/*
	 * Resets an armature to its initial state (resting pose).
	 * To recalculate the bone matrices too, call 'update' afterwards.
	 */
	void reset();

	/*
	 * Blends two armatures into this one. It is safe to use this armature as
	 * an input too. The factor may go beyond [0,1] to "overdrive" animations.
	 */
	void blend(const Armature *a, const Armature *b, Fixed factor);

	/*
	 * Updates the armature's bone matrices if bone data has changed.
	 * Call this after making changes to a bone's pos/rot/scale; the bone's
	 * 'hasChanged' flag must also be set.
	 */
	void update();

	/* Frees data allocated in the armature. Safe to call multiple times. */
	void destroy();

	/* Returns the index of the bone with the given name, or -1 if not found. */
	int findBone(const char *name) const;

	/*
	 * Gets the position in model space of a bone from its matrix.
	 * Assumes the bone matrix was updated beforehand with 'update'.
	 */
	Vector3 getBonePosModelSpace(int boneIdx) const
	{
		return bones[boneIdx].matrix.position;
	}
};


#endif
