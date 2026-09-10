/*
 * Port of tiny3d's t3dskeleton (Copyright (c) 2024 Max Bebök, MIT license,
 * see LICENSE). See the header for the differences from the original.
 */
#include "psyqo/alloc.h"
#include "psyqo/kernel.hh"

#include "animation/e32_armature.h"


static bool same(const char *a, const char *b)
{
	while (*a && *a == *b) { a++; b++; }
	return *a == *b;
}

/* Scale, then rotate, then translate: the rotation's columns carry the
   scale, the way the GTE wants a single 3x3. */
static Transform transform_fromSrt(const Vector3 *scale, const Quaternion *rotation, const Vector3 *position)
{
	Transform t;
	t.rotation = rotation->toMatrix3();
	t.rotation.ex *= scale->x;
	t.rotation.ey *= scale->y;
	t.rotation.ez *= scale->z;
	t.position = *position;
	return t;
}

Armature Armature::createBuffered(const Model *model, int bufferCount)
{
	const SkeletonData *skelRef = model->getSkeleton();
	psyqo::Kernel::assert(skelRef != NULL, "armature: model has no skeleton");

	Armature armature;
	armature.bones = (Bone *)psyqo_malloc(sizeof(Bone) * skelRef->boneCount);
	armature.boneMatricesFP = (Transform *)psyqo_malloc(sizeof(Transform) * skelRef->boneCount * bufferCount);
	armature.skeletonRef = skelRef;
	armature.bufferCount = bufferCount;
	armature.currentBufferIdx = 0;
	psyqo::Kernel::assert(armature.bones != NULL && armature.boneMatricesFP != NULL, "armature: out of memory");

	armature.reset();

	/* Compose the rest pose right away, so the palette is never read
	 * uninitialised. */
	armature.update();
	return armature;
}

Armature Armature::clone(bool useMatrices) const
{
	Armature result;
	result.bones = (Bone *)psyqo_malloc(sizeof(Bone) * skeletonRef->boneCount);
	result.boneMatricesFP = NULL;
	result.bufferCount = bufferCount;
	result.currentBufferIdx = currentBufferIdx;
	result.skeletonRef = skeletonRef;
	psyqo::Kernel::assert(result.bones != NULL, "armature: out of memory");
	for (int i = 0; i < skeletonRef->boneCount; i++)
		result.bones[i] = bones[i];

	if (useMatrices) {
		size_t count = (size_t)skeletonRef->boneCount * bufferCount;
		result.boneMatricesFP = (Transform *)psyqo_malloc(sizeof(Transform) * count);
		psyqo::Kernel::assert(result.boneMatricesFP != NULL, "armature: out of memory");
		for (size_t i = 0; i < count; i++)
			result.boneMatricesFP[i] = boneMatricesFP[i];
	}
	return result;
}

void Armature::reset()
{
	for (int i = 0; i < skeletonRef->boneCount; i++) {
		const BoneData *boneDef = &skeletonRef->bones[i];
		bones[i].scale = boneDef->scale;
		bones[i].rotation = boneDef->rotation;
		bones[i].position = boneDef->position;
		bones[i].hasChanged = true;
	}
}

void Armature::blend(const Armature *a, const Armature *b, Fixed factor)
{
	for (int i = 0; i < skeletonRef->boneCount; i++) {
		Bone *boneRes = &bones[i];
		Bone *boneA = &a->bones[i];
		Bone *boneB = &b->bones[i];

		boneRes->hasChanged = true;
		boneRes->rotation = boneA->rotation.nlerp(boneB->rotation, factor);
		boneRes->position = boneA->position.lerp(boneB->position, factor);
		boneRes->scale = boneA->scale.lerp(boneB->scale, factor);
	}
}

void Armature::update()
{
	int updateLevel = -1;
	bool forceUpdate = false;

	Transform *matStackFP = NULL;

	for (int i = 0; i < skeletonRef->boneCount; i++)
	{
		Bone *bone = &bones[i];
		const BoneData *boneDef = &skeletonRef->bones[i];

		if (forceUpdate && boneDef->depth <= updateLevel) {
			forceUpdate = false;
			updateLevel = -1;
		}

		if (bone->hasChanged || forceUpdate)
		{
			// only cycle through matrices if at least one bone changes.
			// this avoids flickering at the end of an animation, since it would cycle through the last X frames otherwise.
			if (matStackFP == NULL)
			{
				currentBufferIdx = (currentBufferIdx + 1) % bufferCount;
				matStackFP = &boneMatricesFP[skeletonRef->boneCount * currentBufferIdx];
			}

			// if a bone changed we need to also update any children.
			// To do so, update all following bones until we hit one that has the same depth as the changed bone.
			if (!forceUpdate) updateLevel = boneDef->depth;
			forceUpdate = true;

			if (boneDef->parentIdx != 0xFFFF) {
				Transform tmp = transform_fromSrt(&bone->scale, &bone->rotation, &bone->position);
				bone->matrix = bones[boneDef->parentIdx].matrix * tmp;
			} else {
				bone->matrix = transform_fromSrt(&bone->scale, &bone->rotation, &bone->position);
			}

			matStackFP[i] = bone->matrix;

			// if a bone has changed, we need to force updating it until it reached all buffers.
			// otherwise once the updating stops, and we cycle through buffers still, it would flicker.
			// counting up here is also safe when this flag is set to 1 or 'true' externally (e.g.: in Animation::update)
			if (bones[i].hasChanged++ == bufferCount) {
				bones[i].hasChanged = 0;
			}
		}
	}
}

int Armature::findBone(const char *name) const
{
	for (int i = 0; i < skeletonRef->boneCount; i++) {
		if (same(skeletonRef->bones[i].name, name)) {
			return i;
		}
	}
	return -1;
}

void Armature::destroy()
{
	if (bones != NULL) {
		psyqo_free(bones);
		bones = NULL;
	}
	if (boneMatricesFP != NULL) {
		psyqo_free(boneMatricesFP);
		boneMatricesFP = NULL;
	}
	skeletonRef = NULL;
}
