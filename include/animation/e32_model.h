/*
 * Model file parser: objects, materials, skeleton, animation data and BVH
 * as stored in the model binary.
 *
 * Port of tiny3d's t3dmodel and the CPU side of t3d.h (Copyright (c) 2024
 * Max Bebök, MIT license, see LICENSE), modified from the original: renamed,
 * rewired to the engine's math types, reading the E32M format (flat vertices
 * and indices in the console's units, from this repo's gltf_to_e32 importer)
 * and stripped of the ucode API: on this console the render draws through
 * the GTE, and the file is read in place; only the textures move, into VRAM,
 * when the model is loaded.
 */
#ifndef ENGINE_32_MODEL_H
#define ENGINE_32_MODEL_H

#include <stdint.h>

#include "psyqo/primitives/common.hh"

#include "graphics/e32_model_format.h"
#include "graphics/e32_texture.h"
#include "physics/math/e32_vector3.h"
#include "physics/math/e32_quaternion.h"
#include "physics/math/e32_frustum.h"

/* keyframe times in the animation data files are stored in 1/60s ticks */
constexpr Fixed ANIM_KEYFRAME_TICK = 0.016666_fp;


/* Model vertex, straight from the file and fed to the GTE as-is: positions
 * 10.6 in the console's units, normals 4.12, color bytes and texel
 * coordinates. See ModelFileVertex. */
typedef ModelFileVertex RenderVertex;


struct Material
{
	const char *name;

	bool        textured;
	TextureSlot slot;   /* page and offset in VRAM, once uploaded */
	uint16_t    texWidth;
	uint16_t    texHeight;

	/* The emission map, when the material has one: the same polygons go
	   in a second time with this texture, added over the first. */
	bool        glow;
	TextureSlot glow_slot;

	/* The base color. It multiplies the texture when there is one, and is
	   the whole color when there is not. An alpha under 255 draws the
	   material semi transparent, half over what is behind it. */
	psyqo::Color color;
};

/* A triangle or a quad; see ModelFileFace. */
typedef ModelFileFace Face;

struct Object
{
	const char *name;
	uint32_t vertexCount;
	uint32_t faceCount;
	Material *material;
	uint8_t isVisible; /* set by culling checks, otherwise no effect on rendering */
	uint8_t _padding;
	uint8_t userValue0; /* free values usable by users */
	uint8_t userValue1; /* free values usable by users */
	int16_t aabbMin[3];
	int16_t aabbMax[3];

	/* Slices of the file, resolved to pointers at load time. Face indices
	   are object-local: they address 'vertices'. */
	const RenderVertex *vertices;
	const Face *faces;
	const uint8_t *boneIndices; /* NULL when the object is not skinned */

	/* The face across each edge of each face, four per face in the face's
	   own order, MODEL_NO_FACE where there is none. Built at load by
	   matching the edges' ends by position, since the importer splits the
	   vertices per face. The render reads it to cut shared edges alike. */
	const uint16_t *adjacent;
};

#define MODEL_NO_FACE 0xFFFF

struct BvhNode
{
	int16_t aabbMin[3];
	int16_t aabbMax[3];
	uint16_t value;
};

struct Bvh
{
	uint16_t nodeCount;
	uint16_t dataCount;
	const BvhNode *nodes;
	const uint16_t *data; /* object indices */
};

struct BoneData
{
	const char *name;
	uint16_t parentIdx;
	uint16_t depth;
	Vector3 scale;
	Quaternion rotation;
	Vector3 position;
};

struct SkeletonData
{
	uint16_t boneCount;
	uint16_t _reserved;
	BoneData *bones;
};

struct AnimationChannelMapping
{
	uint16_t targetIdx;
	uint8_t targetType;
	uint8_t attributeIdx;
	Fixed quantScale;
	Fixed quantOffset;
};

struct AnimationData
{
	const char *name;
	Fixed duration;
	uint32_t keyframeCount;
	uint16_t channelsQuat;
	uint16_t channelsScalar;
	const char *filePath;
	AnimationChannelMapping *channelMappings;
};

/* Kinds of things a model holds, for the iterator. */
enum ModelChunkType {
	CHUNK_TYPE_MATERIAL = 'M',
	CHUNK_TYPE_OBJECT   = 'O',
	CHUNK_TYPE_SKELETON = 'S',
	CHUNK_TYPE_ANIM     = 'A',
};

struct Model;

/* Traverses the objects, materials, skeleton or animations of a model:
 *
 *   ModelIter it = model->iterCreate(CHUNK_TYPE_OBJECT);
 *   while(it.next()) { ... it.object ... }
 *
 * The iterator does not need to be freed; the model must outlive it. */
struct ModelIter
{
	union {
		void *chunk;
		Object *object;
		Material *material;
		SkeletonData *skeleton;
		AnimationData *anim;
	};

	const Model *_model;
	uint16_t _idx;
	char _chunkType;

	/* Advances to the next chunk of its type. Returns false and sets 'chunk'
	 * to NULL once the end is reached. */
	bool next();
};

struct Model
{
	const ModelFileHeader *file;

	Object   *objects;
	Material *materials;
	uint16_t  objectCount;
	uint16_t  materialCount;

	SkeletonData  *skeleton;    /* NULL when the model has none */
	AnimationData *animations;
	uint16_t       animationCount;
	Bvh           *bvh;         /* NULL when the model has none */

	int16_t aabbMin[3];
	int16_t aabbMax[3];


	/* Loads a model from a file and uploads its textures. Free it with
	 * 'free'. NULL when the path is not there or the file is not a model. */
	static Model *load(const char *path);

	/* Frees the model and any related resources. The VRAM its textures took
	 * stays taken until Texture::init resets the shelf. */
	void free();

	/* Returns the first/main skeleton of a model, NULL if it has none. */
	const SkeletonData *getSkeleton() const { return skeleton; }

	/* Returns the number of animations in the model. */
	uint32_t getAnimationCount() const { return animationCount; }

	/* Stores the pointers to all animations inside the model into 'anims'.
	 * Use 'getAnimationCount' to allocate enough memory. */
	void getAnimations(AnimationData **anims) const;

	/* Returns an animation definition by name, NULL if not found. */
	AnimationData *getAnimation(const char *name) const;

	/* Returns an object by name, NULL if not found. */
	Object *getObject(const char *name) const;

	/* Returns an object by index. No bounds checking is done. */
	Object *getObjectByIndex(uint32_t index) const { return &objects[index]; }

	/* Returns a material by name, NULL if not found. */
	Material *getMaterial(const char *name) const;

	ModelIter iterCreate(ModelChunkType chunkType) const
	{
		ModelIter it;
		it.chunk      = NULL;
		it._model     = this;
		it._idx       = 0;
		it._chunkType = (char)chunkType;
		return it;
	}

	/* Returns the BVH of a model, NULL if it has none (pass '--bvh' to the
	 * gltf importer to create one). */
	const Bvh *bvhGet() const { return bvh; }

	/* Queries the BVH with a frustum (in model space), marking every reached
	 * object as visible via 'isVisible'. Clear the flags first. */
	void bvhQueryFrustum(const Frustum *frustum) const;
};


/* Vertex-buffer helpers. */

static inline const int16_t* vertbuffer_getPos(const RenderVertex vert[], int idx)
{
	return &vert[idx].x;
}

static inline const uint8_t* vertbuffer_getUv(const RenderVertex vert[], int idx)
{
	return &vert[idx].u;
}

static inline const uint8_t* vertbuffer_getRgba(const RenderVertex vert[], int idx)
{
	return &vert[idx].r;
}

static inline const int16_t* vertbuffer_getNorm(const RenderVertex vert[], int idx)
{
	return &vert[idx].nx;
}


#endif
