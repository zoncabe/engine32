/*
 * Model file parser.
 *
 * Port of tiny3d's t3dmodel (Copyright (c) 2024 Max Bebök, MIT license, see
 * LICENSE), modified from the original: renamed, rewired to the engine's
 * math, reading the E32M model format (flat vertices and indices, from this
 * repo's gltf_to_e32 importer). The file is embedded read only, so the
 * tables are built next to it instead of patched in place, and the textures
 * go to VRAM on load.
 */
#include "psyqo/alloc.h"
#include "psyqo/kernel.hh"

#include "animation/e32_model.h"
#include "resource/e32_resource.h"


static bool same(const char *a, const char *b)
{
	while (*a && *a == *b) { a++; b++; }
	return *a == *b;
}

static bool checkMagic(const ModelFileHeader *header)
{
	const char *magic = MODEL_MAGIC;
	for (int i = 0; i < 4; i++)
		if (header->magic[i] != magic[i]) return false;
	return header->version == MODEL_VERSION;
}

static void *allocate(size_t size)
{
	if (size == 0) return NULL;
	void *p = psyqo_malloc(size);
	psyqo::Kernel::assert(p != NULL, "model: out of memory");
	return p;
}

static Fixed fromRaw(int32_t raw)
{
	return Fixed(raw, Fixed::RAW);
}


/* Which face lies across each edge of each face. Two faces never share a
   vertex index, only a place, so every edge goes into a hash by the
   positions of its two ends, the nearer-to-origin end first so both faces
   build the same key; the second face to bring an edge is the first one's
   neighbour across it. The hash lives only while this runs. */
struct EdgeSlot
{
	uint32_t key[4];   /* x,y and z of each end */
	uint16_t face;
	uint8_t  edge;
	uint8_t  used;
};

static void edgeKey(const RenderVertex *a, const RenderVertex *b, uint32_t *key)
{
	uint32_t axy = (uint32_t)(uint16_t)a->x | ((uint32_t)(uint16_t)a->y << 16), az = (uint16_t)a->z;
	uint32_t bxy = (uint32_t)(uint16_t)b->x | ((uint32_t)(uint16_t)b->y << 16), bz = (uint16_t)b->z;

	if (axy < bxy || (axy == bxy && az < bz)) {
		key[0] = axy; key[1] = az; key[2] = bxy; key[3] = bz;
	} else {
		key[0] = bxy; key[1] = bz; key[2] = axy; key[3] = az;
	}
}

static void buildAdjacency(Object *obj)
{
	uint16_t *adjacent = (uint16_t *)allocate(sizeof(uint16_t) * 4 * obj->faceCount);
	obj->adjacent = adjacent;
	for (uint32_t i = 0; i < 4 * obj->faceCount; i++) adjacent[i] = MODEL_NO_FACE;

	/* room for twice the edges, so the probing stays short */
	uint32_t slots = 16;
	while (slots < obj->faceCount * 8) slots <<= 1;
	EdgeSlot *table = (EdgeSlot *)allocate(sizeof(EdgeSlot) * slots);
	for (uint32_t i = 0; i < slots; i++) table[i].used = 0;

	for (uint32_t f = 0; f < obj->faceCount; f++) {
		const Face *face = &obj->faces[f];
		int n = face->v[3] == MODEL_NO_VERTEX ? 3 : 4;

		for (int k = 0; k < n; k++) {
			uint32_t key[4];
			edgeKey(&obj->vertices[face->v[k]], &obj->vertices[face->v[k + 1 == n ? 0 : k + 1]], key);

			uint32_t h = (key[0] * 2654435761u) ^ (key[1] * 40503u) ^ (key[2] * 97u) ^ key[3];
			h ^= h >> 15;

			for (uint32_t i = h & (slots - 1); ; i = (i + 1) & (slots - 1)) {
				EdgeSlot *slot = &table[i];
				if (!slot->used) {
					slot->key[0] = key[0]; slot->key[1] = key[1];
					slot->key[2] = key[2]; slot->key[3] = key[3];
					slot->face = (uint16_t)f;
					slot->edge = (uint8_t)k;
					slot->used = 1;
					break;
				}
				if (slot->key[0] == key[0] && slot->key[1] == key[1]
				 && slot->key[2] == key[2] && slot->key[3] == key[3]) {
					adjacent[f * 4 + k] = slot->face;
					adjacent[slot->face * 4 + slot->edge] = (uint16_t)f;
					break;
				}
			}
		}
	}

	psyqo_free(table);
}

Model *Model::load(const char *path)
{
	size_t size;
	const uint8_t *data = Resource::find(path, &size);
	if (!data || size < sizeof(ModelFileHeader)) return NULL;

	const ModelFileHeader *header = (const ModelFileHeader *)data;
	if (!checkMagic(header)) return NULL;

	Model *model = (Model *)allocate(sizeof(Model));
	*model = Model();

	model->file          = header;
	model->objectCount   = header->object_count;
	model->materialCount = header->material_count;
	for (int i = 0; i < 3; i++) {
		model->aabbMin[i] = header->aabb_min[i];
		model->aabbMax[i] = header->aabb_max[i];
	}

	/* materials, textures to VRAM */
	model->materials = (Material *)allocate(sizeof(Material) * header->material_count);
	const ModelFileMaterial *file_material = (const ModelFileMaterial *)(data + header->materials);
	for (int i = 0; i < header->material_count; i++) {
		const ModelFileMaterial *src = &file_material[i];
		Material *mat = &model->materials[i];

		mat->name      = (const char *)(data + src->name);
		mat->texWidth  = src->width;
		mat->texHeight = src->height;
		mat->color     = {{src->color[0], src->color[1], src->color[2], src->color[3]}};
		mat->textured  = false;
		mat->glow      = false;

		if (src->width && src->height) {
			const uint16_t *pixels = (const uint16_t *)(data + src->pixels);
			mat->textured = Texture::upload(pixels, src->width, src->height, &mat->slot);
		}

		/* The glow adds: its page carries the additive blend, where the
		   base texture's carries the half and half of a transparent material. */
		if (mat->textured && src->glow_width && src->glow_height) {
			const uint16_t *pixels = (const uint16_t *)(data + src->glow_pixels);
			mat->glow = Texture::upload(pixels, src->glow_width, src->glow_height, &mat->glow_slot);
			if (mat->glow) mat->glow_slot.tpage.set(psyqo::Prim::TPageAttr::FullBackAndFullFront);
		}
	}

	/* objects */
	model->objects = (Object *)allocate(sizeof(Object) * header->object_count);
	const ModelFileObject *file_object = (const ModelFileObject *)(data + header->objects);
	for (int i = 0; i < header->object_count; i++) {
		const ModelFileObject *src = &file_object[i];
		Object *obj = &model->objects[i];

		obj->name        = (const char *)(data + src->name);
		obj->vertexCount = src->vertex_count;
		obj->faceCount   = src->face_count;
		obj->material    = src->material < header->material_count
		                 ? &model->materials[src->material] : NULL;
		obj->isVisible   = 1;
		obj->_padding    = 0;
		obj->userValue0  = 0;
		obj->userValue1  = 0;
		for (int k = 0; k < 3; k++) {
			obj->aabbMin[k] = src->aabb_min[k];
			obj->aabbMax[k] = src->aabb_max[k];
		}
		obj->vertices    = (const RenderVertex *)(data + src->vertices);
		obj->faces       = (const Face *)(data + src->faces);
		obj->boneIndices = src->bone_indices ? data + src->bone_indices : NULL;

		buildAdjacency(obj);
	}

	/* skeleton */
	if (header->skeleton) {
		const ModelFileSkeleton *src = (const ModelFileSkeleton *)(data + header->skeleton);
		const ModelFileBone *bones = (const ModelFileBone *)(data + src->bones);

		SkeletonData *skel = (SkeletonData *)allocate(sizeof(SkeletonData));
		skel->boneCount = src->bone_count;
		skel->_reserved = 0;
		skel->bones     = (BoneData *)allocate(sizeof(BoneData) * src->bone_count);

		for (int j = 0; j < src->bone_count; j++) {
			const ModelFileBone *b = &bones[j];
			BoneData *bone = &skel->bones[j];
			bone->name      = (const char *)(data + b->name);
			bone->parentIdx = b->parent;
			bone->depth     = b->depth;
			bone->scale     = { fromRaw(b->scale[0]), fromRaw(b->scale[1]), fromRaw(b->scale[2]) };
			bone->rotation  = { fromRaw(b->rotation[0]), fromRaw(b->rotation[1]),
			                    fromRaw(b->rotation[2]), fromRaw(b->rotation[3]) };
			bone->position  = { fromRaw(b->position[0]), fromRaw(b->position[1]), fromRaw(b->position[2]) };
		}
		model->skeleton = skel;
	}

	/* animations */
	model->animationCount = header->animation_count;
	model->animations = (AnimationData *)allocate(sizeof(AnimationData) * header->animation_count);
	const ModelFileAnimation *file_anim = (const ModelFileAnimation *)(data + header->animations);
	for (int i = 0; i < header->animation_count; i++) {
		const ModelFileAnimation *src = &file_anim[i];
		AnimationData *anim = &model->animations[i];

		anim->name           = (const char *)(data + src->name);
		anim->duration       = fromRaw(src->duration);
		anim->keyframeCount  = src->keyframe_count;
		anim->channelsQuat   = src->channels_quat;
		anim->channelsScalar = src->channels_scalar;
		anim->filePath       = (const char *)(data + src->file_path);

		uint32_t channels = (uint32_t)src->channels_quat + src->channels_scalar;
		anim->channelMappings = (AnimationChannelMapping *)allocate(sizeof(AnimationChannelMapping) * channels);
		const ModelFileAnimationChannel *ch = (const ModelFileAnimationChannel *)(data + src->channels);
		for (uint32_t c = 0; c < channels; c++) {
			anim->channelMappings[c].targetIdx    = ch[c].target;
			anim->channelMappings[c].targetType   = ch[c].target_type;
			anim->channelMappings[c].attributeIdx = ch[c].attribute;
			anim->channelMappings[c].quantScale   = fromRaw(ch[c].quant_scale);
			anim->channelMappings[c].quantOffset  = fromRaw(ch[c].quant_offset);
		}
	}

	/* bvh: the nodes and data are used as the file has them */
	if (header->bvh) {
		const ModelFileBvh *src = (const ModelFileBvh *)(data + header->bvh);
		Bvh *tree = (Bvh *)allocate(sizeof(Bvh));
		tree->nodeCount = src->node_count;
		tree->dataCount = src->data_count;
		tree->nodes     = (const BvhNode *)(data + src->nodes);
		tree->data      = (const uint16_t *)(data + src->data);
		model->bvh = tree;
	}

	return model;
}

void Model::free()
{
	if (bvh) psyqo_free(bvh);

	for (int i = 0; i < animationCount; i++)
		psyqo_free(animations[i].channelMappings);
	if (animations) psyqo_free(animations);

	if (skeleton) {
		psyqo_free(skeleton->bones);
		psyqo_free(skeleton);
	}

	for (int i = 0; i < objectCount; i++)
		if (objects[i].adjacent) psyqo_free((void *)objects[i].adjacent);
	if (objects)   psyqo_free(objects);
	if (materials) psyqo_free(materials);
	psyqo_free(this);
}

AnimationData *Model::getAnimation(const char *name) const
{
	for (int i = 0; i < animationCount; i++)
		if (same(animations[i].name, name)) return &animations[i];
	return NULL;
}

Object *Model::getObject(const char *name) const
{
	for (int i = 0; i < objectCount; i++)
		if (objects[i].name && same(objects[i].name, name)) return &objects[i];
	return NULL;
}

void Model::getAnimations(AnimationData **anims) const
{
	for (int i = 0; i < animationCount; i++)
		anims[i] = &animations[i];
}

Material *Model::getMaterial(const char *name) const
{
	for (int i = 0; i < materialCount; i++)
		if (materials[i].name && same(materials[i].name, name)) return &materials[i];
	return NULL;
}

bool ModelIter::next()
{
	const Model *model = _model;

	switch (_chunkType) {
		case CHUNK_TYPE_OBJECT:
			if (_idx < model->objectCount) { object = &model->objects[_idx++]; return true; }
			break;
		case CHUNK_TYPE_MATERIAL:
			if (_idx < model->materialCount) { material = &model->materials[_idx++]; return true; }
			break;
		case CHUNK_TYPE_SKELETON:
			if (_idx == 0 && model->skeleton) { _idx++; skeleton = model->skeleton; return true; }
			break;
		case CHUNK_TYPE_ANIM:
			if (_idx < model->animationCount) { anim = &model->animations[_idx++]; return true; }
			break;
	}

	chunk = NULL;
	return false;
}

/* context for the functions below, this avoids blowing up the stack-size */
static const Frustum *ctxFrustum;
static const Model *ctxModel;

static void bvh_query_node(const BvhNode *node)
{
	int dataCount = node->value & 0b1111;
	int offset = (int16_t)node->value >> 4;

	if (dataCount == 0) {
		if (ctxFrustum->vsAabbS16(node->aabbMin, node->aabbMax)) {
			bvh_query_node(&node[offset]);
			bvh_query_node(&node[offset + 1]);
		}
		return;
	}

	const uint16_t *data = ctxModel->bvh->data;
	int offsetEnd = offset + dataCount;
	while (offset < offsetEnd) {
		Object *obj = &ctxModel->objects[data[offset++]];
		if (ctxFrustum->vsAabbS16(obj->aabbMin, obj->aabbMax)) {
			obj->isVisible = true;
		}
	}
}

void Model::bvhQueryFrustum(const Frustum *frustum) const
{
	if (!bvh) return;
	ctxFrustum = frustum;
	ctxModel   = this;
	bvh_query_node(bvh->nodes);
}
