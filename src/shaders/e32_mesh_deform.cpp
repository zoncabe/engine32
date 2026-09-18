#include "psyqo/alloc.h"

#include "animation/e32_model.h"

#include "shaders/e32_mesh_deform.h"


/* Positions live in the vertex buffer as int16, so quantising the source the
   same way turns the match into an exact integer compare: no tolerance to
   pick, and no risk of welding two points that are merely close. */

struct DeformBucket
{
	int16_t  pos[3];
	uint16_t source;   /* source index + 1; 0 marks an empty bucket */
};


/* Meters to the vertices' integer units, to the nearest. */
static int16_t quantizeAxis(Fixed v)
{
	int32_t raw = v.raw();
	raw += raw >= 0 ? 2048 : -2048;
	return (int16_t)(raw >> 12);
}

static void meshDeform_quantize(Vector3 v, Fixed scale, int16_t out[3])
{
	out[0] = quantizeAxis(v.x * scale);
	out[1] = quantizeAxis(v.y * scale);
	out[2] = quantizeAxis(v.z * scale);
}

/* Normals are 4.12 in the vertex, one axis per int16. */
static int16_t packAxis(Fixed v)
{
	int32_t raw = v.raw();
	if (raw >  4095) raw =  4095;
	if (raw < -4096) raw = -4096;
	return (int16_t)raw;
}

static void meshDeform_packNormal(Vector3 n, Fixed sign, int16_t out[3])
{
	out[0] = packAxis(n.x * sign);
	out[1] = packAxis(n.y * sign);
	out[2] = packAxis(n.z * sign);
}

static Vector3 meshDeform_unpackNormal(const int16_t packed[3])
{
	return { Fixed(packed[0], Fixed::RAW), Fixed(packed[1], Fixed::RAW), Fixed(packed[2], Fixed::RAW) };
}

static bool samePos(const int16_t a[3], const int16_t b[3])
{
	return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

static uint32_t meshDeform_hash(const int16_t pos[3])
{
	/* FNV-1a over the six bytes of the quantised position */
	uint32_t h = 2166136261u;

	for (int i = 0; i < 3; i++) {
		uint16_t bits = (uint16_t)pos[i];
		h ^= bits & 0xFF;  h *= 16777619u;
		h ^= bits >> 8;    h *= 16777619u;
	}
	return h;
}

static void meshDeform_insert(DeformBucket *bucket, uint32_t mask, const int16_t pos[3], uint16_t source)
{
	uint32_t i = meshDeform_hash(pos) & mask;

	while (bucket[i].source != 0) {
		/* Duplicated source points would map every slot to the first one
		   anyway, so keeping the earlier entry is the stable choice. */
		if (samePos(bucket[i].pos, pos)) return;
		i = (i + 1) & mask;
	}

	for (int k = 0; k < 3; k++) bucket[i].pos[k] = pos[k];
	bucket[i].source = source + 1;
}

static uint16_t meshDeform_find(const DeformBucket *bucket, uint32_t mask, const int16_t pos[3])
{
	uint32_t i = meshDeform_hash(pos) & mask;

	while (bucket[i].source != 0) {
		if (samePos(bucket[i].pos, pos)) return bucket[i].source - 1;
		i = (i + 1) & mask;
	}
	return MESH_DEFORM_UNBOUND;
}


bool MeshDeform::bind(Model *model_, const Vector3 *source_, const Vector3 *source_normal_,
                      uint16_t source_count, Fixed scale_)
{
	*this = MeshDeform();
	model         = model_;
	scale         = scale_;
	source        = source_;
	source_normal = source_normal_;
	normal_sign   = 1.0_fp;

	if (model == NULL || source == NULL || source_count == 0) return false;

	/* Every object's vertices, back to back. */
	object_offset = (uint32_t *)psyqo_malloc(sizeof(uint32_t) * model->objectCount);
	if (object_offset == NULL) return false;

	uint32_t total = 0;
	for (int o = 0; o < model->objectCount; o++) {
		object_offset[o] = total;
		total += model->objects[o].vertexCount;
	}

	slot_count  = (uint16_t)total;
	slot_source = (uint16_t *)psyqo_malloc(sizeof(uint16_t) * slot_count);
	if (slot_source == NULL) return false;

	/* Load factor stays under 0.5, so the linear probes stay short. */
	uint32_t capacity = 16;
	while (capacity < (uint32_t)source_count * 2) capacity *= 2;

	DeformBucket *bucket = (DeformBucket *)psyqo_malloc(capacity * sizeof(DeformBucket));
	if (bucket == NULL) {
		psyqo_free(slot_source);
		slot_source = NULL;
		return false;
	}
	for (uint32_t i = 0; i < capacity; i++) bucket[i].source = 0;

	for (uint16_t i = 0; i < source_count; i++) {
		int16_t pos[3];
		meshDeform_quantize(source[i], scale, pos);
		meshDeform_insert(bucket, capacity - 1, pos, i);
	}

	/* Summed over every bound slot: positive if the source's normals point the
	   same way as the model's, negative if the winding is reversed. */
	Fixed agreement = Fixed();

	for (int o = 0; o < model->objectCount; o++) {
		const Object *obj = &model->objects[o];

		for (uint32_t v = 0; v < obj->vertexCount; v++) {
			uint16_t slot = (uint16_t)(object_offset[o] + v);
			const int16_t *pos = vertbuffer_getPos(obj->positions, obj->vertices[v].position);
			uint16_t match = meshDeform_find(bucket, capacity - 1, pos);

			slot_source[slot] = match;
			if (match == MESH_DEFORM_UNBOUND) continue;

			bound_count++;

			if (source_normal) {
				Vector3 own = meshDeform_unpackNormal(
					vertbuffer_getNorm(obj->shades, obj->vertices[v].shade));
				Vector3 src = source_normal[match];

				agreement += own.dot(src);
			}
		}
	}

	if (source_normal && agreement < 0) normal_sign = -1.0_fp;

	psyqo_free(bucket);

	/* The corners, once: every slot points at its own position and shade,
	   and keeps the texel the model gave it. */
	vertex_map = (RenderVertex *)psyqo_malloc(sizeof(RenderVertex) * slot_count);
	if (vertex_map == NULL) return false;

	for (int o = 0; o < model->objectCount; o++) {
		const Object *obj = &model->objects[o];
		RenderVertex *dst = vertex_map + object_offset[o];
		for (uint32_t v = 0; v < obj->vertexCount; v++) {
			dst[v].position = (uint16_t)v;
			dst[v].shade    = (uint16_t)v;
			dst[v].u        = obj->vertices[v].u;
			dst[v].v        = obj->vertices[v].v;
			dst[v]._pad     = 0;
		}
	}

	/* And the model's own values into every buffer, so a slot that found no
	   source point stays where it was modelled. */
	for (int i = 0; i < MESH_DEFORM_BUFFERS; i++) {
		position_buffer[i] = (RenderPosition *)psyqo_malloc(sizeof(RenderPosition) * slot_count);
		shade_buffer[i]    = (RenderShade *)psyqo_malloc(sizeof(RenderShade) * slot_count);
		if (position_buffer[i] == NULL || shade_buffer[i] == NULL) return false;

		for (int o = 0; o < model->objectCount; o++) {
			const Object *obj = &model->objects[o];
			RenderPosition *dp = position_buffer[i] + object_offset[o];
			RenderShade    *ds = shade_buffer[i]    + object_offset[o];
			for (uint32_t v = 0; v < obj->vertexCount; v++) {
				dp[v] = obj->positions[obj->vertices[v].position];
				ds[v] = obj->shades[obj->vertices[v].shade];
			}
		}
	}

	bound_buffer = 0;
	return true;
}


void MeshDeform::bindFrame(uint8_t fb_index)
{
	bound_buffer = fb_index % MESH_DEFORM_BUFFERS;
}


void MeshDeform::apply(uint8_t fb_index)
{
	if (slot_source == NULL || source == NULL) return;

	/* This frame's copy, not the model's buffer: the GPU is still reading
	   the one the previous frame was drawn from. */
	RenderPosition *pos   = position_buffer[fb_index % MESH_DEFORM_BUFFERS];
	RenderShade    *shade = shade_buffer[fb_index % MESH_DEFORM_BUFFERS];
	if (pos == NULL || shade == NULL) return;

	for (uint16_t slot = 0; slot < slot_count; slot++) {
		uint16_t index = slot_source[slot];
		if (index == MESH_DEFORM_UNBOUND) continue;

		meshDeform_quantize(source[index], scale, &pos[slot].x);

		if (source_normal)
			meshDeform_packNormal(source_normal[index], normal_sign, &shade[slot].nx);

		if (source_rgba) {
			shade[slot].r = source_rgba[index * 4 + 0];
			shade[slot].g = source_rgba[index * 4 + 1];
			shade[slot].b = source_rgba[index * 4 + 2];
			shade[slot].a = source_rgba[index * 4 + 3];
		}
	}
}


void MeshDeform::destroy()
{
	if (slot_source)   psyqo_free(slot_source);
	if (object_offset) psyqo_free(object_offset);
	if (vertex_map)    psyqo_free(vertex_map);

	for (int i = 0; i < MESH_DEFORM_BUFFERS; i++) {
		if (position_buffer[i]) psyqo_free(position_buffer[i]);
		if (shade_buffer[i])    psyqo_free(shade_buffer[i]);
	}

	*this = MeshDeform();
}
