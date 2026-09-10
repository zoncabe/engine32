/*
	The model file, as the importer writes it and the engine reads it. Plain
	C so the host tool and the console share this one header.

	Everything is little endian and 4 byte aligned. Offsets are from the
	start of the file. Vertices are the console's: positions in 10.6 (one
	meter is 64), normals in 4.12, colors 0..255, texture coordinates in
	texels of the object's texture. Nothing is converted at load.
*/
#ifndef ENGINE_32_MODEL_FORMAT_H
#define ENGINE_32_MODEL_FORMAT_H

#include <stdint.h>

#define MODEL_MAGIC   "E32M"
#define MODEL_VERSION 2

/* Blender meters to the GTE's integer: the 6 fractional bits of 10.6.
   From the CPU's 20.12 that is a shift of 12 - 6 bits. */
#define MODEL_UNITS_SHIFT     6
#define MODEL_UNITS_PER_METER (1 << MODEL_UNITS_SHIFT)
#define MODEL_CPU_TO_UNITS    (12 - MODEL_UNITS_SHIFT)


typedef struct ModelFileHeader {
	char     magic[4];
	uint32_t version;
	uint16_t object_count;
	uint16_t material_count;
	int16_t  aabb_min[3];
	int16_t  aabb_max[3];
	uint32_t objects;     /* ModelFileObject[object_count] */
	uint32_t materials;   /* ModelFileMaterial[material_count] */
	uint32_t strings;     /* names, zero terminated, referenced by offset */

	/* Optional, zero when the file has none. */
	uint32_t skeleton;        /* ModelFileSkeleton */
	uint32_t animations;      /* ModelFileAnimation[animation_count], each followed by its channels */
	uint32_t bvh;             /* ModelFileBvh */
	uint16_t animation_count;
	uint16_t _pad;
} ModelFileHeader;

typedef struct ModelFileVertex {
	int16_t  x, y, z, _pad0;
	int16_t  nx, ny, nz, _pad1;
	uint8_t  r, g, b, a;
	uint8_t  u, v;
	uint16_t _pad2;
} ModelFileVertex;

/* A triangle or a quad: four vertex indices, the last one MODEL_NO_VERTEX
   for a triangle. Quads are the GPU's native primitive; the exporter keeps
   them and the render draws them as one. Counter clockwise seen from the
   outside, the quad in contour order. */
#define MODEL_NO_VERTEX 0xFFFF

typedef struct ModelFileFace {
	uint16_t v[4];
} ModelFileFace;

typedef struct ModelFileObject {
	uint32_t name;
	uint16_t material;
	uint16_t vertex_count;
	uint16_t face_count;
	uint16_t _pad;
	int16_t  aabb_min[3];
	int16_t  aabb_max[3];
	uint32_t vertices;      /* ModelFileVertex[vertex_count] */
	uint32_t faces;         /* ModelFileFace[face_count] */
	uint32_t bone_indices;  /* uint8_t[vertex_count]; zero when the object is not skinned */
} ModelFileObject;

typedef struct ModelFileMaterial {
	uint32_t name;
	uint16_t width;         /* 0: no texture, the color alone */
	uint16_t height;
	uint32_t pixels;        /* uint16_t[width * height], 15 bit BGR with the mask bit */
	uint8_t  color[4];      /* base color, multiplies the texture; alpha under 255 draws it semi transparent */

	/* The emission map (map_Ke): a second texture the GPU adds over the
	   first, drawn as another pass of the same polygons. Same size as the
	   base texture, so the vertices' coordinates serve both. 0: none. */
	uint16_t glow_width;
	uint16_t glow_height;
	uint32_t glow_pixels;   /* uint16_t[glow_width * glow_height] */
} ModelFileMaterial;


/* Skeleton: the bones in the model's rest pose. Scalars are 20.12 (one is
   4096); the rotation is a unit quaternion x, y, z, w. */
typedef struct ModelFileBone {
	uint32_t name;
	uint16_t parent;        /* index of the parent bone; the root points at itself */
	uint16_t depth;
	int32_t  scale[3];
	int32_t  rotation[4];
	int32_t  position[3];
} ModelFileBone;

typedef struct ModelFileSkeleton {
	uint16_t bone_count;
	uint16_t _pad;
	uint32_t bones;         /* ModelFileBone[bone_count] */
} ModelFileSkeleton;


#define ANIM_TARGET_TRANSLATION 0
#define ANIM_TARGET_SCALE_XYZ   1
#define ANIM_TARGET_SCALE_S     2
#define ANIM_TARGET_ROTATION    3

/* One animated attribute of one bone. The keyframes are quantized in the
   animation's own file; scale and offset bring them back, in 20.12. */
typedef struct ModelFileAnimationChannel {
	uint16_t target;        /* bone index */
	uint8_t  target_type;   /* ANIM_TARGET_* */
	uint8_t  attribute;     /* component, for scalar targets */
	int32_t  quant_scale;
	int32_t  quant_offset;
} ModelFileAnimationChannel;

typedef struct ModelFileAnimation {
	uint32_t name;
	int32_t  duration;      /* seconds, 20.12 */
	uint32_t keyframe_count;
	uint16_t channels_quat;
	uint16_t channels_scalar;
	uint32_t file_path;     /* the keyframe stream, by path */
	uint32_t channels;      /* ModelFileAnimationChannel[channels_quat + channels_scalar] */
} ModelFileAnimation;


/* Bounding volume hierarchy over the objects, for culling. A node with a
   data count of zero is inner and 'value' >> 4 is the offset to its two
   children; otherwise 'value' >> 4 is the offset into the data and the low
   four bits how many object indices it lists. */
typedef struct ModelFileBvhNode {
	int16_t  aabb_min[3];
	int16_t  aabb_max[3];
	uint16_t value;
	uint16_t _pad;
} ModelFileBvhNode;

typedef struct ModelFileBvh {
	uint16_t node_count;
	uint16_t data_count;
	uint32_t nodes;         /* ModelFileBvhNode[node_count] */
	uint32_t data;          /* uint16_t[data_count], object indices */
} ModelFileBvh;


#endif
