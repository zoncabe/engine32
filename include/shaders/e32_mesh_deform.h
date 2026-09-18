/*
	Drives a model's vertices from an external set of points.

	Binds every vertex slot of a model to the source point sitting at the same
	rest position, then rewrites the buffer from those points on demand. The
	source is a plain array, so anything that moves points can drive a mesh:
	a collision mesh under simulation, a morph target, a spline.

	Binding by position rather than by index is what makes this work: the
	importer splits vertices per UV seam and material border, so one source
	point usually lands on several slots and never on a predictable one.

	The model's own vertices are read only, in the executable, so the deform
	keeps a copy of every object's vertices back to back and the render
	reads that copy instead.
*/
#ifndef ENGINE_32_MESH_DEFORM_H
#define ENGINE_32_MESH_DEFORM_H

#include <stdbool.h>
#include <stdint.h>
#include "animation/e32_model.h"

#include "viewport/e32_viewport.h"      /* FB_COUNT */
#include "physics/math/e32_vector3.h"


#define MESH_DEFORM_UNBOUND 0xFFFF

/* One vertex buffer per framebuffer, same reason the matrices are buffered. */
#define MESH_DEFORM_BUFFERS FB_COUNT

struct MeshDeform
{
	uint16_t      *slot_source;   /* source index per vertex slot, or MESH_DEFORM_UNBOUND */
	uint16_t       slot_count;    /* vertex slots in the model, every object together */
	uint16_t       bound_count;   /* slots that found a source point */
	Fixed          scale;         /* source units -> the vertices' units */
	Model         *model;
	const Vector3 *source;        /* the points driving the mesh; kept, not owned */

	/* Where each object's vertices start inside a buffer. */
	uint32_t      *object_offset;

	/* The GPU runs behind the CPU, so writing one buffer every frame lets it
	   read vertices half-overwritten. One copy per framebuffer, bound at
	   draw time, keeps the frame being drawn intact.

	   The model shares a position and a shade between the corners that agree
	   on them; a driven slot does not, since the source moves each one on its
	   own and may carry its own normal. So the deform gives every slot a
	   position and a shade of its own, and its corners index themselves. That
	   table never changes, so there is one of it. */
	RenderPosition *position_buffer[MESH_DEFORM_BUFFERS];
	RenderShade    *shade_buffer[MESH_DEFORM_BUFFERS];
	RenderVertex   *vertex_map;
	uint8_t         bound_buffer;   /* the copy the render reads this frame */

	/* Optional: one normal per source point, in the same order. The source
	   decides its own winding, which may run against the model's, so the
	   binding measures the two at rest and keeps the sign that agrees. */
	const Vector3 *source_normal;
	Fixed          normal_sign;

	/* Optional: one RGBA per source point, written into the vertex colors.
	   NULL leaves the model's own colors alone. */
	const uint8_t *source_rgba;


	/* Matches the model's rest pose against the source points; both must
	   still be at rest. The source array is kept, so it has to outlive the
	   binding and keep its order. Pass source_normal to drive the shading
	   too, or NULL to leave the model's own normals alone. Returns false
	   only if the binding could not be allocated; a partial match counts as
	   success and leaves the unmatched slots frozen. */
	bool bind(Model *model, const Vector3 *source, const Vector3 *source_normal,
	          uint16_t source_count, Fixed scale);

	/* Writes the current source positions, and normals if bound, into this
	   frame's vertex buffer. */
	void apply(uint8_t fb_index);

	/* Selects this frame's buffer for the draw. Runs before the mesh is
	   drawn. */
	void bindFrame(uint8_t fb_index);

	/* This frame's vertices of one object, from the bound buffer. */
	const RenderVertex *vertices(uint32_t object) const
	{
		return vertex_map + object_offset[object];
	}

	const RenderPosition *positions(uint32_t object) const
	{
		return position_buffer[bound_buffer] + object_offset[object];
	}

	const RenderShade *shades(uint32_t object) const
	{
		return shade_buffer[bound_buffer] + object_offset[object];
	}

	void destroy();
};


#endif
