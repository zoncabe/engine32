/*
	The frame: every triangle of every visible mesh part through the GTE and
	into the ordering table, then the table to the GPU. There is no depth
	buffer; the table sorts triangles by their average depth, far to near,
	and the GPU paints them in that order.

	Two of everything, by frame parity: the GPU reads one frame's table and
	primitives while the CPU writes the next.
*/
#ifndef ENGINE_32_RENDER_H
#define ENGINE_32_RENDER_H

#include <stdbool.h>
#include <stdint.h>

#include "psyqo/fragments.hh"
#include "psyqo/ordering-table.hh"
#include "psyqo/primitives.hh"

#include "physics/math/e32_math.h"
#include "animation/e32_model.h"
#include "animation/e32_armature.h"
#include "physics/math/e32_vector2.h"
#include "physics/math/e32_vector3.h"
#include "viewport/e32_viewport.h"

#define RENDER_MAX_2D_ELEMENTS   64
/* One entry per visible mesh part, not per entity: a skinned character alone
   contributes several, so this has to clear the scene's entity budget. */
#define RENDER_MAX_3D_ELEMENTS    32
#define RENDER_MAX_SECTIONS        8

/* Depth buckets. Each is one GTE unit deep, so the table reaches 64
   meters; anything past it is not drawn. */
#define RENDER_OT_SIZE 4096

struct Scene3D;
struct Scene2D;
struct Graphic;
struct Mesh;
struct ModelDrawConf;

struct RenderTransform
{
	Vector3     position;
	EulerAngles rotation;
	Vector3     scale;

	void init()
	{
		position = Vector3::zero();
		rotation = {};
		scale    = { 1.0_fp, 1.0_fp, 1.0_fp };
	}
};

struct Element2D
{
	const Graphic *graphic;

	Vector2 position;
	Vector2 scale;
	Angle   rotation;
};

/* One part of one mesh to draw: the objects the part covers, the placement
   they share and, for a skinned mesh, the bone palette. */
struct Element3D
{
	const Mesh      *mesh;
	uint8_t          part;      /* which of the mesh's parts; 0 with no parts recorded */
	const Transform *matrix;    /* model transform; NULL = identity */
	const Armature  *skeleton;
	const Transform *palette;   /* view * model * bone per bone; NULL when not skinned */
	ModelDrawConf   *conf;      /* optional, object path only: per-frame material hooks */
};

struct RenderSection
{
	uint8_t element_start;
	uint8_t element_count;

	bool    has_scissor;
	int16_t scissor_x;
	int16_t scissor_y;
	int16_t scissor_w;
	int16_t scissor_h;
};

struct RenderContext
{
	Element2D     element[RENDER_MAX_2D_ELEMENTS];
	uint8_t       element_count;

	RenderSection section[RENDER_MAX_SECTIONS];
	uint8_t       section_count;

	Element3D     object[RENDER_MAX_3D_ELEMENTS];
	uint8_t       object_count;
};


class Render
{
public:

	static Render &get();

	/* Writes the GTE's screen center. Called once after Viewport::init. */
	void init();

	void initContext(RenderContext *ctx);

	/* Draws the frame: hands its context to each scene to fill, then paints
	   it. A scene that is not loaded pushes nothing, so whatever is up is
	   what shows. Reaches the scenes and the viewport itself. */
	void frame();


	/* What the meshes draw into, valid while the frame runs. */

	psyqo::OrderingTable<RENDER_OT_SIZE> *table() const { return _table; }

	const Transform *view() const { return _view; }

	/* Depth limits in the models' units. The GTE stops projecting when a
	   vertex comes closer than half the projection distance; the near plane
	   is at least that. */
	uint32_t near() const { return _near; }
	uint32_t far()  const { return _far; }
	bool     fog()  const { return _fog; }

	/* The projection distance in pixels, what the GTE holds in H. */
	int32_t projection() const { return _projection; }

	/* Debug: quads drawn as their outline, a polyline of one color, in
	   place of the textured polygon. Triangles draw as they are. */
	bool wireframe() const { return _wireframe; }
	void setWireframe(bool on) { _wireframe = on; }

	/* One fragment out of the frame's arena, its GPU command word built.
	   NULL once the arena is full: the rest of the frame is not drawn, and
	   the arena grows to what was asked the next time this parity draws. */
	template <typename F>
	F *alloc()
	{
		uint32_t at = _arena_used;
		_arena_used += sizeof(F);
		_prims++;
		if (_arena_used <= _arena_size[_parity]) return new (_arena[_parity] + at) F();
		return new (extra(sizeof(F))) F();
	}

	/* The last frame's cost: primitives asked for, the CPU time of the
	   whole frame and of the meshes' drawing alone, in horizontal blanks
	   (about 64 us each; a 60 Hz frame is 262). */
	uint32_t prims()       const { return _prims; }
	uint32_t hblanks()     const { return _hblanks; }
	uint32_t drawHblanks() const { return _draw_hblanks; }

private:

	/* One table and arena per frame in flight: the GPU reads a frame's
	   while the CPU fills the other's. The arena holds the frame's
	   primitives carved in order, since clipping and subdivision make their
	   count and kinds vary; it starts sized to the frame's faces and grows
	   to what the frames ask, when the frame starts, once the GPU is done
	   with that parity's. */
	psyqo::OrderingTable<RENDER_OT_SIZE> _ot[FB_COUNT];
	psyqo::Fragments::SimpleFragment<psyqo::Prim::FastFill> _clear[FB_COUNT];

	/* The texture window is GPU state: the last polygon's would stay on
	   for whatever draws after the table (debug text, 2D), so the frame
	   ends by putting the whole page back. */
	psyqo::Fragments::SimpleFragment<TextureWindow> _window_reset[FB_COUNT];

	uint8_t  *_arena[FB_COUNT];
	uint32_t  _arena_size[FB_COUNT];
	uint32_t  _arena_used;               /* bytes asked this frame, taken or not */
	uint32_t  _prims;
	uint32_t  _hblanks;
	uint32_t  _draw_hblanks;

	/* Blocks taken when a frame asks more than its arena holds, so nothing
	   is dropped: kept until this parity draws again, by which time the
	   arena has grown to what was asked. */
	struct ArenaBlock
	{
		ArenaBlock *next;
		uint32_t    size;
		uint32_t    used;
	};
	ArenaBlock *_extra[FB_COUNT];

	void *extra(uint32_t bytes);
	uint32_t  _arena_needed[FB_COUNT];   /* the most a frame of this parity asked */

	psyqo::OrderingTable<RENDER_OT_SIZE> *_table;
	int _parity;

	const Transform *_view;
	uint32_t _near;
	uint32_t _far;
	int32_t  _projection;
	bool     _fog;
	bool     _wireframe;

	void reserve(const RenderContext *ctx);
	void start(Viewport *viewport);
	void end();
};


#endif
