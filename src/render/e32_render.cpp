#include <stdint.h>

#include "psyqo/alloc.h"
#include "psyqo/kernel.hh"
#include "common/hardware/counters.h"
#include "psyqo/fragments.hh"
#include "psyqo/gte-kernels.hh"
#include "psyqo/gte-registers.hh"
#include "psyqo/ordering-table.hh"
#include "psyqo/primitives.hh"

#include "physics/math/e32_math.h"
#include "animation/e32_armature.h"

#include "scene3d/e32_lighting.h"
#include "scene3d/e32_fog.h"
#include "viewport/e32_viewport.h"
#include "graphics/e32_mesh.h"
#include "render/e32_render.h"
#include "debug/e32_debug.h"
#include "time/e32_time.h"
#include "system/e32_system.h"
#include "scene3d/e32_scene3d.h"
#ifdef ENGINE_32_SCENE2D
#include "scene2d/e32_scene2d.h"
#endif
#ifdef ENGINE_32_PARTICLES
#include "particles/e32_particles.h"
#endif

#include "game/e32_game.h"

using namespace psyqo::GTE;


/* The frame's draw list. Filled by the scenes and consumed here, every
   frame; nobody outside sees it. */
static RenderContext render_context;


Render &Render::get()
{
	static Render render;
	return render;
}


void Render::init()
{
	/* The screen center, 16.16. */
	write<Register::OFX, Unsafe>((uint32_t)(SCREEN_WIDTH  / 2) << 16);
	write<Register::OFY, Safe>((uint32_t)(SCREEN_HEIGHT / 2) << 16);

	/* What AVSZ3 and AVSZ4 multiply the sum of the depths by, 4.12: a third
	   and a quarter, so the command leaves the average depth in OTZ. That is
	   the table's bucket, computed by the coprocessor in five cycles instead
	   of by the CPU over values read back from memory. */
	write<Register::ZSF3, Unsafe>((uint32_t)(4096 / 3));
	write<Register::ZSF4, Safe>((uint32_t)(4096 / 4));

	/* No mask test: the BIOS shell leaves the GPU's mask state behind, and
	   with the test on, nothing lands over a pixel whose mask bit is set. */
	psyqo::Prim::MaskControl mask(psyqo::Prim::MaskControl::Set::FromSource,
	                              psyqo::Prim::MaskControl::Test::No);
	System::get().getGpu().sendPrimitive(mask);

	/* Profiling: root counter 2 free running on the system clock over 8,
	   a tick every 0.24 us, wrapping every 15 ms. */
	COUNTERS[2].mode = TM_CLK_DIV8;

	for (int i = 0; i < FB_COUNT; i++) {
		_arena[i]        = NULL;
		_arena_size[i]   = 0;
		_arena_needed[i] = 0;
		_extra[i]        = NULL;

		/* GP0 E2 with no mask and no offset: the whole page */
		_window_reset[i].primitive.command = 0xE2000000;
	}
}


/* Room past the arena, in blocks of this many bytes. */
#define RENDER_ARENA_BLOCK 16384

void *Render::extra(uint32_t bytes)
{
	ArenaBlock *block = _extra[_parity];

	if (!block || block->used + bytes > block->size) {
		block = (ArenaBlock *)psyqo_malloc(sizeof(ArenaBlock) + RENDER_ARENA_BLOCK);
		psyqo::Kernel::assert(block != NULL, "render: out of memory");
		block->next = _extra[_parity];
		block->size = RENDER_ARENA_BLOCK;
		block->used = 0;
		_extra[_parity] = block;
	}

	void *p = (uint8_t *)(block + 1) + block->used;
	block->used += bytes;
	return p;
}


/* Sizes this parity's arena: every face of every mesh drawn as a textured
   quad, or what the last frames of this parity asked, whichever is more.
   Only grows. */
void Render::reserve(const RenderContext *ctx)
{
	uint32_t faces = 0;

	for (int i = 0; i < ctx->object_count; i++) {
		const Model *model = ctx->object[i].mesh->model;
		for (uint32_t o = 0; o < model->objectCount; o++)
			faces += model->objects[o].faceCount;
	}

	uint32_t bytes = faces * sizeof(TexturedQuadFragment);
	if (bytes < _arena_needed[_parity]) bytes = _arena_needed[_parity];
	if (bytes <= _arena_size[_parity]) return;

	if (_arena[_parity]) psyqo_free(_arena[_parity]);
	_arena[_parity] = (uint8_t *)psyqo_malloc(bytes);
	psyqo::Kernel::assert(_arena[_parity] != NULL, "render: out of memory");
	_arena_size[_parity] = bytes;
}


void Render::initContext(RenderContext *ctx)
{
	/* Only the counts matter: entries are fully written before being read. */
	ctx->element_count = 0;
	ctx->section_count = 0;
	ctx->object_count  = 0;
}


void Render::start(Viewport *viewport)
{
	viewport->fb_index = (viewport->fb_index + 1) % FB_COUNT;

	psyqo::GPU &gpu = System::get().getGpu();
	_parity = gpu.getParity();
	_table  = &_ot[_parity];
	_view   = &viewport->view;

	/* The GPU is done with this parity's previous frame: the blocks it
	   overflowed into go, and the arena grows to what it asked. */
	while (_extra[_parity]) {
		ArenaBlock *next = _extra[_parity]->next;
		psyqo_free(_extra[_parity]);
		_extra[_parity] = next;
	}
	reserve(&render_context);
	_arena_used = 0;
	_prims      = 0;

	/* Geometry fades toward the fog color, so the background must be it. */
	const Fog *fog = FogState::get();
	psyqo::Color bg = fog->enabled ? fog->color : viewport->clear_color;

	gpu.getNextClear(_clear[_parity].primitive, bg);
	gpu.chain(_clear[_parity]);
}

void Render::end()
{
	if (_arena_used > _arena_needed[_parity]) _arena_needed[_parity] = _arena_used;

	psyqo::GPU &gpu = System::get().getGpu();
	gpu.chain(*_table);
	gpu.chain(_window_reset[_parity]);
}


void Render::frame()
{
	RenderContext *ctx = &render_context;
	Viewport *viewport = &Viewport::get();

	/* the hardware's horizontal blank counter, 16 bits */
	uint32_t began = COUNTERS[1].value;

#ifdef E32_TRACE
	uint16_t t0 = (uint16_t)COUNTERS[2].value;
#endif

	initContext(ctx);
	scene3d_setRenderContext(scene3d_get(), ctx, viewport);
#ifdef ENGINE_32_SCENE2D
	scene2d_setRenderContext(scene2d_get(), ctx);
#endif

	start(viewport);

	if (ctx->object_count > 0) {
		/* The lens, and the depth limits in the models' units. */
		write<Register::H, Safe>((uint32_t)viewport->projection);

		uint32_t near = (uint32_t)(viewport->camera.near_clipping.raw() >> MODEL_CPU_TO_UNITS);
		uint32_t limit = (uint32_t)viewport->projection / 2 + 1;
		_near = near > limit ? near : limit;
		_far  = (uint32_t)(viewport->camera.far_clipping.raw() >> MODEL_CPU_TO_UNITS);
		_projection = viewport->projection;

		const Fog *fog = FogState::get();
		_fog = fog->enabled;
		FogState::set(fog, viewport->projection);
		Lighting::setAmbient(Lighting::get());

		mesh_profile_faces = 0;
		mesh_profile_transform = 0;
		mesh_profile_emit = 0;
		mesh_profile_pieces = 0;
		mesh_profile_cuts = 0;
		mesh_profile_walked = 0;
		mesh_profile_clip = 0;
		mesh_profile_prim = 0;
		mesh_profile_direct = 0;
		mesh_profile_cut = 0;
		mesh_profile_lerp = 0;
		mesh_profile_project = 0;
		mesh_profile_level = 0;
		mesh_profile_object = 0;
		mesh_profile_draw = 0;
		mesh_profile_prep = 0;
		mesh_profile_cut2 = 0;
		mesh_profile_slow = 0;
		mesh_profile_back = 0;
		mesh_profile_far = 0;
		mesh_profile_whole = 0;
		mesh_profile_sub = 0;
		mesh_profile_cover = 0;
		mesh_profile_light = 0;
		mesh_profile_elements = 0;
#ifdef E32_TRACE
		debug_profile_setup += (uint16_t)((uint16_t)COUNTERS[2].value - t0);
#endif
		uint32_t drawing = COUNTERS[1].value;

		/* Drawing the list backwards was tried, on the grounds that the table
		   hangs each new primitive at the front of its bucket's chain, so the
		   first element inserted is drawn last and covers the rest of what
		   lands in the same bucket. It changed nothing on screen: the
		   polygons that fight are not landing in the same bucket. */
		for (int i = 0; i < ctx->object_count; i++) {
			ctx->object[i].mesh->draw(&ctx->object[i]);
#ifdef E32_OTZ_TRACE
			debug_traceOtz(i, ctx->object[i].mesh->subdivide,
			               mesh_otz_min, mesh_otz_max);
#endif
		}
#ifdef E32_OTZ_TRACE
		debug_traceOtzEnd();
#endif
		_draw_hblanks = (COUNTERS[1].value - drawing) & 0xFFFF;
	}

#ifdef E32_TRACE
	uint16_t t1 = (uint16_t)COUNTERS[2].value;
#endif

#ifdef ENGINE_32_PARTICLES
	particles_draw();
#endif

	end();

#ifdef E32_TRACE
	debug_profile_end += (uint16_t)((uint16_t)COUNTERS[2].value - t1);
#endif

	_hblanks = (COUNTERS[1].value - began) & 0xFFFF;

	debugUI_draw();

#ifdef E32_TRACE
	debug_trace();
#endif
}
