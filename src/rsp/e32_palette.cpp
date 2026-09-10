#include <assert.h>
#include <malloc.h>

#include "rsp/e32_palette.h"

#include "animation/e32_armature.h"
#include "physics/math/e32_matrix4.h"
#include "render/e32_render.h"


DEFINE_RSP_UCODE(rsp_e32_palette);

/* First command of the overlay, in declaration order. */
#define PALETTE_CMD_UPDATE 0x0

static uint32_t palette_overlay_id;
static bool     palette_ready = false;

/* The two matrices every bone of one mesh shares. The RSP reads them long
   after the call returns, so each mesh of each frame in flight needs a copy
   of its own that stays put, the same reason the renderer keeps a pool for
   its object matrices. Uncached, and the two have to stay adjacent: the
   ucode fetches both in a single transfer. */
typedef struct {
	mgfx_matrix_t view_model;
	mgfx_matrix_t view_projection_model;
} PaletteFrameMatrices;

static PaletteFrameMatrices *palette_pool;
static uint8_t               palette_pool_fb   = 0xFF;
static uint16_t              palette_pool_slot = 0;

static void palette_init(void)
{
	palette_overlay_id = rspq_overlay_register(&rsp_e32_palette);

	palette_pool = malloc_uncached(sizeof(PaletteFrameMatrices) * FB_COUNT * RENDER_MAX_3D_ELEMENTS);
	assert(palette_pool);

	palette_ready = true;
}

/* One slot per mesh and per frame buffer. The counter restarts on its own
   when the frame buffer changes, so the module needs no hook in the frame
   loop to reset it. */
static PaletteFrameMatrices *palette_slot(uint8_t fb_index)
{
	if (fb_index != palette_pool_fb) {
		palette_pool_fb   = fb_index;
		palette_pool_slot = 0;
	}

	assert(palette_pool_slot < RENDER_MAX_3D_ELEMENTS);
	return &palette_pool[fb_index * RENDER_MAX_3D_ELEMENTS + palette_pool_slot++];
}

void palette_update(Mesh *mesh, const Viewport *viewport, uint8_t fb_index)
{
	if (mesh->palette == NULL) return;

	if (!palette_ready) palette_init();

	const Armature *armature = mesh->skeleton;
	uint16_t bones = armature->skeletonRef->boneCount;

	/* Same preamble as the CPU version: view and view-projection over the
	   model matrix, once for the whole mesh. What is gone from here is the
	   loop that repeated a pair of products for every bone. */
	const Matrix4 *model = &mesh->matrix_buffer[fb_index];
	Matrix4 view_model, view_projection_model;
	matrix4_product(&view_model, &viewport->view, model);
	matrix4_product(&view_projection_model, &viewport->view_projection, model);

	PaletteFrameMatrices *frame = palette_slot(fb_index);
	matrix4_toFixed(&frame->view_model, &view_model);
	matrix4_toFixed(&frame->view_projection_model, &view_projection_model);

	/* The bone matrices the RSP multiplies are the fixed point ones the
	   armature already writes on every update, which are the same values
	   the CPU path read in floating point. They are buffered for exactly
	   this reason: the buffer handed over here is not rewritten until the
	   RSP has long finished with it. */
	const mgfx_matrix_t *bone_matrices = armature_getMatrices(armature);
	mgfx_matrices_t     *entry         = mesh->palette + fb_index * bones;

	rspq_write(palette_overlay_id, PALETTE_CMD_UPDATE,
		bones,
		PhysicalAddr(frame),
		PhysicalAddr(bone_matrices),
		PhysicalAddr(entry));
}
