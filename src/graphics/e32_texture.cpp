#include "graphics/e32_texture.h"

#include "system/e32_system.h"
#include "viewport/e32_viewport.h"


/* VRAM is 1024x512. The two framebuffers take the first 320 columns; the
   system font sits at 960,464 and below. The shelf runs over the rest. */
static const int VRAM_LEFT   = SCREEN_WIDTH;
static const int VRAM_RIGHT  = 960;
static const int VRAM_BOTTOM = 512;

/* A 15 bit texture page is 256 texels wide and tall, and pages start every
   64 columns and every 256 rows. */
static const int PAGE_COLUMNS = 64;
static const int PAGE_SIZE    = 256;

static int shelf_x;
static int shelf_y;
static int shelf_height;


void Texture::init()
{
	shelf_x      = VRAM_LEFT;
	shelf_y      = 0;
	shelf_height = 0;
}


/* Column and row of the page that holds x,y, and the offset inside it. A
   texture must not straddle a page, so a placement that would is pushed to
   the next page boundary before it is accepted. */
static bool fits(int x, int y, int width, int height)
{
	int u = x - (x / PAGE_COLUMNS) * PAGE_COLUMNS;
	int v = y - (y / PAGE_SIZE) * PAGE_SIZE;
	return u + width <= PAGE_SIZE && v + height <= PAGE_SIZE;
}


bool Texture::upload(const uint16_t *pixels, uint16_t width, uint16_t height, TextureSlot *slot)
{
	if (width == 0 || height == 0 || width > PAGE_SIZE || height > PAGE_SIZE) return false;

	for (;;) {
		/* The window masks the coordinate bits below the texture's size,
		   so the texture has to sit at a multiple of its own size inside
		   the page: aligned on both axes. */
		shelf_x = (shelf_x + width - 1) & ~(width - 1);
		int aligned_y = (shelf_y + height - 1) & ~(height - 1);
		if (aligned_y != shelf_y) {
			shelf_height += aligned_y - shelf_y;
			shelf_y = aligned_y;
		}

		if (shelf_x + width > VRAM_RIGHT) {
			/* next shelf */
			shelf_x      = VRAM_LEFT;
			shelf_y     += shelf_height;
			shelf_height = 0;
			if (shelf_y + height > VRAM_BOTTOM) return false;
		}

		if (fits(shelf_x, shelf_y, width, height)) break;

		/* Straddles a page: skip to the next page column, or the next page
		   row when it is the height that does not fit. */
		int v = shelf_y - (shelf_y / PAGE_SIZE) * PAGE_SIZE;
		if (v + height > PAGE_SIZE) {
			shelf_x      = VRAM_RIGHT;   /* forces a new shelf */
			shelf_height = PAGE_SIZE - v;
			continue;
		}
		shelf_x = (shelf_x / PAGE_COLUMNS + 1) * PAGE_COLUMNS;
	}

	int x = shelf_x;
	int y = shelf_y;

	psyqo::Rect region = {
		.pos  = {{.x = (int16_t)x, .y = (int16_t)y}},
		.size = {{.w = (int16_t)width, .h = (int16_t)height}},
	};
	System::get().getGpu().uploadToVRAM(pixels, region);

	slot->tpage = psyqo::PrimPieces::TPageAttr();
	/* The blend mode only acts on primitives flagged semi transparent:
	   half and half is what a transparent material wants, and it costs
	   the opaque ones nothing. */
	slot->tpage.setPageX((uint8_t)(x / PAGE_COLUMNS))
	           .setPageY((uint8_t)(y / PAGE_SIZE))
	           .set(psyqo::Prim::TPageAttr::Tex16Bits)
	           .set(psyqo::Prim::TPageAttr::HalfBackAndHalfFront)
	           .setDithering(true);
	slot->u = (uint8_t)(x - (x / PAGE_COLUMNS) * PAGE_COLUMNS);
	slot->v = (uint8_t)(y - (y / PAGE_SIZE) * PAGE_SIZE);

	/* GP0 E2, in units of 8 texels: the mask clears the coordinate bits
	   at and above the texture's size, the offset puts the texture's
	   corner back in their place. */
	uint32_t mask_x   = (32 - width  / 8) & 31;
	uint32_t mask_y   = (32 - height / 8) & 31;
	uint32_t offset_x = (uint32_t)slot->u / 8;
	uint32_t offset_y = (uint32_t)slot->v / 8;
	slot->window.command = 0xE2000000 | mask_x | (mask_y << 5) | (offset_x << 10) | (offset_y << 15);

	shelf_x += width;
	if (height > shelf_height) shelf_height = height;

	return true;
}
