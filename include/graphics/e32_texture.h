/*
	Textures live in VRAM, next to the framebuffers, and a primitive names
	one through its texture page and its offset inside that page. This hands
	out places for them: a shelf allocator over the free part of VRAM, filled
	once at load. Nothing is ever given back; a scene unload resets the whole
	shelf.
*/
#ifndef ENGINE_32_TEXTURE_H
#define ENGINE_32_TEXTURE_H

#include <stdint.h>

#include "psyqo/primitives/common.hh"


/* The GPU's texture window: the command that makes texel coordinates
   wrap inside one texture instead of running across the page. Sent in
   front of every textured primitive, since the ordering table interleaves
   materials. */
struct TextureWindow
{
	uint32_t command;
};

/* Where a texture landed: the page a primitive selects, the texel offset
   of the texture's corner inside that page, and the window that repeats
   it. */
struct TextureSlot
{
	psyqo::PrimPieces::TPageAttr tpage;
	uint8_t u;
	uint8_t v;
	TextureWindow window;
};


class Texture
{
public:

	/* Empties the shelf. The framebuffers keep the left of VRAM, the system
	   font the bottom right corner; everything between is ours. */
	static void init();

	/* Uploads a 15 bit image and says where it went. Sides are powers of
	   two, so the window can repeat them. False when VRAM is full. */
	static bool upload(const uint16_t *pixels, uint16_t width, uint16_t height, TextureSlot *slot);
};


#endif
