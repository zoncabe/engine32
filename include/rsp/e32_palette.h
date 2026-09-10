#ifndef ENGINE_32_PALETTE_H
#define ENGINE_32_PALETTE_H

#include <libdragon.h>

#include "graphics/e32_mesh.h"
#include "viewport/e32_viewport.h"


/*
 * Same call as 'mesh_updatePalette', same bytes in the same buffer: the bone
 * palette of a skinned mesh for one frame buffer. What changes is where the
 * work happens. The CPU only composes the two matrices every bone of the mesh
 * shares; the two products per bone, the normal matrix and the conversion to
 * fixed point all move to the RSP, which writes the palette itself.
 *
 * Magma is untouched: it keeps loading its matrices uniform out of that same
 * palette, one entry per bone, exactly as before.
 *
 * The overlay registers itself on the first call, so nothing has to be set up
 * at boot.
 */
void palette_update(Mesh *mesh, const Viewport *viewport, uint8_t fb_index);

#endif
