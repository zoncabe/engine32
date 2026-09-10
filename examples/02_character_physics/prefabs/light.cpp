/*
	One lamp over the middle of the room, plus enough ambient to keep the
	corners out of pitch black. The size is its reach: past 40 metres it
	lights nothing, which covers the room end to end.

	Three slots, shared between kinds. Declared in order and cut at the first
	empty one, so the two left over cost nothing.
*/
#include "scene3d/e32_lighting.h"


extern const LightDef light = {

	.ambient_color = {{60, 60, 70}},

	.source = {
		{ .type  = LIGHT_POINT,
		  .color = {{255, 245, 220}},
		  .point = { .position = { 0.0_fp, 0.0_fp, 12.0_fp }, .size = 40.0_fp } },
	},
};
