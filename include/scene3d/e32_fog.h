#ifndef ENGINE_32_FOG_H
#define ENGINE_32_FOG_H

#include <stdbool.h>

#include "psyqo/primitives/common.hh"

#include "physics/math/e32_fixed.h"

/* Distance fog: the GTE's depth cueing. Each vertex color fades toward the
   fog color by its depth, between near and far, in meters along the view
   axis, inside the camera planes. */

struct FogDef
{
	psyqo::Color color;
	Fixed near;
	Fixed far;
	bool  enabled;
};

struct Fog
{
	psyqo::Color color;
	Fixed near;
	Fixed far;
	bool  enabled;
};


class FogState
{
public:

	static Fog *get();

	static void init(const FogDef *def);

	/* Loads the depth cue: the far color, and the two coefficients that map
	   the GTE's perspective factor at near to no fog and at far to all fog.
	   Disabled fog leaves the coefficients at zero, so the lighting kernel
	   with depth cue reads the same as the one without. */
	static void set(const Fog *fog, int projection);
};

#endif
