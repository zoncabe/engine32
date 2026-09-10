#ifndef ENGINE_32_LIGHTING_H
#define ENGINE_32_LIGHTING_H

#include "psyqo/primitives/common.hh"

#include "physics/math/e32_math.h"

/* Three slots (the GTE's light matrix has three rows) and a light takes one
   whatever its kind, so the split between directional and point is the
   scene's to make. */
#define LIGHT_COUNT 3


typedef enum {

	/* An empty slot. Zero on purpose: the set walks the table in order and
	   stops at the first one, so a scene pays only for what it declared. */
	LIGHT_NONE,

	LIGHT_DIRECTIONAL,
	LIGHT_POINT,

} LightType;


struct LightSource
{
	LightType    type;
	psyqo::Color color;

	union {
		/* Where the light comes from; normalised by the init. */
		struct { Vector3 direction; } directional{};

		/* Where it stands and how far it carries. */
		struct { Vector3 position; Fixed size; } point;
	};
};


struct LightDef
{
	psyqo::Color ambient_color;
	LightSource  source[LIGHT_COUNT];
};

typedef LightDef Light;


class Lighting
{
public:

	static Light *get();

	/* Copies the scene's declaration into the live lights. */
	static void init(const LightDef *def);

	/* Writes the ambient term to the GTE. Once per frame. */
	static void setAmbient(const Light *light);

	/* Loads the GTE's light and color matrices for an object at this
	   placement, stopping at the first empty slot. The GTE lights in model
	   space, so the directions go through the inverse of the object's
	   rotation, and a point light becomes the direction from the object to
	   it, dimmed by distance. */
	static void set(const Light *light, const Transform *model);
};

#endif
