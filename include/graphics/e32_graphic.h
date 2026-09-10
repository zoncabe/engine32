/*
	What a 2D entity draws with, the way Mesh is what a 3D entity draws
	with: a rectangle, a sprite or a text, and whether it shows. Where it is
	drawn, how big and how turned is the entity's, not here.
*/
#ifndef ENGINE_32_GRAPHIC_H
#define ENGINE_32_GRAPHIC_H

#include <stdbool.h>
#include <stdint.h>

#include "graphics/e32_shapes.h"
#include "graphics/e32_sprites.h"
#include "graphics/e32_font.h"


typedef enum {

	GRAPHIC_RECTANGLE,
	GRAPHIC_SPRITE,
	GRAPHIC_TEXT,

} GraphicType;

typedef struct {

	GraphicType type;

	union {
		Rectangle rectangle;
		Sprite    sprite;
		Text      text;
	};

	uint8_t transparency;
	bool    is_hidden;

} Graphic;

#endif
