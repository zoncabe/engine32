#ifndef ENGINE_32_SCENE2D_H
#define ENGINE_32_SCENE2D_H

#include "render/e32_render.h"
#include "entity/e32_entity2d.h"
#include "prefab/e32_prefab2d.h"
#include "character2d/e32_character2d.h"

#define SCENE2D_MAX_LAYER      8
#define SCENE2D_MAX_ENTITY    64
#define SCENE2D_MAX_CHARACTER  4
#define SCENE2D_MAX_FONT       8


/* One prefab placed in a layer. The load turns it into an entity of the
   live scene, in placement order. A zero scale means identity. */
typedef struct Scene2DPrefab {

	const Prefab2D *prefab;
	Vector2         position;
	Vector2         scale;
	float           rotation;

} Scene2DPrefab;

/* A layer groups what it draws under one order and one scissor, the
   CanvasLayer of Godot. */
typedef struct Scene2DLayer {

	const Scene2DPrefab *prefab;
	uint8_t              prefab_count;

	bool               has_scissor;
	float              scissor_x;
	float              scissor_y;
	float              scissor_w;
	float              scissor_h;

} Scene2DLayer;

typedef struct Scene2DDef {

	const Scene2DLayer *layer;
	uint8_t             layer_count;

} Scene2DDef;


/* The live scene: the entities flat, in placement order, with where each
   layer's begin. */
typedef struct Scene2D {

	const Scene2DDef *def;

	Entity2D *entity[SCENE2D_MAX_ENTITY];
	uint8_t   entity_count;

	Character2D *character[SCENE2D_MAX_CHARACTER];
	uint8_t      character2d_count;

	/* The font ids its texts name, each loaded once for the scene. */
	uint8_t   font[SCENE2D_MAX_FONT];
	uint8_t   font_count;

	uint8_t   layer_start[SCENE2D_MAX_LAYER];

} Scene2D;


Scene2D *scene2d_get(void);

void scene2d_load(const Scene2DDef *def);
void scene2d_unload(void);

/* Advances every character's animation and carries the frame to what
   draws it. */
void scene2d_updateCharacters(float dt);

/* A layer's entity by its placement index in the definition. */
Entity2D *scene2d_getEntity(Scene2D *scene2d, uint8_t layer, uint8_t prefab);

Character2D *scene2d_getCharacter2D(uint8_t index);

/* Pushes one section per layer into the frame's context, with the layer's
   scissor and one Element2D per entity in it. */
void scene2d_setRenderContext(const Scene2D *scene, RenderContext *ctx);

#endif
