#include <libdragon.h>
#include "graphics/e32_sprites.h"


void sprite_setMode()
{
	rdpq_set_mode_standard();
	rdpq_mode_blender(RDPQ_BLENDER_MULTIPLY);
	rdpq_mode_alphacompare(1);
}

void sprite_drawTiled(const Sprite *element, Vector2 position, Vector2 size)
{
	rdpq_sprite_upload(TILE0, element->asset, &(rdpq_texparms_t){
		.s = { .repeats = REPEAT_INFINITE },
		.t = { .repeats = REPEAT_INFINITE },
	});
	rdpq_texture_rectangle(TILE0, position.x, position.y,
	                       position.x + size.x, position.y + size.y, 0, 0);
}

void sprite_draw(const Sprite *element, Vector2 position, Vector2 scale, float rotation)
{
	sprite_t *s = element->asset;
	int cols = element->cols ? element->cols : 1;
	int rows = element->rows ? element->rows : 1;
	int w    = s->width  / cols;
	int h    = s->height / rows;

	/* The cell of the frame: column across, row down. */
	int col = element->frame % cols;
	int row = element->frame / cols;

	rdpq_sprite_blit(s, position.x, position.y, &(rdpq_blitparms_t){
		.s0      = col * w,
		.t0      = row * h,
		.width   = w,
		.height  = h,
		.flip_x  = element->flip_x,
		.scale_x = scale.x,
		.scale_y = scale.y,
		.theta   = rotation,
		.cx      = (rotation != 0.0f) ? w / 2 : 0,
		.cy      = (rotation != 0.0f) ? h / 2 : 0,
	});
}
