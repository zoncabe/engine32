#include <assert.h>
#include "graphics/e32_font.h"
#include "resource/e32_resource.h"


/* The game's table, handed over at font_init. rdpq keeps the loaded font
   behind its id, so nothing else is stored here. */
static const FontDef *font_def;
static uint8_t        font_count;


void font_init(const FontDef *fonts, uint8_t count)
{
	font_def   = fonts;
	font_count = count;
}

void font_loadAsset(uint8_t id)
{
	assert(id < font_count && font_def[id].path);

	const FontDef *def = &font_def[id];

	rdpq_font_t *font = resource_load(def->path, RESOURCE_FONT, NULL);
	assert(font);

	for (int i = 0; i < def->style_count; i++)
		rdpq_font_style(font, def->style[i].id, &def->style[i].style);

	rdpq_text_register_font(id, font);
}

void font_unloadAsset(uint8_t id)
{
	rdpq_font_t *font = (rdpq_font_t *)rdpq_text_get_font(id);
	rdpq_text_unregister_font(id);
	resource_unload(font);
}

void text_draw(const Text *element, Vector2 position)
{
	rdpq_text_printf(element->parms, element->font, position.x, position.y, "^%02d%s", element->style, element->text);
}
