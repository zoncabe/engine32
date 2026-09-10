/*
	Debug UI: live values on screen, independent of whatever state runs.

	A special module on purpose: it skips the scene2d path and draws its
	lines straight through the GPU at the tail of every frame, with PSYQo's
	builtin system font — no game asset, no def, no load. Off, it costs one
	branch.
*/
#ifndef ENGINE_32_DEBUG_H
#define ENGINE_32_DEBUG_H

#include <stdbool.h>
#include <stdint.h>


/* Uploads the builtin font and turns the overlay on. Once, from the game's
   setup. Never called, the overlay stays off and draws nothing. */
void debugUI_init(void);

void debugUI_show(bool show);

/* Rewrites one line, printf style. Lines draw top to bottom in slot order;
   an empty slot skips its row. */
void debugUI_set(uint8_t line, const char *fmt, ...);

/* Asks for the framerate at the top right, first line. Call it from the
   state update; the number draws at the tail of that same frame and the
   request expires with it. */
void debugUI_showFPS(void);

/* Drawn by the render at the tail of the frame, over everything. */
void debugUI_draw(void);

#endif
