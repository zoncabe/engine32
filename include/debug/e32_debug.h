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


/* Prints the frame's cost through the BIOS teletype, one line every
   DEBUG_TRACE_FRAMES frames, averaged over them: a frame's numbers alone
   dance too much to read. ares routes the call to its own terminal, so the
   measurement needs nobody watching the screen.

   Called by the render at the tail of the frame, once the counters are
   closed. Without E32_TRACE it is not built and nothing is called. */
void debug_trace(void);

#ifdef E32_OTZ_TRACE
/* One line a second listing, per element drawn, the nearest and farthest
   bucket of the ordering table it wrote into. For finding out whether two
   meshes that fight share a bucket or sit in different ones. Built only
   with E32_OTZ_TRACE. */
void debug_traceOtz(int element, bool subdivide, uint32_t low, uint32_t high);
void debug_traceOtzEnd(void);
#endif

#ifdef E32_TRACE
/* What the frame spends outside the render: the game's own step, in ticks
   of root counter 2. Written by game_runStep, read and cleared by the
   trace. 'state' is the state update, 'input' the pad and the clock. */
extern uint32_t debug_profile_state;
extern uint32_t debug_profile_input;

/* And what the render spends around the meshes: 'setup' is building the
   frame's list of things to draw and setting the GPU up for it, 'end' is
   the particles, the clear, the table's chaining and the flip. */
extern uint32_t debug_profile_setup;
extern uint32_t debug_profile_end;
#endif

#endif
