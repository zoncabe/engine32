#include <stdarg.h>

#include "psyqo/font.hh"
#include "psyqo/hardware/gpu.hh"
#include "psyqo/xprintf.h"

#include "debug/e32_debug.h"
#include "viewport/e32_viewport.h"
#include "camera/e32_camera.h"
#include "camera/e32_spring_arm.h"
#include "system/e32_system.h"
#include "time/e32_time.h"

#define DEBUG_UI_LINES    16
#define DEBUG_UI_LINE_MAX 40

#define DEBUG_UI_X       8
#define DEBUG_UI_Y       16
#define DEBUG_UI_STEP    10
#define DEBUG_UI_RIGHT_X 264

/* The font keeps one fragment per print, reused in a ring: two frames'
   worth of lines and the three of the framerate corner. */
static psyqo::Font<2 * (DEBUG_UI_LINES + 3)> debug_font;

/* An 8x8 font of our own, PSYQo's builtin being 8x16: 96 glyphs from the
   space, one bit per pixel. */
#include "e32_debug_font.inc"

#define DEBUG_FONT_W    8
#define DEBUG_FONT_H    8
#define DEBUG_FONT_X    960   /* in VRAM, where PSYQo puts its own */
#define DEBUG_FONT_Y    464
#define DEBUG_FONT_ROW  32    /* glyphs per row of the 256 px wide texture */

/* Expands the bits to the 4 bpp texture the font draws from, straight
   into VRAM, a word of eight pixels at a time; the palette is the font's
   own, index 1 white. Then the font takes the glyph size. */
static void debugUI_uploadFont(psyqo::GPU &gpu)
{
	int rows = 96 / DEBUG_FONT_ROW;

	psyqo::Prim::VRAMUpload upload;
	upload.region.pos  = {{.x = DEBUG_FONT_X, .y = DEBUG_FONT_Y}};
	upload.region.size = {{.w = DEBUG_FONT_ROW * DEBUG_FONT_W / 4, .h = (int16_t)(rows * DEBUG_FONT_H)}};
	gpu.sendPrimitive(upload);

	for (int row = 0; row < rows; row++) {
		for (int y = 0; y < DEBUG_FONT_H; y++) {
			for (int g = 0; g < DEBUG_FONT_ROW; g++) {
				uint8_t bits = debug_font_bits[(row * DEBUG_FONT_ROW + g) * DEBUG_FONT_H + y];
				uint32_t word = 0;
				for (int x = 0; x < 8; x++)
					if (bits & (0x80 >> x)) word |= 1u << (x * 4);
				psyqo::Hardware::GPU::Data = word;
			}
		}
	}

	debug_font.initialize(gpu, {{.x = DEBUG_FONT_X, .y = DEBUG_FONT_Y}},
	                      {{.w = DEBUG_FONT_W, .h = DEBUG_FONT_H}});
}

static const psyqo::Color debug_color = {{255, 255, 255}};

static char debug_line[DEBUG_UI_LINES][DEBUG_UI_LINE_MAX];
static bool debug_active;
static bool debug_show_fps;


void debugUI_init(void)
{
	debugUI_uploadFont(System::get().getGpu());
	debug_active = true;
}

void debugUI_show(bool show)
{
	debug_active = show;
}

void debugUI_set(uint8_t line, const char *fmt, ...)
{
	if (line >= DEBUG_UI_LINES) return;

	va_list args;
	va_start(args, fmt);
	vsnprintf(debug_line[line], DEBUG_UI_LINE_MAX, fmt, args);
	va_end(args);
}

/* The state asks here; the frame is not attached yet, so the drawing
   itself waits for debugUI_draw. The request lasts one frame. */
void debugUI_showFPS(void)
{
	debug_show_fps = true;
}

void debugUI_draw(void)
{
	if (!debug_active) return;

	psyqo::GPU &gpu = System::get().getGpu();

	for (int i = 0; i < DEBUG_UI_LINES; i++) {
		if (debug_line[i][0] == '\0') continue;
		debug_font.chainprint(gpu, debug_line[i],
		                      {{.x = DEBUG_UI_X, .y = (int16_t)(DEBUG_UI_Y + i * DEBUG_UI_STEP)}},
		                      debug_color);
	}

	/* Top right: the rate, then the camera's arm and lens under it. */
	if (debug_show_fps) {
		const Camera *camera = &Viewport::get().camera;

		debug_font.chainprintf(gpu, {{.x = DEBUG_UI_RIGHT_X, .y = DEBUG_UI_Y}}, debug_color,
		                       "fps %d", Time::get().rate.integer());
		debug_font.chainprintf(gpu, {{.x = DEBUG_UI_RIGHT_X, .y = DEBUG_UI_Y + DEBUG_UI_STEP}}, debug_color,
		                       "arm %d", CameraSpringArm::getLength(camera).integer());
		debug_font.chainprintf(gpu, {{.x = DEBUG_UI_RIGHT_X, .y = DEBUG_UI_Y + 2 * DEBUG_UI_STEP}}, debug_color,
		                       "fov %d", (Fixed(camera->field_of_view) * 180).integer());
		debug_show_fps = false;
	}
}
