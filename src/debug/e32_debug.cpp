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

#ifdef E32_TRACE
#include "common/hardware/counters.h"
#include "common/syscalls/syscalls.h"
#include "graphics/e32_mesh.h"
#include "render/e32_render.h"
#endif

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

/* A fixed point value as the two numbers a line prints, there being no
   float formatting here: the part before the point and the first decimal.

   Both come off the raw 20.12, not off integer(), which rounds to the
   nearest whole. Rounding gave the part before the point one value and the
   subtraction that used to make the decimal another, so 1.94 printed as
   "2.1": the whole went up to 2 while the tenths stayed at 19, and the
   difference came out negative and was flipped. The number read higher as
   the value went down, and did it again under 1. */
static int debug_whole(Fixed v)
{
	int32_t raw = v.raw();
	bool neg = raw < 0;
	if (neg) raw = -raw;
	int32_t w = raw >> 12;
	return (int)(neg ? -w : w);
}

/* The first decimal, always positive: the sign rides on the part before
   the point. */
static int debug_tenth(Fixed v)
{
	int32_t raw = v.raw();
	if (raw < 0) raw = -raw;
	return (int)(((raw & 4095) * 10) >> 12);
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

		Fixed arm = CameraSpringArm::getLength(camera);
		Fixed fov = Fixed(camera->field_of_view) * 180;

		debug_font.chainprintf(gpu, {{.x = DEBUG_UI_RIGHT_X, .y = DEBUG_UI_Y}}, debug_color,
		                       "fps %d", Time::get().rate.integer());
		debug_font.chainprintf(gpu, {{.x = DEBUG_UI_RIGHT_X, .y = DEBUG_UI_Y + DEBUG_UI_STEP}}, debug_color,
		                       "arm %d.%d", debug_whole(arm), debug_tenth(arm));
		debug_font.chainprintf(gpu, {{.x = DEBUG_UI_RIGHT_X, .y = DEBUG_UI_Y + 2 * DEBUG_UI_STEP}}, debug_color,
		                       "fov %d.%d", debug_whole(fov), debug_tenth(fov));
		debug_show_fps = false;
	}
}


#ifdef E32_OTZ_TRACE

/* The buckets of the ordering table each element wrote into, one line a
   second: which element, whether it subdivides, and the nearest and
   farthest bucket it reached. Two meshes that fight are either landing in
   the same bucket, where nothing sorts them, or in different ones, where
   the one in the nearer bucket is painted over the other. */
static char  debug_otz_line[224];
static int   debug_otz_len;
static uint32_t debug_otz_frames;

void debug_traceOtz(int element, bool subdivide, uint32_t low, uint32_t high)
{
	if (debug_otz_frames != 0 || low > high) return;
	if (debug_otz_len > (int)sizeof(debug_otz_line) - 32) return;

	debug_otz_len += snprintf(debug_otz_line + debug_otz_len,
	                          sizeof(debug_otz_line) - debug_otz_len,
	                          " e%d%s %d-%d", element, subdivide ? "s" : "",
	                          (int)low, (int)high);
}

void debug_traceOtzEnd(void)
{
	if (debug_otz_frames == 0 && debug_otz_len > 0) {
		syscall_puts("e32 otz");
		syscall_puts(debug_otz_line);
		syscall_puts("\n");
	}
	debug_otz_len = 0;
	if (++debug_otz_frames >= 60) debug_otz_frames = 0;
}

#endif


#ifdef E32_TRACE

/* Frames averaged into one line: a second's worth at sixty. */
#define DEBUG_TRACE_FRAMES 60

/* The frame's cost out the teletype. The BIOS call costs time of its own, so
   it lands here, after every counter of the frame is closed, and only once
   every DEBUG_TRACE_FRAMES.

   'cpu' is the whole frame in horizontal blanks of about 64 us. Everything
   else is ticks of root counter 2, 0.24 us each: 'faces' the face loops,
   'xform' the vertices through the GTE, 'emit' the primitives into the
   table. */
void debug_trace(void)
{
	static uint32_t frames;
	static uint32_t cpu, faces, xform, emit, prims, pieces, period, cuts, walked;
	static uint32_t clip, prim, direct, cut, lerp, project, state, input, setup, rend;
	static uint32_t level, objprep, whole, prep, lights, elems, cut2;
	static uint32_t whole1, sub, cover, slow, back, away;

	/* The whole frame, measured between two of these calls: the render's own
	   counters start at Render::frame and stop before the overlay, so they
	   miss the state's update, the overlay and the flip. 'period' is what
	   the frame rate is actually made of. */
	static uint16_t last;
	uint16_t now = COUNTERS[1].value;
	if (frames || period) period += (uint16_t)(now - last);
	last = now;

	const Render &render = Render::get();

	cpu    += render.hblanks();
	prims  += render.prims();
	faces  += mesh_profile_faces;
	xform  += mesh_profile_transform;
	emit   += mesh_profile_emit;
	pieces += mesh_profile_pieces;
	cuts   += mesh_profile_cuts;
	walked += mesh_profile_walked;
	clip    += mesh_profile_clip;
	prim    += mesh_profile_prim;
	direct  += mesh_profile_direct;
	cut     += mesh_profile_cut;
	lerp    += mesh_profile_lerp;
	project += mesh_profile_project;
	level   += mesh_profile_level;
	objprep += mesh_profile_object;
	whole   += mesh_profile_draw;
	prep    += mesh_profile_prep;
	cut2    += mesh_profile_cut2;
	whole1  += mesh_profile_whole;
	sub     += mesh_profile_sub;
	cover   += mesh_profile_cover;
	slow    += mesh_profile_slow;
	back    += mesh_profile_back;
	away    += mesh_profile_far;
	lights  += mesh_profile_light;
	elems   += mesh_profile_elements;
	state   += debug_profile_state;
	input   += debug_profile_input;
	setup   += debug_profile_setup;
	rend    += debug_profile_end;
	debug_profile_state = debug_profile_input = 0;
	debug_profile_setup = debug_profile_end = 0;

	if (++frames < DEBUG_TRACE_FRAMES) return;

	char line[320];
	snprintf(line, sizeof(line),
	         "e32 frame %d cpu %d state %d setup %d rend %d faces %d xform %d direct %d prim %d "
	         "flat %d sub %d cover %d clip %d cut %d cut2 %d lerp %d proj %d draw %d prep %d lights %d elems %d level %d objp %d emit %d cuts %d prims %d pieces %d walked %d slow %d back %d away %d\n",
	         (int)(period / frames), (int)(cpu / frames), (int)(state / frames),
	         (int)(setup / frames), (int)(rend / frames), (int)(faces / frames),
	         (int)(xform / frames), (int)(direct / frames), (int)(prim / frames),
	         (int)(whole1 / frames), (int)(sub / frames), (int)(cover / frames),
	         (int)(clip / frames),
	         (int)(cut / frames), (int)(cut2 / frames),
	         (int)(lerp / frames), (int)(project / frames),
	         (int)(whole / frames), (int)(prep / frames),
	         (int)(lights / frames), (int)(elems / frames),
	         (int)(level / frames), (int)(objprep / frames),
	         (int)(emit / frames), (int)(cuts / frames), (int)(prims / frames),
	         (int)(pieces / frames), (int)(walked / frames), (int)(slow / frames),
	         (int)(back / frames), (int)(away / frames));
	syscall_puts(line);

	frames = 0;
	cpu = faces = xform = emit = prims = pieces = period = cuts = walked = 0;
	clip = prim = direct = cut = lerp = project = state = input = setup = rend = 0;
	level = objprep = whole = prep = lights = elems = cut2 = 0;
	whole1 = sub = cover = slow = back = away = 0;
}

#endif
