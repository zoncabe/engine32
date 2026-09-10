/*
	The smallest executable the engine can boot: the video mode up and a
	frame cleared to a color that drifts with time. It exercises the
	platform layer alone: system, viewport, time.

	The frame loop is PSYQo's, not the engine's: run() never returns, and
	from there the library calls the scene once per frame and flips.
*/
#include "psyqo/scene.hh"

#include "system/e32_system.h"
#include "time/e32_time.h"
#include "viewport/e32_viewport.h"


/* 0..1 to 0..255 */
static uint8_t channel(Fixed v)
{
	return (uint8_t)(v * 255).integer();
}

/* Three seconds around the wheel: red, green, blue and back. */
static void set_clear_color(Fixed seconds)
{
	/* position inside the three second cycle, 0..3 */
	Fixed t = seconds - Fixed(seconds.floor() / 3 * 3, 0);

	Fixed r = 0.0_fp, g = 0.0_fp, b = 0.0_fp;

	if      (t < 1) { r = 1 - t;  g = t;     }
	else if (t < 2) { g = 2 - t;  b = t - 1; }
	else            { b = 3 - t;  r = t - 2; }

	Viewport::get().setClearColor(channel(r), channel(g), channel(b));
}


class BootScene final : public psyqo::Scene
{
	void frame() override;

	Fixed hue_time;
};

static BootScene boot_scene;


void BootScene::frame()
{
	Time::get().update();

	hue_time += Time::get().delta;

	set_clear_color(hue_time);
	Viewport::get().clear();
}


/* Called once the GPU is up, before the first frame. */
static void ready()
{
	Time::get().init();
	Viewport::get().init();

	System::get().pushScene(&boot_scene);
}


int main()
{
	return System::get().run(ready);
}
