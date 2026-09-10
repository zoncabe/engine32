#include "system/e32_system.h"

#include "psyqo/application.hh"


/* The one application object. PSYQo calls prepare() with interrupts still
   disabled, so only the video mode is set there; anything that touches
   hardware waits for start(). */
class EngineApplication final : public psyqo::Application
{
	void prepare() override;
	void createScene() override;
};

static EngineApplication application;

/* Handed in by run(), called once the GPU is up. */
static void (*scene_hook)() = nullptr;


void EngineApplication::prepare()
{
	/* 320x240, 15 bit color, progressive. AUTO reads the console's region
	   and picks NTSC or PAL, so one executable runs on both. */
	psyqo::GPU::Configuration config;
	config.set(psyqo::GPU::Resolution::W320)
	      .set(psyqo::GPU::VideoMode::AUTO)
	      .set(psyqo::GPU::ColorMode::C15BITS)
	      .set(psyqo::GPU::Interlace::PROGRESSIVE);

	gpu().initialize(config);
}


void EngineApplication::createScene()
{
	if (scene_hook) scene_hook();
}


System &System::get()
{
	static System system;
	return system;
}


int System::run(void (*ready)())
{
	scene_hook = ready;
	return application.run();
}


void System::pushScene(psyqo::Scene *scene) { application.pushScene(scene); }

psyqo::Scene *System::popScene() { return application.popScene(); }

psyqo::GPU &System::getGpu() { return application.gpu(); }
