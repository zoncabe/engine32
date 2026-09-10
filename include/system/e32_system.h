/*
	The platform side of the engine: everything PSYQo needs up before a game
	can run, gathered in one place so no game writes it out.

	PSYQo owns the frame loop. Its Application is started once and never
	returns: from then on the library calls back, and scenes are what it
	calls into. So the system holds that application, and the game hands it
	the first scene through run().

	The counterpart of ultra64's system module, which brings up the RCP
	scheduler instead.
*/
#ifndef ENGINE_32_SYSTEM_H
#define ENGINE_32_SYSTEM_H

#include "psyqo/gpu.hh"
#include "psyqo/scene.hh"


class System
{
public:

	static System &get();

	/* Starts the application and never returns: the video mode comes up,
	   then 'ready' is called once for the game to push its first scene, and
	   from there PSYQo drives a frame at a time. First thing main does. */
	int run(void (*ready)());

	/* Puts a scene on top of the stack, tearing down whatever was running.
	   Valid from 'ready' onwards: before that the GPU is not up yet. */
	void pushScene(psyqo::Scene *scene);

	/* Takes the top scene off and resumes the one below it. */
	psyqo::Scene *popScene();

	/* The GPU the engine draws through. */
	psyqo::GPU &getGpu();
};


#endif
