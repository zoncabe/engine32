/*
	Frame timing: the game reads a delta in seconds and never asks the
	hardware itself.

	The clock underneath is PSYQo's, a microsecond counter driven by one of
	the console's root counters.
*/
#ifndef ENGINE_32_TIME_H
#define ENGINE_32_TIME_H

#include <stdint.h>

#include "physics/math/e32_fixed.h"


class Time
{
public:

	static Time &get();

	void init();
	void update();
	void setScale(Fixed scale);

	/* After a blocking load: drops the time it took, so the next frame's
	   delta is a normal one instead of the whole load measured as
	   gameplay. */
	void reset();

	Fixed counter;   /* seconds since init, scaled */
	Fixed delta;     /* seconds, scaled; 1/4096 s resolution */
	Fixed rate;      /* frames per second, over the last real second */

private:

	Fixed scale;

	/* PSYQo's clock is microseconds in a 32 bit counter: it wraps around
	   every 71 minutes or so. Unsigned subtraction gives the right interval
	   across the wrap, so elapsed time is always a difference and never an
	   absolute. */
	uint32_t last_time;

	/* The rate is frames counted over a real second, not an average of
	   1/delta: that average reads high when frames alternate between one
	   and two refreshes. */
	uint32_t rate_start;
	int      rate_frames;
};


#endif
