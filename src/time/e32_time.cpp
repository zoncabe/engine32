#include "time/e32_time.h"

#include "system/e32_system.h"


static const uint32_t MICROSECONDS = 1000000;


/* Microseconds to seconds in Q12: the product fits 64 bits, and the
   division is one 32-bit divide once the shift is folded in. */
static Fixed seconds(uint32_t microseconds)
{
	int64_t raw = ((int64_t)microseconds << 12) / MICROSECONDS;
	return Fixed((int32_t)raw, Fixed::RAW);
}


Time &Time::get()
{
	static Time time;
	return time;
}


void Time::init()
{
	counter = 1.0_fp;
	delta   = 0.0_fp;
	rate    = 0.0_fp;
	scale   = 1.0_fp;

	last_time  = System::get().getGpu().now();
	rate_start = last_time;

	rate_frames = 0;
}


void Time::setScale(Fixed value) { scale = value; }


void Time::reset()
{
	last_time   = System::get().getGpu().now();
	rate_start  = last_time;
	rate_frames = 0;
}


void Time::update()
{
	uint32_t now = System::get().getGpu().now();

	delta    = seconds(now - last_time) * scale;
	counter += delta;

	last_time = now;

	rate_frames++;
	if (now - rate_start >= MICROSECONDS) {
		/* frames / elapsed seconds, without going through a Q12 divide */
		int64_t raw = ((int64_t)rate_frames * MICROSECONDS << 12) / (now - rate_start);
		rate = Fixed((int32_t)raw, Fixed::RAW);

		rate_start  = now;
		rate_frames = 0;
	}
}
