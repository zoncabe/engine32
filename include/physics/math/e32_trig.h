/*
	Angles and trigonometry on fixed point.

	An Angle is PSYQo's: 10 fractional bits, in fractions of pi. 1.0_pi is a
	half turn, 0.5_pi a quarter. There are no radians or degrees anywhere.

	Sine and cosine come from PSYQo's 512 entry table. atan2 and acos are
	not in PSYQo; the approximation here is Rajan, Wang, Inkol and Joyal
	(2006), max error 0.22 degrees, which the camera and the euler readback
	tolerate. Anything that accumulates error should stay in matrices.
*/
#ifndef ENGINE_32_TRIG_H
#define ENGINE_32_TRIG_H

#include "psyqo/trigonometry.hh"

#include "physics/math/e32_fixed.h"


using Angle = psyqo::Angle;

using namespace psyqo::trig_literals;

/* Degrees, for reading: 90.0_deg is 0.5_pi. Compile time only, like _pi;
   the value in the binary is the Angle. */
consteval Angle operator""_deg(long double degrees) { return degrees / 180.0L; }


class Trig
{
public:

	static Fixed sin(Angle a);
	static Fixed cos(Angle a);

	/* Both at once: the table is walked once per angle. */
	static void sincos(Angle a, Fixed *s, Fixed *c);

	/* Result in (-1.0_pi, 1.0_pi]. Both zero gives zero. */
	static Angle atan2(Fixed y, Fixed x);

	/* x in [-1, 1]; result in [0, 1.0_pi]. */
	static Angle acos(Fixed x);

	/* Wraps into (-1.0_pi, 1.0_pi]. */
	static Angle wrap(Angle a);
	static Angle wrapRelative(Angle a, Angle reference);
};


#endif
