#include "physics/math/e32_trig.h"

#include "physics/math/e32_math_common.h"


/* One table for the whole engine, 2 KB, generated at startup. */
static psyqo::Trig<> table;


Fixed Trig::sin(Angle a) { return table.sin(a); }
Fixed Trig::cos(Angle a) { return table.cos(a); }

void Trig::sincos(Angle a, Fixed *s, Fixed *c)
{
	*s = table.sin(a);
	*c = table.cos(a);
}


Angle Trig::atan2(Fixed y, Fixed x)
{
	if (x == 0 && y == 0) return Angle();

	Fixed ax = x.abs();
	Fixed ay = y.abs();

	/* Reduce to the first octant: z in [0, 1]. */
	bool  steep = ay > ax;
	Fixed z     = steep ? ax / ay : ay / ax;

	/* atan(z) / pi, for z in [0, 1] */
	Fixed a = z * (0.25_fp + 0.0869_fp * (1 - z));

	if (steep) a = 0.5_fp - a;
	if (x < 0) a = 1 - a;
	if (y < 0) a = -a;

	return Angle(a);
}


Angle Trig::acos(Fixed x)
{
	Fixed s = sqrt(1 - x * x);
	return atan2(s, x);
}


Angle Trig::wrap(Angle a)
{
	while (a >  1.0_pi) a -= 2.0_pi;
	while (a <= -1.0_pi) a += 2.0_pi;
	return a;
}

Angle Trig::wrapRelative(Angle a, Angle reference)
{
	while (a >  reference + 1.0_pi) a -= 2.0_pi;
	while (a <= reference - 1.0_pi) a += 2.0_pi;
	return a;
}
