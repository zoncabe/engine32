#ifndef ENGINE_32_MATH_COMMON_H
#define ENGINE_32_MATH_COMMON_H

#include "physics/math/e32_fixed.h"


constexpr Fixed PI         = 3.141592_fp;
constexpr Fixed PI_TIMES_2 = 6.283185_fp;

/* One raw step: the smallest difference Q12 can tell apart. */
constexpr Fixed TOLERANCE = Fixed(1, Fixed::RAW);


/* One-liners are inline: the body is a single op, a call is not. */

static inline constexpr Fixed lerp(Fixed a, Fixed b, Fixed t)
{
	return a + t * (b - a);
}

static inline constexpr Fixed clamp(Fixed v, Fixed lo, Fixed hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

/* Exact integer square root of a Q12 value: 32 rounds of shift and
   subtract on the 64-bit radicand, no division anywhere. */
Fixed sqrt(Fixed x);

/*
	1/sqrt(x) the Quake way, in integers: the seed is 2^(-e/2) from the
	leading zero count of the raw value, then two Newton steps that only
	multiply: y = y * (3 - x * y * y) / 2. No division. About 0.1% error,
	fine for normals and directions; use sqrt where the exact value matters.
*/
Fixed inverseSqrt(Fixed x);

/* e^-x for x >= 0, the factor an exponential smoothing needs per frame.
   Polynomial on the fraction of x*log2(e), shift on the integer part. */
Fixed expNeg(Fixed x);


Fixed ease_linear(Fixed t);

Fixed ease_quad_in(Fixed t);
Fixed ease_quad_out(Fixed t);
Fixed ease_quad_in_out(Fixed t);

Fixed ease_cubic_in(Fixed t);
Fixed ease_cubic_out(Fixed t);
Fixed ease_cubic_in_out(Fixed t);

Fixed ease_expo_in(Fixed t);
Fixed ease_expo_out(Fixed t);
Fixed ease_expo_in_out(Fixed t);


#endif
