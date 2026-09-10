#include "physics/math/e32_math_common.h"


/* sqrt(x) in Q12 is isqrt(raw << 12): the radicand is 44 bits wide, so it
   runs on 64. Classic digit-by-digit routine, one bit of result per round. */
Fixed sqrt(Fixed x)
{
	if (x.raw() <= 0) return Fixed();

	uint64_t n   = (uint64_t)x.raw() << 12;
	uint64_t res = 0;
	uint64_t bit = (uint64_t)1 << 42;   /* highest power of 4 below 2^44 */

	while (bit > n) bit >>= 2;

	while (bit != 0) {
		if (n >= res + bit) {
			n   -= res + bit;
			res  = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}

	return Fixed((int32_t)res, Fixed::RAW);
}


Fixed inverseSqrt(Fixed x)
{
	if (x.raw() <= 0) return Fixed();

	/* x = m * 4^k with m in [0.25, 1): the exponent comes out of the
	   root as 2^-k, and the Newton runs on m alone, where every
	   intermediate stays between 0.25 and 2 and keeps its bits. */
	int32_t m = x.raw();
	int k = 0;
	while (m >= 4096) { m >>= 2; k++; }
	while (m <  1024) { m <<= 2; k--; }

	Fixed mf   = Fixed(m, Fixed::RAW);
	Fixed half = mf / 2;

	/* The seed is the line through (0.25, 1.375) and (1, 1): never more
	   than 30% under the root, so three Newton steps land within 0.1%. */
	Fixed y = 1.5_fp - half;

	/* Newton on f(y) = 1/y^2 - m, multiplies only. */
	y = y * (1.5_fp - half * y * y);
	y = y * (1.5_fp - half * y * y);
	y = y * (1.5_fp - half * y * y);

	int32_t raw = y.raw();
	if (k > 0) raw >>= k;
	else       raw <<= -k;

	return Fixed(raw, Fixed::RAW);
}


Fixed expNeg(Fixed x)
{
	if (x <= 0) return 1.0_fp;
	if (x > 20) return Fixed();

	/* e^x = 2^(x * log2 e): integer part is a shift, fraction a cubic. */
	Fixed   t = x * 1.442695_fp;
	int32_t i = t.floor();
	Fixed   f = t - Fixed(i, 0);

	Fixed p = 1 + f * (0.693147_fp + f * (0.240227_fp + f * 0.055504_fp));

	/* 1 / (2^i * p): divide first, shift after, so the small value keeps
	   its bits. */
	Fixed inv = 1 / p;
	return Fixed(inv.raw() >> i, Fixed::RAW);
}


Fixed ease_linear(Fixed t)
{
	return t;
}

Fixed ease_quad_in(Fixed t)
{
	return t * t;
}

Fixed ease_quad_out(Fixed t)
{
	return 1 - (1 - t) * (1 - t);
}

Fixed ease_quad_in_out(Fixed t)
{
	if (t < 0.5_fp) return t * t * 2;
	Fixed inv = 1 - t;
	return 1 - inv * inv * 2;
}

Fixed ease_cubic_in(Fixed t)
{
	return t * t * t;
}

Fixed ease_cubic_out(Fixed t)
{
	Fixed inv = 1 - t;
	return 1 - inv * inv * inv;
}

Fixed ease_cubic_in_out(Fixed t)
{
	if (t < 0.5_fp) return t * t * t * 4;
	Fixed inv = 1 - t;
	return 1 - inv * inv * inv * 4;
}

/* The exponential eases are 2^(10 t - 10): expNeg of the negated exponent
   times ln 2. */
Fixed ease_expo_in(Fixed t)
{
	if (t <= 0) return Fixed();
	return expNeg((10 - t * 10) * 0.693147_fp);
}

Fixed ease_expo_out(Fixed t)
{
	if (t >= 1) return 1.0_fp;
	return 1 - expNeg(t * 10 * 0.693147_fp);
}

Fixed ease_expo_in_out(Fixed t)
{
	if (t <= 0) return Fixed();
	if (t >= 1) return 1.0_fp;
	if (t < 0.5_fp) return expNeg((10 - t * 20) * 0.693147_fp) / 2;
	return 1 - expNeg((t * 20 - 10) * 0.693147_fp) / 2;
}
