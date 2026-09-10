#include "physics/math/e32_vector3.h"

#include "physics/math/e32_math_common.h"


/* Only the square-root operations live here; the rest of the arithmetic is
   inline in the header. */

Fixed Vector3::magnitude() const
{
	return sqrt(squaredMagnitude());
}

/* The inverse root is the Quake trick in integers: no division at all. */
void Vector3::normalize()
{
	Fixed sq = squaredMagnitude();
	if (sq <= 0) return;
	Fixed inv = inverseSqrt(sq);
	x *= inv;
	y *= inv;
	z *= inv;
}

Vector3 Vector3::normalized() const
{
	Vector3 out = *this;
	out.normalize();
	return out;
}
