#include "physics/math/e32_math_common.h"
#include "physics/geometry/e32_triangle.h"


/* Möller–Trumbore. A back face is a miss: a surface only counts from the side
   its normal points to, so a floor is floor from above and nothing from below.
   Same convention as the shape raycasts: ray->t bounds the ray, and a hit
   writes the distance along dir into ray->toi.

   Each ratio divides by det on its own instead of going through 1/det:
   the inverse would keep 12 bits, the division keeps them all. */
bool Triangle::raycast(RaycastData *ray) const
{
	Vector3 ab = vertices[1] - vertices[0];
	Vector3 ac = vertices[2] - vertices[0];

	Vector3 p   = ray->dir.cross(ac);
	Fixed   det = ab.dot(p);
	if (det < TOLERANCE) return false;   /* parallel to the face, or coming from behind it */

	Vector3 ap = ray->start - vertices[0];

	Fixed u = ap.dot(p) / det;
	if (u < 0 || u > 1.0_fp) return false;

	Vector3 q = ap.cross(ab);
	Fixed   v = ray->dir.dot(q) / det;
	if (v < 0 || u + v > 1.0_fp) return false;

	Fixed toi = ac.dot(q) / det;
	if (toi < 0 || toi > ray->t) return false;

	ray->toi    = toi;
	ray->normal = normal;
	return true;
}
