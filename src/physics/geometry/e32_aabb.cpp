/*
	Ported from qu3e q3Geometry.cpp — altered source, not the original software.

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/

#include "physics/geometry/e32_aabb.h"


Fixed AABB::surfaceArea() const
{
	Fixed x = max.x - min.x;
	Fixed y = max.y - min.y;
	Fixed z = max.z - min.z;
	return (x*y + x*z + y*z) * 2;
}

AABB AABB::combined(const AABB &b) const
{
	AABB c;
	c.min = {
		min.x < b.min.x ? min.x : b.min.x,
		min.y < b.min.y ? min.y : b.min.y,
		min.z < b.min.z ? min.z : b.min.z,
	};
	c.max = {
		max.x > b.max.x ? max.x : b.max.x,
		max.y > b.max.y ? max.y : b.max.y,
		max.z > b.max.z ? max.z : b.max.z,
	};
	return c;
}


Vector3 AABB::closestToPoint(const Vector3 &p) const
{
	Vector3 c;
	c.x = p.x < min.x ? min.x : (p.x > max.x ? max.x : p.x);
	c.y = p.y < min.y ? min.y : (p.y > max.y ? max.y : p.y);
	c.z = p.z < min.z ? min.z : (p.z > max.z ? max.z : p.z);
	return c;
}


/* Closest point on the AABB to the segment [a, b]. Ported from the old engine
   (ODE-style iterative clipping against the box slabs). */
Vector3 AABB::closestToSegment(const Vector3 &a, const Vector3 &b) const
{
	Vector3 center    = (min + max) * 0.5_fp;
	Vector3 half_size = (max - min) * 0.5_fp;

	Vector3 s = a - center;
	Vector3 v = b - a;
	int sign[3] = { 1, 1, 1 };

	if (v.x < 0) { s.x = -s.x; v.x = -v.x; sign[0] = -1; }
	if (v.y < 0) { s.y = -s.y; v.y = -v.y; sign[1] = -1; }
	if (v.z < 0) { s.z = -s.z; v.z = -v.z; sign[2] = -1; }

	Vector3 v2 = { v.x * v.x, v.y * v.y, v.z * v.z };
	Vector3 tanchor = { 2.0_fp, 2.0_fp, 2.0_fp };
	int region[3] = { 0, 0, 0 };

	if (v.x > 0) {
		if      (s.x < -half_size.x) { region[0] = -1; tanchor.x = (-half_size.x - s.x) / v.x; }
		else if (s.x >  half_size.x) { region[0] =  1; tanchor.x = ( half_size.x - s.x) / v.x; }
	}
	if (v.y > 0) {
		if      (s.y < -half_size.y) { region[1] = -1; tanchor.y = (-half_size.y - s.y) / v.y; }
		else if (s.y >  half_size.y) { region[1] =  1; tanchor.y = ( half_size.y - s.y) / v.y; }
	}
	if (v.z > 0) {
		if      (s.z < -half_size.z) { region[2] = -1; tanchor.z = (-half_size.z - s.z) / v.z; }
		else if (s.z >  half_size.z) { region[2] =  1; tanchor.z = ( half_size.z - s.z) / v.z; }
	}

	Fixed t = 0;
	Fixed dd2dt = 0;
	if (region[0]) dd2dt -= v2.x * tanchor.x;
	if (region[1]) dd2dt -= v2.y * tanchor.y;
	if (region[2]) dd2dt -= v2.z * tanchor.z;

	if (dd2dt < 0) {
		for (;;) {
			Fixed next_t = 1.0_fp;
			if (tanchor.x > t && tanchor.x < 1.0_fp && tanchor.x < next_t) next_t = tanchor.x;
			if (tanchor.y > t && tanchor.y < 1.0_fp && tanchor.y < next_t) next_t = tanchor.y;
			if (tanchor.z > t && tanchor.z < 1.0_fp && tanchor.z < next_t) next_t = tanchor.z;

			Fixed next_dd2dt = 0;
			if (region[0]) next_dd2dt += v2.x * (next_t - tanchor.x);
			if (region[1]) next_dd2dt += v2.y * (next_t - tanchor.y);
			if (region[2]) next_dd2dt += v2.z * (next_t - tanchor.z);

			if (next_dd2dt >= 0) {
				Fixed m = (next_dd2dt - dd2dt) / (next_t - t);
				t -= dd2dt / m;
				break;
			}

			if (tanchor.x == next_t) { tanchor.x = ( half_size.x - s.x) / v.x; region[0] += 1; }
			if (tanchor.y == next_t) { tanchor.y = ( half_size.y - s.y) / v.y; region[1] += 1; }
			if (tanchor.z == next_t) { tanchor.z = ( half_size.z - s.z) / v.z; region[2] += 1; }

			t     = next_t;
			dd2dt = next_dd2dt;
			if (t >= 1.0_fp) { t = 1.0_fp; break; }
		}
	}

	Vector3 tmp = {
		(s.x + v.x * t) * sign[0],
		(s.y + v.y * t) * sign[1],
		(s.z + v.z * t) * sign[2],
	};

	if (tmp.x < -half_size.x) tmp.x = -half_size.x;
	else if (tmp.x > half_size.x) tmp.x = half_size.x;
	if (tmp.y < -half_size.y) tmp.y = -half_size.y;
	else if (tmp.y > half_size.y) tmp.y = half_size.y;
	if (tmp.z < -half_size.z) tmp.z = -half_size.z;
	else if (tmp.z > half_size.z) tmp.z = half_size.z;

	return tmp + center;
}
