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

#include "physics/geometry/e32_half_space.h"


void HalfSpace::setFromTriangle(const Vector3 &a, const Vector3 &b, const Vector3 &c)
{
	Vector3 ab = b - a;
	Vector3 ac = c - a;
	normal   = ab.cross(ac).normalized();
	distance = normal.dot(a);
}

void HalfSpace::setFromNormalPoint(const Vector3 &n, const Vector3 &p)
{
	normal   = n.normalized();
	distance = normal.dot(p);
}


void vector3_computeBasis(const Vector3 &a, Vector3 *b, Vector3 *c)
{
	/* 1/sqrt(3): the axis a leans on most */
	if (a.x.abs() >= 0.57735027_fp) {
		*b = { a.y, -a.x, 0.0_fp };
	} else {
		*b = { 0.0_fp, a.z, -a.y };
	}
	*b = b->normalized();
	*c = a.cross(*b);
}
