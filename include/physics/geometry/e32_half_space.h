/*
	Ported from qu3e q3Geometry.h — altered source, not the original software.

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

/*
	Plane as (normal, distance from origin).
*/
#ifndef ENGINE_32_HALF_SPACE_H
#define ENGINE_32_HALF_SPACE_H

#include "physics/math/e32_vector3.h"


struct HalfSpace
{
	Vector3 normal;
	Fixed   distance;

	static constexpr HalfSpace create(const Vector3 &normal, Fixed distance) { return {normal, distance}; }

	void setFromTriangle(const Vector3 &a, const Vector3 &b, const Vector3 &c);
	void setFromNormalPoint(const Vector3 &n, const Vector3 &p);

	constexpr Fixed distanceTo(const Vector3 &p) const { return normal.dot(p) - distance; }

	constexpr Vector3 projected(const Vector3 &p) const { return p - normal * distanceTo(p); }

	/* A single scale, inline. */
	constexpr Vector3 origin() const { return normal * distance; }
};


/* Two vectors perpendicular to a, and to each other. */
void vector3_computeBasis(const Vector3 &a, Vector3 *b, Vector3 *c);


#endif
