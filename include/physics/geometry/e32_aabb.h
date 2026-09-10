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
	Axis-aligned bounding box.
*/
#ifndef ENGINE_32_AABB_H
#define ENGINE_32_AABB_H

#include "physics/math/e32_vector3.h"


struct AABB
{
	Vector3 min;
	Vector3 max;

	constexpr bool containsAABB(const AABB &other) const
	{
		return
			min.x <= other.min.x && min.y <= other.min.y && min.z <= other.min.z &&
			max.x >= other.max.x && max.y >= other.max.y && max.z >= other.max.z;
	}

	constexpr bool containsPoint(const Vector3 &p) const
	{
		return
			min.x <= p.x && min.y <= p.y && min.z <= p.z &&
			max.x >= p.x && max.y >= p.y && max.z >= p.z;
	}

	constexpr bool overlaps(const AABB &b) const
	{
		if (max.x < b.min.x || min.x > b.max.x) return false;
		if (max.y < b.min.y || min.y > b.max.y) return false;
		if (max.z < b.min.z || min.z > b.max.z) return false;
		return true;
	}

	/* 2(xy + xz + yz): in 20.12 it holds up to a box of 295 m a side.
	   Only the tree's cost heuristic reads it, and only to compare. */
	Fixed surfaceArea() const;

	AABB combined(const AABB &b) const;

	Vector3 closestToPoint  (const Vector3 &p) const;
	Vector3 closestToSegment(const Vector3 &a, const Vector3 &b) const;
};


#endif
