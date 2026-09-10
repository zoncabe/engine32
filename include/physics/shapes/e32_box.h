/*
	Ported from qu3e q3Box.h — altered source, not the original software.

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
	OBB geometry (half-extents on each local axis). Admin fields live in
	PhysicsShape.
*/
#ifndef ENGINE_32_BOX_H
#define ENGINE_32_BOX_H

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_transform.h"
#include "physics/geometry/e32_aabb.h"
#include "physics/geometry/e32_raycast.h"


struct MassData;


struct Box
{
	Vector3 e;   /* half-extents on each OBB axis */

	bool testPoint(const Transform &world, const Vector3 &p) const;
	bool raycast(const Transform &world, RaycastData *ray) const;
	AABB computeAABB(const Transform &world) const;
	void computeMass(const Transform &local, Fixed density, MassData *md) const;
};


struct BoxDef
{
	Transform tx;
	Vector3   e;
	Fixed     friction;
	Fixed     restitution;
	Fixed     density;
	int       sensor;

	void init();
	void set(const Transform &tx, const Vector3 &full_extents);

	void setFriction(Fixed f)    { friction = f; }
	void setRestitution(Fixed r) { restitution = r; }
	void setDensity(Fixed rho)   { density = rho; }
	void setSensor(int s)        { sensor = s; }
};


#endif
