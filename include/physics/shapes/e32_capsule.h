/*
	Capsule geometry (local Z axis): cylinder + hemispheres. Admin fields live
	in PhysicsShape.
*/
#ifndef ENGINE_32_CAPSULE_H
#define ENGINE_32_CAPSULE_H

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_transform.h"
#include "physics/geometry/e32_aabb.h"
#include "physics/geometry/e32_raycast.h"


struct MassData;


struct Capsule
{
	Fixed radius;
	Fixed half_height;   /* half-height along local Z, excluding caps */

	/* Endpoints of the inner segment (center ± half_height along Z), in world space. */
	void getSegment(const Transform &world, Vector3 *a, Vector3 *b) const;

	bool testPoint(const Transform &world, const Vector3 &p) const;
	bool raycast(const Transform &world, RaycastData *ray) const;
	AABB computeAABB(const Transform &world) const;
	void computeMass(const Transform &local, Fixed density, MassData *md) const;
};


struct CapsuleDef
{
	Transform tx;
	Fixed     radius;
	Fixed     half_height;
	Fixed     friction;
	Fixed     restitution;
	Fixed     density;
	int       sensor;

	void init();
	void set(const Transform &tx, Fixed radius, Fixed half_height);

	void setFriction(Fixed f)    { friction = f; }
	void setRestitution(Fixed r) { restitution = r; }
	void setDensity(Fixed rho)   { density = rho; }
	void setSensor(int s)        { sensor = s; }
};


#endif
