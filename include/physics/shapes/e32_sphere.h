/*
	Sphere geometry (radius). Admin fields live in PhysicsShape.
*/
#ifndef ENGINE_32_SPHERE_H
#define ENGINE_32_SPHERE_H

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_transform.h"
#include "physics/geometry/e32_aabb.h"
#include "physics/geometry/e32_raycast.h"


struct MassData;


struct Sphere
{
	Fixed radius;

	bool testPoint(const Transform &world, const Vector3 &p) const;
	bool raycast(const Transform &world, RaycastData *ray) const;
	AABB computeAABB(const Transform &world) const;
	void computeMass(const Transform &local, Fixed density, MassData *md) const;
};


struct SphereDef
{
	Transform tx;
	Fixed     radius;
	Fixed     friction;
	Fixed     restitution;
	Fixed     density;
	int       sensor;

	void init();
	void set(const Transform &tx, Fixed radius);

	void setFriction(Fixed f)    { friction = f; }
	void setRestitution(Fixed r) { restitution = r; }
	void setDensity(Fixed rho)   { density = rho; }
	void setSensor(int s)        { sensor = s; }
};


#endif
