/*
	Ported from qu3e q3Box.cpp — altered source, not the original software.

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

#include "physics/shapes/e32_box.h"
#include "physics/shapes/e32_physics_shape.h"   /* MassData */


bool Box::testPoint(const Transform &world, const Vector3 &p) const
{
	Vector3 p0 = world.mulVectorTransposed(p);

	for (int i = 0; i < 3; ++i) {
		if (p0[i] >  e[i]) return false;
		if (p0[i] < -e[i]) return false;
	}
	return true;
}


bool Box::raycast(const Transform &world, RaycastData *ray) const
{
	Vector3 d = world.rotation.transformTransposed(ray->dir);
	Vector3 p = world.mulVectorTransposed(ray->start);

	Fixed tmin = 0;
	Fixed tmax = ray->t;

	Vector3 n0 = Vector3::zero();

	for (int i = 0; i < 3; ++i) {
		/* Under one raw unit the direction has no component on this axis:
		   the ray runs along the slab. */
		if (d[i].raw() == 0) {
			if (p[i] < -e[i] || p[i] > e[i]) return false;
		}
		else {
			int   s  = (d[i] >= 0) ? 1 : -1;
			Fixed ei = e[i] * s;

			Vector3 n = Vector3::zero();
			n[i] = (s > 0) ? -1.0_fp : 1.0_fp;

			Fixed t0 = -(ei + p[i]) / d[i];
			Fixed t1 =  (ei - p[i]) / d[i];

			if (t0 > tmin) { n0 = n; tmin = t0; }
			if (t1 < tmax) { tmax = t1; }

			if (tmin > tmax) return false;
		}
	}

	ray->normal = world.rotation.transform(n0);
	ray->toi    = tmin;
	return true;
}


AABB Box::computeAABB(const Transform &world) const
{
	Vector3 v[8] = {
		{-e.x, -e.y, -e.z}, {-e.x, -e.y,  e.z},
		{-e.x,  e.y, -e.z}, {-e.x,  e.y,  e.z},
		{ e.x, -e.y, -e.z}, { e.x, -e.y,  e.z},
		{ e.x,  e.y, -e.z}, { e.x,  e.y,  e.z},
	};

	for (int i = 0; i < 8; ++i) {
		v[i] = world.mulVector(v[i]);
	}

	AABB aabb = { v[0], v[0] };

	for (int i = 1; i < 8; ++i) {
		if (v[i].x < aabb.min.x) aabb.min.x = v[i].x;
		if (v[i].y < aabb.min.y) aabb.min.y = v[i].y;
		if (v[i].z < aabb.min.z) aabb.min.z = v[i].z;
		if (v[i].x > aabb.max.x) aabb.max.x = v[i].x;
		if (v[i].y > aabb.max.y) aabb.max.y = v[i].y;
		if (v[i].z > aabb.max.z) aabb.max.z = v[i].z;
	}

	return aabb;
}


void Box::computeMass(const Transform &local, Fixed density, MassData *md) const
{
	Fixed ex2  = e.x * e.x * 4;
	Fixed ey2  = e.y * e.y * 4;
	Fixed ez2  = e.z * e.z * 4;
	Fixed mass = e.x * e.y * e.z * density * 8;

	Fixed ix = mass * (ey2 + ez2) / 12;
	Fixed iy = mass * (ex2 + ez2) / 12;
	Fixed iz = mass * (ex2 + ey2) / 12;
	Matrix3 I = Matrix3::diagonal(ix, iy, iz);

	/* I = R·I·Rᵀ. */
	I = local.rotation * I * local.rotation.transposed();

	/* Parallel-axis theorem: I += m · (|c|² · identity - c⊗c). */
	Fixed dot = local.position.dot(local.position);
	I += (Matrix3::identity() * dot - Matrix3::outerProduct(local.position, local.position)) * mass;

	md->center  = local.position;
	md->inertia = I;
	md->mass    = mass;
}


void BoxDef::init()
{
	tx.init();
	e           = Vector3::zero();
	friction    = 0.4_fp;
	restitution = 0.2_fp;
	density     = 1.0_fp;
	sensor      = 0;
}


void BoxDef::set(const Transform &t, const Vector3 &full_extents)
{
	tx = t;
	e  = full_extents * 0.5_fp;
}
