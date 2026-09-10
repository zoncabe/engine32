/*
	Ported from qu3e q3ContactSolver.h — altered source, not the original software.

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
	Sequential impulse constraint solver.
*/
#ifndef ENGINE_32_CONTACT_SOLVER_H
#define ENGINE_32_CONTACT_SOLVER_H

#include <stdint.h>

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_matrix3.h"
#include "physics/e32_physics_settings.h"


struct PhysicsIsland;
struct VelocityState;


struct ContactState
{
	Vector3 ra;
	Vector3 rb;
	Fixed   penetration;
	Fixed   normal_impulse;
	Fixed   tangent_impulse[2];
	Fixed   bias;
	Fixed   normal_mass;
	Fixed   tangent_mass[2];
};


struct ContactConstraintState
{
	ContactState contacts[8];
	int32_t      contact_count;
	Vector3      tangent_vectors[2];
	Vector3      normal;
	Vector3      center_a;
	Vector3      center_b;
	Matrix3      iA;
	Matrix3      iB;
	Fixed        mA;
	Fixed        mB;
	Fixed        restitution;
	Fixed        friction;
	int32_t      index_a;
	int32_t      index_b;
};


struct ContactSolver
{
	PhysicsIsland          *island;
	ContactConstraintState *contacts;
	int32_t                 contact_count;
	VelocityState          *velocities;
	int                     enable_friction;

	void initialize(PhysicsIsland *island);
	void shutdown();
	void preSolve(Fixed dt);
	void solve();
};


#endif
