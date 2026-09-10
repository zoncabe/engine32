/*
	Ported from qu3e q3Contact.h — altered source, not the original software.

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
	Contact point, manifold, edge, constraint. Pairs work on PhysicsShape (box
	/ sphere / capsule via tagged union).
*/
#ifndef ENGINE_32_CONTACT_H
#define ENGINE_32_CONTACT_H

#include <stdint.h>

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_math_common.h"
#include "physics/shapes/e32_physics_shape.h"


struct RigidBody;
struct ContactConstraint;


/* 32-bit key identifying a contact point across frames. */
union FeaturePair
{
	struct {
		uint8_t in_r;
		uint8_t out_r;
		uint8_t in_i;
		uint8_t out_i;
	};
	int32_t key;
};


struct ContactPoint
{
	Vector3     position;
	Fixed       penetration;
	Fixed       normal_impulse;
	Fixed       tangent_impulse[2];
	Fixed       bias;
	Fixed       normal_mass;
	Fixed       tangent_mass[2];
	FeaturePair fp;
	uint8_t     warm_started;
};


/* Up to 8 contact points between two shapes. */
struct ContactManifold
{
	PhysicsShape *A;
	PhysicsShape *B;

	Vector3       normal;               /* from A to B */
	Vector3       tangent_vectors[2];
	ContactPoint  contacts[8];
	int32_t       contact_count;

	ContactManifold *next;
	ContactManifold *prev;

	int sensor;

	void setPair(PhysicsShape *a, PhysicsShape *b);
};


/* Node in a body's intrusive contact list. */
struct ContactEdge
{
	RigidBody         *other;
	ContactConstraint *constraint;
	ContactEdge       *next;
	ContactEdge       *prev;
};


enum {
	CONSTRAINT_COLLIDING     = 0x00000001,
	CONSTRAINT_WAS_COLLIDING = 0x00000002,
	CONSTRAINT_ISLAND        = 0x00000004,
};


/* Persistent constraint between two bodies. */
struct ContactConstraint
{
	PhysicsShape *A;
	PhysicsShape *B;
	RigidBody    *body_a;
	RigidBody    *body_b;

	ContactEdge        edge_a;
	ContactEdge        edge_b;
	ContactConstraint *next;
	ContactConstraint *prev;

	Fixed friction;
	Fixed restitution;

	ContactManifold manifold;

	int32_t flags;

	/* Runs the narrowphase on the pair and keeps the colliding flags. */
	void solveCollision();
};


/* Restitution keeps the max, so the bounciest side wins; friction takes the
   geometric mean, so the slippery side dominates. */
static inline Fixed contact_mixRestitution(const PhysicsShape *A, const PhysicsShape *B) {
	return (A->restitution > B->restitution) ? A->restitution : B->restitution;
}

static inline Fixed contact_mixFriction(const PhysicsShape *A, const PhysicsShape *B) {
	return sqrt(A->friction * B->friction);
}


#endif
