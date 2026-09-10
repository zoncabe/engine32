/*
	Ported from qu3e q3Scene.h — altered source, not the original software.

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
	Top-level world: bodies, broadphase and contact manager. Owns the memory
	allocators.
*/
#ifndef ENGINE_32_PHYSICS_WORLD_H
#define ENGINE_32_PHYSICS_WORLD_H

#include <stdint.h>

#include "physics/math/e32_vector3.h"
#include "physics/memory/e32_physics_stack.h"
#include "physics/memory/e32_physics_heap.h"
#include "physics/memory/e32_physics_paged_allocator.h"
#include "physics/collision/e32_contact_manager.h"
#include "physics/body/e32_rigid_body.h"
#include "physics/buoyancy/e32_buoyancy.h"
#include "physics/cloth/e32_cloth.h"
#include "physics/shapes/e32_physics_shape.h"
#include "physics/geometry/e32_aabb.h"
#include "physics/geometry/e32_raycast.h"


struct ContactConstraint;


struct ContactListener
{
	void *user_data;
	void (*begin_contact)(void *user_data, const ContactConstraint *contact);
	void (*end_contact)  (void *user_data, const ContactConstraint *contact);
};


typedef int (*PhysicsWorldQueryCallback)(void *user_data, PhysicsShape *shape);


/* Registered water volumes; one per water surface. */
#define PHYSICS_MAX_BUOYANCY_VOLUMES 2


struct PhysicsWorld
{
	ContactManager        contact_manager;
	PhysicsPagedAllocator shape_allocator;

	int32_t               body_count;
	RigidBody            *body_list;

	int32_t               cloth_count;
	Cloth                *cloth_list;

	const BuoyancyVolume *buoyancy[PHYSICS_MAX_BUOYANCY_VOLUMES];
	int32_t               buoyancy_count;

	PhysicsStack          stack;
	PhysicsHeap           heap;

	Vector3               gravity;
	Vector3               wind;      /* pushes cloths only; write it per frame */
	Fixed                 dt;
	Fixed                 accumulator;   /* cloth clock; rigid bodies step on the frame's dt */
	int32_t               iterations;

	int                   new_shape;
	int                   allow_sleep;
	int                   enable_friction;

	ContactListener      *contact_listener;


	/* 'capacity' is how many bodies and cloths, together, the world will
	   ever hold: the heap they live in is taken once, sized to exactly that. */
	void init(Fixed dt, const Vector3 &gravity, int32_t iterations, int32_t capacity);
	void shutdown();

	/* Advances the world by the frame's elapsed time. Call this one, not
	   step, from the game loop: stepping once per frame ties the
	   simulation's speed to the framerate. */
	void update(Fixed delta);

	void step();

	RigidBody *createBody(const RigidBodyDef *def);
	void       removeBody(RigidBody *body);
	void       removeAllBodies();

	/* Cloths are stepped by step along with the bodies. The def names the
	   welded collision mesh that seeds the particles; it is loaded here,
	   read, and dropped, so the caller never handles it. */
	Cloth *createCloth(const ClothDef *def);
	void   removeCloth(Cloth *cloth);
	void   removeAllCloths();

	void setAllowSleep(int allow_sleep);
	void setIterations(int32_t iterations);
	void setEnableFriction(int enabled) { enable_friction = enabled; }

	Vector3 getGravity() const           { return gravity; }
	void    setGravity(const Vector3 &g) { gravity = g; }

	/* Only cloths feel it. Meant to be rewritten every frame, gusts included. */
	void setWind(const Vector3 &w) { wind = w; }

	/* The volume is borrowed, not copied: its owner keeps it alive for the
	   world's lifetime. Buoyancy runs inside step on every dynamic body
	   overlapping the volume's sensor shape. */
	void addBuoyancy(const BuoyancyVolume *volume);

	void setContactListener(ContactListener *listener);

	void queryAABB (void *cb_user_data, PhysicsWorldQueryCallback cb, const AABB &aabb) const;
	void queryPoint(void *cb_user_data, PhysicsWorldQueryCallback cb, const Vector3 &point) const;
	void rayCast   (void *cb_user_data, PhysicsWorldQueryCallback cb, RaycastData *ray) const;

	/* Shapes live in the world's pool; the bodies take and return them here. */
	PhysicsShape *allocShape();
	void          freeShape(PhysicsShape *shape);
	void          markNewShape() { new_shape = 1; }
};


#endif
