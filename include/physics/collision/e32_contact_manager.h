/*
	Ported from qu3e q3ContactManager.h — altered source, not the original software.

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
	Owns the list of ContactConstraints and the BroadPhase.
*/
#ifndef ENGINE_32_CONTACT_MANAGER_H
#define ENGINE_32_CONTACT_MANAGER_H

#include <stdint.h>

#include "physics/broadphase/e32_broad_phase.h"
#include "physics/memory/e32_physics_paged_allocator.h"


struct ContactConstraint;
struct PhysicsShape;
struct RigidBody;
struct PhysicsStack;


struct ContactManager
{
	ContactConstraint     *contact_list;
	int32_t                contact_count;
	PhysicsStack          *stack;
	PhysicsPagedAllocator  allocator;
	BroadPhase             broadphase;
	void                  *contact_listener;

	void init(PhysicsStack *stack);
	void shutdown();

	void addContact(PhysicsShape *A, PhysicsShape *B);
	void findNewContacts();
	void removeContact(ContactConstraint *contact);
	void removeContactsFromBody(RigidBody *body);
	void removeFromBroadphase(RigidBody *body);
	void testCollisions();
};


#endif
