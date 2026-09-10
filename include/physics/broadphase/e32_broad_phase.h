/*
	Ported from qu3e q3BroadPhase.h — altered source, not the original software.

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
	Pair-finding broadphase over a dynamic AABB tree.
*/
#ifndef ENGINE_32_BROAD_PHASE_H
#define ENGINE_32_BROAD_PHASE_H

#include <stdint.h>

#include "physics/geometry/e32_aabb.h"
#include "physics/broadphase/e32_dynamic_aabb_tree.h"


struct ContactManager;
struct PhysicsShape;


struct ContactPair
{
	int32_t A;
	int32_t B;
};


struct BroadPhase
{
	ContactManager *manager;

	ContactPair *pair_buffer;
	int32_t      pair_count;
	int32_t      pair_capacity;

	int32_t     *move_buffer;
	int32_t      move_count;
	int32_t      move_capacity;

	DynamicAABBTree tree;
	int32_t         current_index;

	void init(ContactManager *manager);
	void shutdown();

	void insertShape(PhysicsShape *shape, const AABB &aabb);
	void removeShape(const PhysicsShape *shape);
	void updatePairs();
	void update(int32_t id, const AABB &aabb);
	bool testOverlap(int32_t A, int32_t B) const;

	/* The tree query's leaf callback: cb is the BroadPhase. */
	static int treeCallback(void *cb, int32_t index);
};


#endif
