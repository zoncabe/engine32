/*
	Ported from qu3e q3DynamicAABBTree.h — altered source, not the original software.

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
	Bounding-volume hierarchy for broadphase queries. The C++ template
	Query<T> becomes a function-pointer callback.
*/
#ifndef ENGINE_32_DYNAMIC_AABB_TREE_H
#define ENGINE_32_DYNAMIC_AABB_TREE_H

#include <stdint.h>

#include "physics/geometry/e32_aabb.h"
#include "physics/geometry/e32_raycast.h"


#define PHYSICS_TREE_NULL (-1)


struct DynamicAABBTreeNode
{
	AABB    aabb;
	int32_t parent_or_next;   /* parent (active) / next pointer (free list) */
	int32_t left;
	int32_t right;
	void   *user_data;
	int32_t height;            /* leaf = 0, free = -1 */

	constexpr bool isLeaf() const { return right == PHYSICS_TREE_NULL; }
};


/* Called per leaf a query reaches; returning 0 stops the query. */
typedef int (*PhysicsQueryCallback)(void *cb, int32_t id);


struct DynamicAABBTree
{
	int32_t              root;
	DynamicAABBTreeNode *nodes;
	int32_t              count;
	int32_t              capacity;
	int32_t              free_list;

	void init();
	void shutdown();

	int32_t insert(const AABB &aabb, void *user_data);
	void    remove(int32_t id);
	bool    update(int32_t id, const AABB &aabb);

	void *userData(int32_t id) const;
	AABB  fatAABB (int32_t id) const;

	void queryAABB(void *cb, PhysicsQueryCallback callback, const AABB &aabb) const;
	void queryRay (void *cb, PhysicsQueryCallback callback, RaycastData *ray) const;
};


#endif
