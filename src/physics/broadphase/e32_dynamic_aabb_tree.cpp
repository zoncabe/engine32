/*
	Ported from qu3e q3DynamicAABBTree.cpp — altered source, not the original software.

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
	BVH for broadphase. Nodes live in a pool with a free list, insert uses a
	surface-area heuristic, and balancing is single-rotation AVL-style.
*/
#include <stddef.h>

#include "psyqo/kernel.hh"

#include "physics/broadphase/e32_dynamic_aabb_tree.h"
#include "physics/memory/e32_physics_memory.h"
#include "physics/e32_physics_settings.h"


static inline int imax(int a, int b) { return a > b ? a : b; }


static inline void fattenAABB(AABB *aabb)
{
	Vector3 v = { PHYSICS_AABB_FATTENER, PHYSICS_AABB_FATTENER, PHYSICS_AABB_FATTENER };
	aabb->min -= v;
	aabb->max += v;
}


static void addToFreeList(DynamicAABBTree *t, int32_t index)
{
	for (int32_t i = index; i < t->capacity - 1; ++i) {
		t->nodes[i].parent_or_next = i + 1;
		t->nodes[i].height         = PHYSICS_TREE_NULL;
	}
	t->nodes[t->capacity - 1].parent_or_next = PHYSICS_TREE_NULL;
	t->nodes[t->capacity - 1].height         = PHYSICS_TREE_NULL;
	t->free_list = index;
}


static int32_t allocateNode(DynamicAABBTree *t)
{
	if (t->free_list == PHYSICS_TREE_NULL) {
		t->capacity *= 2;
		DynamicAABBTreeNode *new_nodes = (DynamicAABBTreeNode *)
			physics_alloc((int32_t)(sizeof(DynamicAABBTreeNode) * t->capacity));
		psyqo::Kernel::assert(new_nodes != NULL, "physics: out of memory");
		__builtin_memcpy(new_nodes, t->nodes, sizeof(DynamicAABBTreeNode) * (size_t)t->count);
		physics_free(t->nodes);
		t->nodes = new_nodes;
		addToFreeList(t, t->count);
	}

	int32_t free_node = t->free_list;
	t->free_list = t->nodes[t->free_list].parent_or_next;
	t->nodes[free_node].height         = 0;
	t->nodes[free_node].left           = PHYSICS_TREE_NULL;
	t->nodes[free_node].right          = PHYSICS_TREE_NULL;
	t->nodes[free_node].parent_or_next = PHYSICS_TREE_NULL;
	t->nodes[free_node].user_data      = NULL;
	++t->count;
	return free_node;
}


static void deallocateNode(DynamicAABBTree *t, int32_t index)
{
	psyqo::Kernel::assert(index >= 0 && index < t->capacity, "physics: tree node out of range");
	t->nodes[index].parent_or_next = t->free_list;
	t->nodes[index].height         = PHYSICS_TREE_NULL;
	t->free_list = index;
	--t->count;
}


static int32_t balance(DynamicAABBTree *t, int32_t iA)
{
	DynamicAABBTreeNode *A = t->nodes + iA;
	if (A->isLeaf() || A->height == 1) return iA;

	int32_t iB = A->left;
	int32_t iC = A->right;
	DynamicAABBTreeNode *B = t->nodes + iB;
	DynamicAABBTreeNode *C = t->nodes + iC;

	int32_t bal = C->height - B->height;

	if (bal > 1) {
		int32_t iF = C->left;
		int32_t iG = C->right;
		DynamicAABBTreeNode *F = t->nodes + iF;
		DynamicAABBTreeNode *G = t->nodes + iG;

		if (A->parent_or_next != PHYSICS_TREE_NULL) {
			if (t->nodes[A->parent_or_next].left == iA)
				t->nodes[A->parent_or_next].left = iC;
			else
				t->nodes[A->parent_or_next].right = iC;
		} else {
			t->root = iC;
		}

		C->left           = iA;
		C->parent_or_next = A->parent_or_next;
		A->parent_or_next = iC;

		if (F->height > G->height) {
			C->right          = iF;
			A->right          = iG;
			G->parent_or_next = iA;
			A->aabb           = B->aabb.combined(G->aabb);
			C->aabb           = A->aabb.combined(F->aabb);
			A->height         = 1 + imax(B->height, G->height);
			C->height         = 1 + imax(A->height, F->height);
		} else {
			C->right          = iG;
			A->right          = iF;
			F->parent_or_next = iA;
			A->aabb           = B->aabb.combined(F->aabb);
			C->aabb           = A->aabb.combined(G->aabb);
			A->height         = 1 + imax(B->height, F->height);
			C->height         = 1 + imax(A->height, G->height);
		}
		return iC;
	}
	else if (bal < -1) {
		int32_t iD = B->left;
		int32_t iE = B->right;
		DynamicAABBTreeNode *D = t->nodes + iD;
		DynamicAABBTreeNode *E = t->nodes + iE;

		if (A->parent_or_next != PHYSICS_TREE_NULL) {
			if (t->nodes[A->parent_or_next].left == iA)
				t->nodes[A->parent_or_next].left = iB;
			else
				t->nodes[A->parent_or_next].right = iB;
		} else {
			t->root = iB;
		}

		B->right          = iA;
		B->parent_or_next = A->parent_or_next;
		A->parent_or_next = iB;

		if (D->height > E->height) {
			B->left           = iD;
			A->left           = iE;
			E->parent_or_next = iA;
			A->aabb           = C->aabb.combined(E->aabb);
			B->aabb           = A->aabb.combined(D->aabb);
			A->height         = 1 + imax(C->height, E->height);
			B->height         = 1 + imax(A->height, D->height);
		} else {
			B->left           = iE;
			A->left           = iD;
			D->parent_or_next = iA;
			A->aabb           = C->aabb.combined(D->aabb);
			B->aabb           = A->aabb.combined(E->aabb);
			A->height         = 1 + imax(C->height, D->height);
			B->height         = 1 + imax(A->height, E->height);
		}
		return iB;
	}
	return iA;
}


static void syncHierarchy(DynamicAABBTree *t, int32_t index)
{
	while (index != PHYSICS_TREE_NULL) {
		index = balance(t, index);
		int32_t left  = t->nodes[index].left;
		int32_t right = t->nodes[index].right;

		t->nodes[index].height = 1 + imax(t->nodes[left].height, t->nodes[right].height);
		t->nodes[index].aabb   = t->nodes[left].aabb.combined(t->nodes[right].aabb);

		index = t->nodes[index].parent_or_next;
	}
}


static void insertLeaf(DynamicAABBTree *t, int32_t id)
{
	if (t->root == PHYSICS_TREE_NULL) {
		t->root = id;
		t->nodes[t->root].parent_or_next = PHYSICS_TREE_NULL;
		return;
	}

	int32_t search = t->root;
	AABB leaf_aabb = t->nodes[id].aabb;
	while (!t->nodes[search].isLeaf()) {
		AABB  combined      = leaf_aabb.combined(t->nodes[search].aabb);
		Fixed combined_area = combined.surfaceArea();
		Fixed branch_cost   = combined_area * 2;
		Fixed inherited     = (combined_area - t->nodes[search].aabb.surfaceArea()) * 2;

		int32_t left  = t->nodes[search].left;
		int32_t right = t->nodes[search].right;

		Fixed left_cost;
		if (t->nodes[left].isLeaf()) {
			AABB c = leaf_aabb.combined(t->nodes[left].aabb);
			left_cost = c.surfaceArea() + inherited;
		} else {
			AABB  c = leaf_aabb.combined(t->nodes[left].aabb);
			Fixed inflated = c.surfaceArea();
			Fixed branch   = t->nodes[left].aabb.surfaceArea();
			left_cost = inflated - branch + inherited;
		}

		Fixed right_cost;
		if (t->nodes[right].isLeaf()) {
			AABB c = leaf_aabb.combined(t->nodes[right].aabb);
			right_cost = c.surfaceArea() + inherited;
		} else {
			AABB  c = leaf_aabb.combined(t->nodes[right].aabb);
			Fixed inflated = c.surfaceArea();
			Fixed branch   = t->nodes[right].aabb.surfaceArea();
			right_cost = inflated - branch + inherited;
		}

		if (branch_cost < left_cost && branch_cost < right_cost) break;
		search = (left_cost < right_cost) ? left : right;
	}

	int32_t sibling    = search;
	int32_t old_parent = t->nodes[sibling].parent_or_next;
	int32_t new_parent = allocateNode(t);

	t->nodes[new_parent].parent_or_next = old_parent;
	t->nodes[new_parent].user_data      = NULL;
	t->nodes[new_parent].aabb           = leaf_aabb.combined(t->nodes[sibling].aabb);
	t->nodes[new_parent].height         = t->nodes[sibling].height + 1;

	if (old_parent == PHYSICS_TREE_NULL) {
		t->nodes[new_parent].left        = sibling;
		t->nodes[new_parent].right       = id;
		t->nodes[sibling].parent_or_next = new_parent;
		t->nodes[id].parent_or_next      = new_parent;
		t->root = new_parent;
	} else {
		if (t->nodes[old_parent].left == sibling)
			t->nodes[old_parent].left = new_parent;
		else
			t->nodes[old_parent].right = new_parent;

		t->nodes[new_parent].left        = sibling;
		t->nodes[new_parent].right       = id;
		t->nodes[sibling].parent_or_next = new_parent;
		t->nodes[id].parent_or_next      = new_parent;
	}

	syncHierarchy(t, t->nodes[id].parent_or_next);
}


static void removeLeaf(DynamicAABBTree *t, int32_t id)
{
	if (id == t->root) {
		t->root = PHYSICS_TREE_NULL;
		return;
	}

	int32_t parent      = t->nodes[id].parent_or_next;
	int32_t grandparent = t->nodes[parent].parent_or_next;
	int32_t sibling     = (t->nodes[parent].left == id)
		? t->nodes[parent].right : t->nodes[parent].left;

	if (grandparent != PHYSICS_TREE_NULL) {
		if (t->nodes[grandparent].left == parent)
			t->nodes[grandparent].left = sibling;
		else
			t->nodes[grandparent].right = sibling;

		t->nodes[sibling].parent_or_next = grandparent;
	} else {
		t->root = sibling;
		t->nodes[sibling].parent_or_next = PHYSICS_TREE_NULL;
	}

	deallocateNode(t, parent);
	syncHierarchy(t, grandparent);
}


void DynamicAABBTree::init()
{
	root     = PHYSICS_TREE_NULL;
	capacity = 1024;
	count    = 0;
	nodes    = (DynamicAABBTreeNode *)physics_alloc((int32_t)(sizeof(DynamicAABBTreeNode) * capacity));
	psyqo::Kernel::assert(nodes != NULL, "physics: out of memory");
	addToFreeList(this, 0);
}


void DynamicAABBTree::shutdown()
{
	if (nodes) physics_free(nodes);
	nodes = NULL;
}


int32_t DynamicAABBTree::insert(const AABB &aabb, void *user_data)
{
	int32_t id = allocateNode(this);
	nodes[id].aabb = aabb;
	fattenAABB(&nodes[id].aabb);
	nodes[id].user_data = user_data;
	nodes[id].height    = 0;
	insertLeaf(this, id);
	return id;
}


void DynamicAABBTree::remove(int32_t id)
{
	psyqo::Kernel::assert(id >= 0 && id < capacity && nodes[id].isLeaf(), "physics: tree remove of no leaf");
	removeLeaf(this, id);
	deallocateNode(this, id);
}


bool DynamicAABBTree::update(int32_t id, const AABB &aabb)
{
	psyqo::Kernel::assert(id >= 0 && id < capacity && nodes[id].isLeaf(), "physics: tree update of no leaf");

	if (nodes[id].aabb.containsAABB(aabb)) return false;

	removeLeaf(this, id);
	nodes[id].aabb = aabb;
	fattenAABB(&nodes[id].aabb);
	insertLeaf(this, id);
	return true;
}


void *DynamicAABBTree::userData(int32_t id) const
{
	psyqo::Kernel::assert(id >= 0 && id < capacity, "physics: tree node out of range");
	return nodes[id].user_data;
}


AABB DynamicAABBTree::fatAABB(int32_t id) const
{
	psyqo::Kernel::assert(id >= 0 && id < capacity, "physics: tree node out of range");
	return nodes[id].aabb;
}


#define TREE_QUERY_STACK_CAP 256

/* A build that fills locals would fill this 1 KB stack on every query.
   Only the slots below sp are ever read, so both queries opt out. */

void DynamicAABBTree::queryAABB(void *cb, PhysicsQueryCallback callback, const AABB &aabb) const
{
	int32_t stack[TREE_QUERY_STACK_CAP] __attribute__((uninitialized));
	int32_t sp = 1;
	stack[0] = root;

	while (sp) {
		psyqo::Kernel::assert(sp < TREE_QUERY_STACK_CAP, "physics: tree query stack overflow");
		int32_t id = stack[--sp];
		if (id == PHYSICS_TREE_NULL) continue;

		const DynamicAABBTreeNode *n = nodes + id;
		if (aabb.overlaps(n->aabb)) {
			if (n->isLeaf()) {
				if (!callback(cb, id)) return;
			} else {
				stack[sp++] = n->left;
				stack[sp++] = n->right;
			}
		}
	}
}


void DynamicAABBTree::queryRay(void *cb, PhysicsQueryCallback callback, RaycastData *ray) const
{
	int32_t stack[TREE_QUERY_STACK_CAP] __attribute__((uninitialized));
	int32_t sp = 1;
	stack[0] = root;

	Vector3 p0 = ray->start;
	Vector3 p1 = p0 + ray->dir * ray->t;

	/* Segment terms, the same for every node visited. */
	Vector3 d     = p1 - p0;
	Vector3 p_sum = p0 + p1;
	Fixed   adx   = d.x.abs();
	Fixed   ady   = d.y.abs();
	Fixed   adz   = d.z.abs();
	Fixed   adx_e = adx + TOLERANCE;
	Fixed   ady_e = ady + TOLERANCE;
	Fixed   adz_e = adz + TOLERANCE;

	while (sp) {
		psyqo::Kernel::assert(sp < TREE_QUERY_STACK_CAP, "physics: tree query stack overflow");
		int32_t id = stack[--sp];
		if (id == PHYSICS_TREE_NULL) continue;

		const DynamicAABBTreeNode *n = nodes + id;

		Vector3 e = n->aabb.max - n->aabb.min;
		Vector3 m = p_sum - n->aabb.min - n->aabb.max;

		if (m.x.abs() > e.x + adx) continue;
		if (m.y.abs() > e.y + ady) continue;
		if (m.z.abs() > e.z + adz) continue;

		if ((m.y * d.z - m.z * d.y).abs() > e.y * adz_e + e.z * ady_e) continue;
		if ((m.z * d.x - m.x * d.z).abs() > e.x * adz_e + e.z * adx_e) continue;
		if ((m.x * d.y - m.y * d.x).abs() > e.x * ady_e + e.y * adx_e) continue;

		if (n->isLeaf()) {
			if (!callback(cb, id)) return;
		} else {
			stack[sp++] = n->left;
			stack[sp++] = n->right;
		}
	}
}
