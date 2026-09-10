/*
	Ported from qu3e q3ContactManager.cpp — altered source, not the original software.

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
	Maintains the list of active ContactConstraints, drives broadphase pair
	generation, and refreshes contacts each step.
*/
#include <stddef.h>

#include "physics/collision/e32_contact_manager.h"
#include "physics/collision/e32_contact.h"
#include "physics/body/e32_rigid_body.h"
#include "physics/shapes/e32_physics_shape.h"
#include "physics/geometry/e32_half_space.h"   /* vector3_computeBasis */
#include "physics/memory/e32_physics_stack.h"


void ContactManager::init(PhysicsStack *s)
{
	stack = s;
	/* A ContactConstraint carries a whole manifold, so it is over 500
	   bytes: the qu3e page of 256 would ask for 140 KB in one malloc, which
	   fragments the heap badly on a console with 2 MB. 32 keeps a page at
	   17 KB and the allocator simply adds another when a busy scene needs it. */
	physicsPagedAllocator_init(&allocator, (int32_t)sizeof(ContactConstraint), 32);
	broadphase.init(this);
	contact_list     = NULL;
	contact_count    = 0;
	contact_listener = NULL;
}


void ContactManager::shutdown()
{
	broadphase.shutdown();
	physicsPagedAllocator_shutdown(&allocator);
	contact_list  = NULL;
	contact_count = 0;
}


void ContactManager::addContact(PhysicsShape *A, PhysicsShape *B)
{
	RigidBody *body_a = A->body;
	RigidBody *body_b = B->body;
	if (!body_a->canCollide(body_b)) return;

	/* Dedup. */
	ContactEdge *edge = body_a->contact_list;
	while (edge) {
		if (edge->other == body_b) {
			PhysicsShape *shape_a = edge->constraint->A;
			PhysicsShape *shape_b = edge->constraint->B;
			if (A == shape_a && B == shape_b) return;
		}
		edge = edge->next;
	}

	ContactConstraint *contact = (ContactConstraint *)physicsPagedAllocator_allocate(&allocator);
	contact->A            = A;
	contact->B            = B;
	contact->body_a       = A->body;
	contact->body_b       = B->body;
	contact->manifold.setPair(A, B);
	contact->flags        = 0;
	contact->friction     = contact_mixFriction(A, B);
	contact->restitution  = contact_mixRestitution(A, B);
	contact->manifold.contact_count = 0;

	/* The allocator hands out raw malloc memory, and testCollisions runs
	   computeBasis on this normal whether or not the pair touched. Left as it
	   came, a fresh page from a previous scene makes it garbage: which is why
	   the first run survives and the second one does not. */
	contact->manifold.normal             = { 0.0_fp, 0.0_fp, 1.0_fp };
	contact->manifold.tangent_vectors[0] = { 1.0_fp, 0.0_fp, 0.0_fp };
	contact->manifold.tangent_vectors[1] = { 0.0_fp, 1.0_fp, 0.0_fp };

	for (int32_t i = 0; i < 8; ++i) contact->manifold.contacts[i].warm_started = 0;

	contact->prev = NULL;
	contact->next = contact_list;
	if (contact_list) contact_list->prev = contact;
	contact_list = contact;

	contact->edge_a.constraint = contact;
	contact->edge_a.other      = body_b;
	contact->edge_a.prev = NULL;
	contact->edge_a.next = body_a->contact_list;
	if (body_a->contact_list) body_a->contact_list->prev = &contact->edge_a;
	body_a->contact_list = &contact->edge_a;

	contact->edge_b.constraint = contact;
	contact->edge_b.other      = body_a;
	contact->edge_b.prev = NULL;
	contact->edge_b.next = body_b->contact_list;
	if (body_b->contact_list) body_b->contact_list->prev = &contact->edge_b;
	body_b->contact_list = &contact->edge_b;

	body_a->setToAwake();
	body_b->setToAwake();

	++contact_count;
}


void ContactManager::findNewContacts()
{
	broadphase.updatePairs();
}


void ContactManager::removeContact(ContactConstraint *contact)
{
	RigidBody *A = contact->body_a;
	RigidBody *B = contact->body_b;

	/* Remove from A. */
	if (contact->edge_a.prev) contact->edge_a.prev->next = contact->edge_a.next;
	if (contact->edge_a.next) contact->edge_a.next->prev = contact->edge_a.prev;
	if (&contact->edge_a == A->contact_list) A->contact_list = contact->edge_a.next;

	/* Remove from B. */
	if (contact->edge_b.prev) contact->edge_b.prev->next = contact->edge_b.next;
	if (contact->edge_b.next) contact->edge_b.next->prev = contact->edge_b.prev;
	if (&contact->edge_b == B->contact_list) B->contact_list = contact->edge_b.next;

	A->setToAwake();
	B->setToAwake();

	if (contact->prev) contact->prev->next = contact->next;
	if (contact->next) contact->next->prev = contact->prev;
	if (contact == contact_list) contact_list = contact->next;

	--contact_count;

	physicsPagedAllocator_free(&allocator, contact);
}


void ContactManager::removeContactsFromBody(RigidBody *body)
{
	ContactEdge *edge = body->contact_list;
	while (edge) {
		ContactEdge *next = edge->next;
		removeContact(edge->constraint);
		edge = next;
	}
}


void ContactManager::removeFromBroadphase(RigidBody *body)
{
	PhysicsShape *shape = body->shapes;
	while (shape) {
		broadphase.removeShape(shape);
		shape = shape->next;
	}
}


void ContactManager::testCollisions()
{
	ContactConstraint *constraint = contact_list;

	while (constraint) {
		PhysicsShape *A = constraint->A;
		PhysicsShape *B = constraint->B;
		RigidBody *body_a = A->body;
		RigidBody *body_b = B->body;

		constraint->flags &= ~CONSTRAINT_ISLAND;

		if (!body_a->isAwake() && !body_b->isAwake()) {
			constraint = constraint->next;
			continue;
		}

		if (!body_a->canCollide(body_b)) {
			ContactConstraint *next = constraint->next;
			removeContact(constraint);
			constraint = next;
			continue;
		}

		if (!broadphase.testOverlap(A->broadphase_index, B->broadphase_index)) {
			ContactConstraint *next = constraint->next;
			removeContact(constraint);
			constraint = next;
			continue;
		}

		ContactManifold *manifold = &constraint->manifold;

		/* Warm start needs only the previous impulses, keyed by feature, and
		   the tangents they were measured on: a copy of the whole manifold
		   would move 500 bytes per pair per step for these 130. */
		struct {
			uint32_t key;
			Fixed    normal_impulse;
			Fixed    tangent_impulse[2];
		} old_points[8] __attribute__((uninitialized));
		int32_t old_count = manifold->contact_count;
		for (int32_t j = 0; j < old_count; ++j) {
			const ContactPoint *oc = manifold->contacts + j;
			old_points[j].key                = oc->fp.key;
			old_points[j].normal_impulse     = oc->normal_impulse;
			old_points[j].tangent_impulse[0] = oc->tangent_impulse[0];
			old_points[j].tangent_impulse[1] = oc->tangent_impulse[1];
		}
		Vector3 ot0 = manifold->tangent_vectors[0];
		Vector3 ot1 = manifold->tangent_vectors[1];

		constraint->solveCollision();

		/* Tangents are read by the solver and by next step's warm start, both
		   gated by contact_count: a pair that does not touch skips the
		   normalization the basis costs. */
		if (!manifold->contact_count) {
			constraint = constraint->next;
			continue;
		}

		vector3_computeBasis(manifold->normal, &manifold->tangent_vectors[0], &manifold->tangent_vectors[1]);

		for (int32_t i = 0; i < manifold->contact_count; ++i) {
			ContactPoint *c = manifold->contacts + i;
			c->tangent_impulse[0] = 0;
			c->tangent_impulse[1] = 0;
			c->normal_impulse     = 0;
			uint8_t old_warm      = c->warm_started;
			c->warm_started       = 0;

			for (int32_t j = 0; j < old_count; ++j) {
				if (c->fp.key == old_points[j].key) {
					c->normal_impulse = old_points[j].normal_impulse;

					Vector3 friction = ot0 * old_points[j].tangent_impulse[0]
					                 + ot1 * old_points[j].tangent_impulse[1];
					c->tangent_impulse[0] = friction.dot(manifold->tangent_vectors[0]);
					c->tangent_impulse[1] = friction.dot(manifold->tangent_vectors[1]);
					uint8_t next_warm = (uint8_t)(old_warm + 1);
					c->warm_started = (old_warm > next_warm) ? old_warm : next_warm;
					break;
				}
			}
		}

		constraint = constraint->next;
	}
}
