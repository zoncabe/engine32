/*
	Ported from qu3e q3Scene.cpp — altered source, not the original software.

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
	World assembly, step, body and shape management.
*/
#include <stddef.h>

#include "psyqo/kernel.hh"

#include "physics/world/e32_physics_world.h"
#include "physics/world/e32_physics_island.h"
#include "physics/collision/e32_contact.h"
#include "physics/collision/e32_contact_solver.h"
#include "physics/collision/e32_contact_manager.h"
#include "physics/collision/e32_collision.h"
#include "physics/collision/e32_collision_mesh.h"
#include "physics/broadphase/e32_broad_phase.h"


PhysicsShape *PhysicsWorld::allocShape()
{
	return (PhysicsShape *)physicsPagedAllocator_allocate(&shape_allocator);
}

void PhysicsWorld::freeShape(PhysicsShape *shape)
{
	physicsPagedAllocator_free(&shape_allocator, shape);
}


void PhysicsWorld::init(Fixed step_dt, const Vector3 &g, int32_t iters, int32_t capacity)
{
	/* A RigidBody is the larger of the two things the heap holds, so one
	   slot of that size, header included, fits either. */
	physicsStack_init(&stack);
	physicsHeap_init (&heap, capacity * (int32_t)(sizeof(RigidBody) + sizeof(PhysicsHeader)));
	contact_manager.init(&stack);
	physicsPagedAllocator_init(&shape_allocator, (int32_t)sizeof(PhysicsShape), 256);

	body_count       = 0;
	body_list        = NULL;
	cloth_count      = 0;
	cloth_list       = NULL;
	buoyancy_count   = 0;
	wind             = Vector3::zero();
	gravity          = g;
	dt               = step_dt;
	accumulator      = 0;
	iterations       = iters;
	new_shape        = 0;
	allow_sleep      = 1;
	enable_friction  = 1;
	contact_listener = NULL;
}


void PhysicsWorld::removeAllBodies()
{
	RigidBody *body = body_list;
	while (body) {
		RigidBody *next = body->next;
		body->removeAllShapes();
		physicsHeap_free(&heap, body);
		body = next;
	}
	body_list  = NULL;
	body_count = 0;
}


Cloth *PhysicsWorld::createCloth(const ClothDef *def)
{
	Cloth *cloth = (Cloth *)physicsHeap_allocate(&heap, sizeof(Cloth));
	if (cloth == NULL) return NULL;

	/* The mesh is scaffolding: it seeds the particles and the constraints, and
	   nothing keeps a reference to it afterwards. */
	CollisionMesh *mesh = CollisionMesh::load(def->mesh_path);
	bool built = cloth->create(mesh, def);
	mesh->free();

	if (!built) {
		physicsHeap_free(&heap, cloth);
		return NULL;
	}

	cloth->gravity = gravity;
	cloth->wind    = wind;

	cloth->next = cloth_list;
	cloth_list  = cloth;
	cloth_count++;

	return cloth;
}


void PhysicsWorld::removeCloth(Cloth *cloth)
{
	for (Cloth **link = &cloth_list; *link; link = &(*link)->next) {
		if (*link != cloth) continue;

		*link = cloth->next;
		cloth->free();
		physicsHeap_free(&heap, cloth);
		cloth_count--;
		return;
	}
}


void PhysicsWorld::removeAllCloths()
{
	Cloth *cloth = cloth_list;
	while (cloth) {
		Cloth *next = cloth->next;
		cloth->free();
		physicsHeap_free(&heap, cloth);
		cloth = next;
	}
	cloth_list  = NULL;
	cloth_count = 0;
}


void PhysicsWorld::shutdown()
{
	removeAllCloths();
	removeAllBodies();
	physicsPagedAllocator_shutdown(&shape_allocator);
	contact_manager.shutdown();
	physicsHeap_shutdown(&heap);
	physicsStack_shutdown(&stack);
}


/* One step per rendered frame, on the frame's own clock. The clamp is what
   keeps a hiccup from becoming one huge step: past it the simulation runs in
   slow motion for that frame instead of blowing up the solver. */
void PhysicsWorld::update(Fixed delta)
{
	if (delta <= 0) return;   /* first frame; 1/dt lives in the solver bias */

	dt = (delta > PHYSICS_MAX_TIMESTEP) ? PHYSICS_MAX_TIMESTEP : delta;
	step();

	/* Cloths keep the fixed step: a Verlet cloth's look is tuned to its step
	   size, so it steps on its own clock instead of the frame's dt. Its cost
	   per second is constant, so it cannot feed back into the frame time. */
	accumulator += delta;

	Fixed ceiling = PHYSICS_TIMESTEP * PHYSICS_CLOTH_MAX_SUBSTEPS;
	if (accumulator > ceiling) accumulator = ceiling;

	/* A cloth nobody is looking at holds its pose: skipping its step leaves
	   both Verlet slots alone, so it resumes with the velocity it had. The
	   accumulator is the world's and drains either way, so coming back into
	   view never owes a burst of substeps. */
	while (accumulator >= PHYSICS_TIMESTEP) {
		for (Cloth *cloth = cloth_list; cloth; cloth = cloth->next) {
			if (cloth->isCulled()) continue;
			cloth->gravity = gravity;
			cloth->wind    = wind;
			cloth->step(PHYSICS_TIMESTEP);
		}
		accumulator -= PHYSICS_TIMESTEP;
	}

	/* Shown state: previous and current step blended by the leftover
	   fraction, same scheme as the dynamic bones. */
	Fixed t = accumulator / PHYSICS_TIMESTEP;
	for (Cloth *cloth = cloth_list; cloth; cloth = cloth->next)
		if (!cloth->isCulled()) cloth->blendRenderState(t);
}


void PhysicsWorld::step()
{
	if (new_shape) {
		contact_manager.broadphase.updatePairs();
		new_shape = 0;
	}

	contact_manager.testCollisions();

	/* After the narrowphase, so the sensors' COLLIDING flags are fresh;
	   before the islands, so the forces integrate in this same step. */
	for (int32_t i = 0; i < buoyancy_count; i++)
		buoyancy[i]->apply(this);

	for (RigidBody *body = body_list; body; body = body->next) {
		body->flags &= ~BODY_FLAG_ISLAND;
	}

	/* Reserve stack for island buffers. */
	physicsStack_reserve(&stack,
		(uint32_t)(sizeof(RigidBody *)             * body_count
		         + sizeof(VelocityState)           * body_count
		         + sizeof(ContactConstraint *)     * contact_manager.contact_count
		         + sizeof(ContactConstraintState)  * contact_manager.contact_count
		         + sizeof(RigidBody *)             * body_count)
	);

	PhysicsIsland island;
	island.body_capacity    = body_count;
	island.contact_capacity = contact_manager.contact_count;
	island.bodies           = (RigidBody **)            physicsStack_allocate(&stack, (int32_t)(sizeof(RigidBody *) * body_count));
	island.velocities       = (VelocityState *)         physicsStack_allocate(&stack, (int32_t)(sizeof(VelocityState) * body_count));
	island.contacts         = (ContactConstraint **)    physicsStack_allocate(&stack, (int32_t)(sizeof(ContactConstraint *) * island.contact_capacity));
	island.contact_states   = (ContactConstraintState *)physicsStack_allocate(&stack, (int32_t)(sizeof(ContactConstraintState) * island.contact_capacity));
	island.allow_sleep      = allow_sleep;
	island.enable_friction  = enable_friction;
	island.body_count       = 0;
	island.contact_count    = 0;
	island.dt               = dt;
	island.gravity          = gravity;
	island.iterations       = iterations;

	int32_t stack_size = body_count;
	RigidBody **seeds = (RigidBody **)physicsStack_allocate(&stack, (int32_t)(sizeof(RigidBody *) * stack_size));

	for (RigidBody *seed = body_list; seed; seed = seed->next) {
		if (seed->flags & BODY_FLAG_ISLAND) continue;
		if (!(seed->flags & BODY_FLAG_AWAKE)) continue;
		if (seed->flags & BODY_FLAG_STATIC) continue;

		int32_t stack_count = 0;
		seeds[stack_count++] = seed;
		island.body_count    = 0;
		island.contact_count = 0;

		seed->flags |= BODY_FLAG_ISLAND;

		while (stack_count > 0) {
			RigidBody *body = seeds[--stack_count];
			island.addBody(body);

			body->setToAwake();

			if (body->flags & BODY_FLAG_STATIC) continue;

			for (ContactEdge *edge = body->contact_list; edge; edge = edge->next) {
				ContactConstraint *contact = edge->constraint;

				if (contact->flags & CONSTRAINT_ISLAND) continue;
				if (!(contact->flags & CONSTRAINT_COLLIDING)) continue;
				if (contact->A->sensor || contact->B->sensor) continue;

				contact->flags |= CONSTRAINT_ISLAND;
				island.addContact(contact);

				RigidBody *other = edge->other;
				if (other->flags & BODY_FLAG_ISLAND) continue;

				psyqo::Kernel::assert(stack_count < stack_size, "physics: island stack overflow");
				seeds[stack_count++] = other;
				other->flags |= BODY_FLAG_ISLAND;
			}
		}

		psyqo::Kernel::assert(island.body_count != 0, "physics: empty island");

		island.initialize();
		island.solve();

		/* Reset static island flag so statics can participate in multiple islands. */
		for (int32_t i = 0; i < island.body_count; ++i) {
			RigidBody *body = island.bodies[i];
			if (body->flags & BODY_FLAG_STATIC) body->flags &= ~BODY_FLAG_ISLAND;
		}
	}

	physicsStack_free(&stack, seeds);
	physicsStack_free(&stack, island.contact_states);
	physicsStack_free(&stack, island.contacts);
	physicsStack_free(&stack, island.velocities);
	physicsStack_free(&stack, island.bodies);

	/* Sync broadphase AABBs. A sleeping body did not move, so its proxy is
	   already where it belongs: re-inserting it into the tree every substep is
	   what makes a settled scene keep costing, and settled is the normal case. */
	for (RigidBody *body = body_list; body; body = body->next) {
		if (body->flags & BODY_FLAG_STATIC) continue;
		if (!(body->flags & BODY_FLAG_AWAKE)) continue;

		body->synchronizeProxies();
	}

	contact_manager.findNewContacts();

	for (RigidBody *body = body_list; body; body = body->next) {
		body->force  = Vector3::zero();
		body->torque = Vector3::zero();
	}

}


RigidBody *PhysicsWorld::createBody(const RigidBodyDef *def)
{
	RigidBody *body = (RigidBody *)physicsHeap_allocate(&heap, (int32_t)sizeof(RigidBody));

	/* The heap holds exactly the capacity the world was created with and
	   hands back NULL past it. Writing the body through that NULL corrupts
	   low memory and then the world walks a list holding it, which surfaces
	   far from here as garbage in the solver. */
	psyqo::Kernel::assert(body != NULL, "physics: more bodies than the world was sized for");
	body->init(def, this);

	body->prev = NULL;
	body->next = body_list;
	if (body_list) body_list->prev = body;
	body_list = body;
	++body_count;
	return body;
}


void PhysicsWorld::removeBody(RigidBody *body)
{
	psyqo::Kernel::assert(body_count > 0, "physics: remove on an empty world");

	contact_manager.removeContactsFromBody(body);
	body->removeAllShapes();

	if (body->next) body->next->prev = body->prev;
	if (body->prev) body->prev->next = body->next;
	if (body == body_list) body_list = body->next;
	--body_count;

	physicsHeap_free(&heap, body);
}


void PhysicsWorld::setAllowSleep(int allow)
{
	allow_sleep = allow;
	if (!allow) {
		for (RigidBody *body = body_list; body; body = body->next) body->setToAwake();
	}
}


void PhysicsWorld::setIterations(int32_t iters)
{
	iterations = (iters > 1) ? iters : 1;
}


void PhysicsWorld::addBuoyancy(const BuoyancyVolume *volume)
{
	if (buoyancy_count >= PHYSICS_MAX_BUOYANCY_VOLUMES) return;
	buoyancy[buoyancy_count++] = volume;
}


void PhysicsWorld::setContactListener(ContactListener *listener)
{
	contact_listener = listener;
	contact_manager.contact_listener = listener;
}


struct QueryAABB_ctx
{
	const BroadPhase         *broadphase;
	PhysicsWorldQueryCallback cb;
	void                     *cb_user_data;
	AABB                      aabb;
};

static int queryAABB_cb(void *ctx_v, int32_t id)
{
	QueryAABB_ctx *ctx = (QueryAABB_ctx *)ctx_v;
	PhysicsShape *shape = (PhysicsShape *)ctx->broadphase->tree.userData(id);
	if (ctx->aabb.overlaps(shape->computeAABB())) {
		return ctx->cb(ctx->cb_user_data, shape);
	}
	return 1;
}

void PhysicsWorld::queryAABB(void *cb_user_data, PhysicsWorldQueryCallback cb, const AABB &aabb) const
{
	QueryAABB_ctx ctx;
	ctx.broadphase   = &contact_manager.broadphase;
	ctx.cb           = cb;
	ctx.cb_user_data = cb_user_data;
	ctx.aabb         = aabb;
	contact_manager.broadphase.tree.queryAABB(&ctx, queryAABB_cb, aabb);
}


struct QueryPoint_ctx
{
	const BroadPhase         *broadphase;
	PhysicsWorldQueryCallback cb;
	void                     *cb_user_data;
	Vector3                   point;
};

static int queryPoint_cb(void *ctx_v, int32_t id)
{
	QueryPoint_ctx *ctx = (QueryPoint_ctx *)ctx_v;
	PhysicsShape *shape = (PhysicsShape *)ctx->broadphase->tree.userData(id);
	if (shape->testPoint(ctx->point)) {
		ctx->cb(ctx->cb_user_data, shape);
	}
	return 1;
}

void PhysicsWorld::queryPoint(void *cb_user_data, PhysicsWorldQueryCallback cb, const Vector3 &point) const
{
	QueryPoint_ctx ctx;
	ctx.broadphase   = &contact_manager.broadphase;
	ctx.cb           = cb;
	ctx.cb_user_data = cb_user_data;
	ctx.point        = point;

	const Fixed k_fattener = 0.5_fp;
	Vector3 v = { k_fattener, k_fattener, k_fattener };
	AABB aabb = { point - v, point + v };
	contact_manager.broadphase.tree.queryAABB(&ctx, queryPoint_cb, aabb);
}


struct QueryRaycast_ctx
{
	const BroadPhase         *broadphase;
	PhysicsWorldQueryCallback cb;
	void                     *cb_user_data;
	RaycastData              *ray;
};

static int queryRaycast_cb(void *ctx_v, int32_t id)
{
	QueryRaycast_ctx *ctx = (QueryRaycast_ctx *)ctx_v;
	PhysicsShape *shape = (PhysicsShape *)ctx->broadphase->tree.userData(id);
	if (shape->raycast(ctx->ray)) {
		return ctx->cb(ctx->cb_user_data, shape);
	}
	return 1;
}

void PhysicsWorld::rayCast(void *cb_user_data, PhysicsWorldQueryCallback cb, RaycastData *ray) const
{
	QueryRaycast_ctx ctx;
	ctx.broadphase   = &contact_manager.broadphase;
	ctx.cb           = cb;
	ctx.cb_user_data = cb_user_data;
	ctx.ray          = ray;
	contact_manager.broadphase.tree.queryRay(&ctx, queryRaycast_cb, ray);
}
