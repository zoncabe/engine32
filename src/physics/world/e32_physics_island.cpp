/*
	Ported from qu3e q3Island.cpp — altered source, not the original software.

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
	Integrate velocities, run the solver, integrate positions, manage sleep.
*/
#include "psyqo/kernel.hh"

#include "physics/world/e32_physics_island.h"
#include "physics/body/e32_rigid_body.h"
#include "physics/collision/e32_contact.h"
#include "physics/collision/e32_contact_solver.h"


void PhysicsIsland::addBody(RigidBody *body)
{
	psyqo::Kernel::assert(body_count < body_capacity, "physics: island full");
	body->island_index = body_count;
	bodies[body_count++] = body;
}


void PhysicsIsland::addContact(ContactConstraint *contact)
{
	psyqo::Kernel::assert(contact_count < contact_capacity, "physics: island full");
	contacts[contact_count++] = contact;
}


void PhysicsIsland::initialize()
{
	for (int32_t i = 0; i < contact_count; ++i) {
		ContactConstraint      *cc = contacts[i];
		ContactConstraintState *c  = contact_states + i;

		c->center_a     = cc->body_a->world_center;
		c->center_b     = cc->body_b->world_center;
		c->iA           = cc->body_a->inv_inertia_world;
		c->iB           = cc->body_b->inv_inertia_world;
		c->mA           = cc->body_a->inv_mass;
		c->mB           = cc->body_b->inv_mass;
		c->restitution  = cc->restitution;
		c->friction     = cc->friction;
		c->index_a      = cc->body_a->island_index;
		c->index_b      = cc->body_b->island_index;
		c->normal       = cc->manifold.normal;
		c->tangent_vectors[0] = cc->manifold.tangent_vectors[0];
		c->tangent_vectors[1] = cc->manifold.tangent_vectors[1];
		c->contact_count = cc->manifold.contact_count;

		for (int32_t j = 0; j < c->contact_count; ++j) {
			ContactState *s  = c->contacts + j;
			ContactPoint *cp = cc->manifold.contacts + j;
			s->ra = cp->position - c->center_a;
			s->rb = cp->position - c->center_b;
			s->penetration        = cp->penetration;
			s->normal_impulse     = cp->normal_impulse;
			s->tangent_impulse[0] = cp->tangent_impulse[0];
			s->tangent_impulse[1] = cp->tangent_impulse[1];
		}
	}
}


void PhysicsIsland::solve()
{
	/* Integrate forces into velocity. */
	for (int32_t i = 0; i < body_count; ++i) {
		RigidBody     *body = bodies[i];
		VelocityState *v    = velocities + i;

		if (body->flags & BODY_FLAG_DYNAMIC) {
			body->applyLinearForce(gravity * body->gravity_scale);

			/* iW = R · iModel · Rᵀ. */
			const Matrix3 &r = body->tx.rotation;
			body->inv_inertia_world = r * body->inv_inertia_model * r.transposed();

			body->linear_velocity  += body->force * (body->inv_mass * dt);
			body->angular_velocity += body->inv_inertia_world.transform(body->torque) * dt;

			/* Pade damping. */
			Fixed lin_d = 1.0_fp / (1.0_fp + dt * body->linear_damping);
			Fixed ang_d = 1.0_fp / (1.0_fp + dt * body->angular_damping);
			body->linear_velocity  *= lin_d;
			body->angular_velocity *= ang_d;
		}

		v->v = body->linear_velocity;
		v->w = body->angular_velocity;
	}

	/* Contact solver. */
	ContactSolver solver;
	solver.initialize(this);
	solver.preSolve(dt);

	for (int32_t i = 0; i < iterations; ++i) {
		solver.solve();
	}

	solver.shutdown();

	/* Integrate positions. */
	for (int32_t i = 0; i < body_count; ++i) {
		RigidBody     *body = bodies[i];
		VelocityState *v    = velocities + i;

		if (body->flags & BODY_FLAG_STATIC) continue;

		/* A kinematic body is placed by whoever owns it, not by the solver: it
		   carries a velocity so contacts know how hard it pushes, but
		   integrating that velocity here would advance it a second time. */
		if (body->flags & BODY_FLAG_KINEMATIC) continue;

		body->linear_velocity  = v->v;
		body->angular_velocity = v->w;

		body->world_center += body->linear_velocity * dt;

		/* integrate normalizes on its own way out. */
		body->q.integrate(body->angular_velocity, dt);
		body->tx.rotation = body->q.toMatrix3();
	}

	/* Sleep management. */
	if (allow_sleep) {
		Fixed min_sleep = Fixed(0x7FFFFFFF, Fixed::RAW);
		for (int32_t i = 0; i < body_count; ++i) {
			RigidBody *body = bodies[i];
			if (body->flags & BODY_FLAG_STATIC) continue;

			/* Kinematics move on someone else's schedule, so their stillness
			   says nothing about whether the island can rest. */
			if (body->flags & BODY_FLAG_KINEMATIC) continue;

			Fixed sqr_lin = body->linear_velocity.dot(body->linear_velocity);
			Fixed sqr_ang = body->angular_velocity.dot(body->angular_velocity);

			if (sqr_lin > PHYSICS_SLEEP_LINEAR || sqr_ang > PHYSICS_SLEEP_ANGULAR) {
				min_sleep = 0;
				body->sleep_time = 0;
			} else {
				body->sleep_time += dt;
				if (body->sleep_time < min_sleep) min_sleep = body->sleep_time;
			}
		}

		if (min_sleep > PHYSICS_SLEEP_TIME) {
			for (int32_t i = 0; i < body_count; ++i) {
				/* Asleep it would stop seeding islands, and the bodies resting
				   against it would never learn that it started moving again. */
				if (bodies[i]->flags & BODY_FLAG_KINEMATIC) continue;

				bodies[i]->setToSleep();
			}
		}
	}
}
