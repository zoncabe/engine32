/*
	Ported from qu3e q3ContactSolver.cpp — altered source, not the original software.

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
	Sequential impulse solver (PGS).
*/
#include "physics/collision/e32_contact_solver.h"
#include "physics/collision/e32_contact.h"
#include "physics/world/e32_physics_island.h"
#include "physics/body/e32_rigid_body.h"


static inline Fixed invert_or_zero(Fixed x)
{
	return x.raw() != 0 ? 1.0_fp / x : Fixed(0);
}


void ContactSolver::initialize(PhysicsIsland *i)
{
	island          = i;
	contact_count   = i->contact_count;
	contacts        = i->contact_states;
	velocities      = i->velocities;
	enable_friction = i->enable_friction;
}


void ContactSolver::shutdown()
{
	for (int32_t i = 0; i < contact_count; ++i) {
		ContactConstraintState *c  = contacts + i;
		ContactConstraint      *cc = island->contacts[i];

		for (int32_t j = 0; j < c->contact_count; ++j) {
			ContactPoint *oc = cc->manifold.contacts + j;
			ContactState *cs = c->contacts + j;
			oc->normal_impulse     = cs->normal_impulse;
			oc->tangent_impulse[0] = cs->tangent_impulse[0];
			oc->tangent_impulse[1] = cs->tangent_impulse[1];
		}
	}
}


void ContactSolver::preSolve(Fixed dt)
{
	for (int32_t i = 0; i < contact_count; ++i) {
		ContactConstraintState *cs = contacts + i;

		Vector3 vA = velocities[cs->index_a].v;
		Vector3 wA = velocities[cs->index_a].w;
		Vector3 vB = velocities[cs->index_b].v;
		Vector3 wB = velocities[cs->index_b].w;

		/* A static or kinematic side has inv_mass 0 and a zero inverse
		   inertia: every term it contributes is 0 and no impulse can move it.
		   Most contacts in a game have one such side, so that half is
		   skipped rather than multiplied out to nothing. */
		bool move_a = cs->mA.raw() != 0;
		bool move_b = cs->mB.raw() != 0;

		for (int32_t j = 0; j < cs->contact_count; ++j) {
			ContactState *c = cs->contacts + j;

			Fixed nm = cs->mA + cs->mB;
			Fixed tm[2] = { nm, nm };

			if (move_a) {
				Vector3 raCn = c->ra.cross(cs->normal);
				nm += raCn.dot(cs->iA.transform(raCn));
			}
			if (move_b) {
				Vector3 rbCn = c->rb.cross(cs->normal);
				nm += rbCn.dot(cs->iB.transform(rbCn));
			}
			c->normal_mass = invert_or_zero(nm);

			for (int32_t k = 0; k < 2; ++k) {
				if (move_a) {
					Vector3 raCt = cs->tangent_vectors[k].cross(c->ra);
					tm[k] += raCt.dot(cs->iA.transform(raCt));
				}
				if (move_b) {
					Vector3 rbCt = cs->tangent_vectors[k].cross(c->rb);
					tm[k] += rbCt.dot(cs->iB.transform(rbCt));
				}
				c->tangent_mass[k] = invert_or_zero(tm[k]);
			}

			Fixed pen_bias = c->penetration + PHYSICS_PENETRATION_SLOP;
			if (pen_bias > 0) pen_bias = 0;
			c->bias = -PHYSICS_BAUMGARTE * pen_bias / dt;

			Vector3 P = cs->normal * c->normal_impulse;

			if (enable_friction) {
				P += cs->tangent_vectors[0] * c->tangent_impulse[0];
				P += cs->tangent_vectors[1] * c->tangent_impulse[1];
			}

			if (move_a) {
				vA -= P * cs->mA;
				wA -= cs->iA.transform(c->ra.cross(P));
			}
			if (move_b) {
				vB += P * cs->mB;
				wB += cs->iB.transform(c->rb.cross(P));
			}

			/* rel = (vB + wB × rb) - vA - wA × ra */
			Vector3 rel = (vB + wB.cross(c->rb)) - (vA + wA.cross(c->ra));
			Fixed dv = rel.dot(cs->normal);
			if (dv < -1.0_fp) c->bias += -(cs->restitution) * dv;
		}

		velocities[cs->index_a].v = vA;
		velocities[cs->index_a].w = wA;
		velocities[cs->index_b].v = vB;
		velocities[cs->index_b].w = wB;
	}
}


void ContactSolver::solve()
{
	for (int32_t i = 0; i < contact_count; ++i) {
		ContactConstraintState *cs = contacts + i;

		Vector3 vA = velocities[cs->index_a].v;
		Vector3 wA = velocities[cs->index_a].w;
		Vector3 vB = velocities[cs->index_b].v;
		Vector3 wB = velocities[cs->index_b].w;

		/* Same skip as preSolve: an impulse on a massless side is a no-op. */
		bool move_a = cs->mA.raw() != 0;
		bool move_b = cs->mB.raw() != 0;

		for (int32_t j = 0; j < cs->contact_count; ++j) {
			ContactState *c = cs->contacts + j;

			/* dv = (vB + wB × rb) - vA - wA × ra */
			Vector3 dv = (vB + wB.cross(c->rb)) - (vA + wA.cross(c->ra));

			if (enable_friction) {
				for (int32_t k = 0; k < 2; ++k) {
					Fixed lambda     = -dv.dot(cs->tangent_vectors[k]) * c->tangent_mass[k];
					Fixed max_lambda = cs->friction * c->normal_impulse;
					Fixed old_pt     = c->tangent_impulse[k];
					c->tangent_impulse[k] = clamp(old_pt + lambda, -max_lambda, max_lambda);
					lambda = c->tangent_impulse[k] - old_pt;

					Vector3 impulse = cs->tangent_vectors[k] * lambda;

					if (move_a) {
						vA -= impulse * cs->mA;
						wA -= cs->iA.transform(c->ra.cross(impulse));
					}
					if (move_b) {
						vB += impulse * cs->mB;
						wB += cs->iB.transform(c->rb.cross(impulse));
					}
				}
			}

			/* Recompute dv after friction. */
			dv = (vB + wB.cross(c->rb)) - (vA + wA.cross(c->ra));

			Fixed vn      = dv.dot(cs->normal);
			Fixed lambda  = c->normal_mass * (-vn + c->bias);
			Fixed temp_pn = c->normal_impulse;
			c->normal_impulse = (temp_pn + lambda > 0) ? (temp_pn + lambda) : Fixed(0);
			lambda = c->normal_impulse - temp_pn;

			Vector3 impulse = cs->normal * lambda;

			if (move_a) {
				vA -= impulse * cs->mA;
				wA -= cs->iA.transform(c->ra.cross(impulse));
			}
			if (move_b) {
				vB += impulse * cs->mB;
				wB += cs->iB.transform(c->rb.cross(impulse));
			}
		}

		velocities[cs->index_a].v = vA;
		velocities[cs->index_a].w = wA;
		velocities[cs->index_b].v = vB;
		velocities[cs->index_b].w = wB;
	}
}
