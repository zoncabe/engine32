/*
	Ported from qu3e q3Body.cpp — altered source, not the original software.

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
	Owns a linked list of PhysicsShape (Box / Sphere / Capsule via tagged
	union).
*/
#include <stddef.h>

#include "psyqo/kernel.hh"

#include "physics/body/e32_rigid_body.h"
#include "physics/shapes/e32_physics_shape.h"
#include "physics/world/e32_physics_world.h"
#include "physics/math/e32_trig.h"
#include "physics/math/e32_math_common.h"


void RigidBodyDef::init()
{
	axis             = Vector3::zero();
	angle            = 0.0_pi;
	position         = Vector3::zero();
	linear_velocity  = Vector3::zero();
	angular_velocity = Vector3::zero();

	gravity_scale   = 1.0_fp;
	body_type       = BODY_STATIC;
	layers          = 0x00000001;
	owner           = NULL;
	allow_sleep     = 1;
	awake           = 1;
	active          = 1;
	lock_axis_x     = 0;
	lock_axis_y     = 0;
	lock_axis_z     = 0;
	linear_damping  = 0.0_fp;
	angular_damping = 0.1_fp;
}


void RigidBody::init(const RigidBodyDef *def, PhysicsWorld *w)
{
	linear_velocity  = def->linear_velocity;
	angular_velocity = def->angular_velocity;
	force            = Vector3::zero();
	torque           = Vector3::zero();

	q           = Quaternion::fromAxisAngle(def->axis.normalized(), def->angle);
	tx.rotation = q.toMatrix3();
	tx.position = def->position;

	sleep_time      = 0;
	gravity_scale   = def->gravity_scale;
	layers          = def->layers;
	owner           = def->owner;
	world           = w;
	flags           = 0;
	linear_damping  = def->linear_damping;
	angular_damping = def->angular_damping;

	if (def->body_type == BODY_DYNAMIC) {
		flags |= BODY_FLAG_DYNAMIC;
	}
	else if (def->body_type == BODY_STATIC) {
		flags |= BODY_FLAG_STATIC;
		linear_velocity  = Vector3::zero();
		angular_velocity = Vector3::zero();
		force            = Vector3::zero();
		torque           = Vector3::zero();
	}
	else if (def->body_type == BODY_KINEMATIC) {
		flags |= BODY_FLAG_KINEMATIC;
	}

	if (def->allow_sleep) flags |= BODY_FLAG_ALLOW_SLEEP;
	if (def->awake)       flags |= BODY_FLAG_AWAKE;
	if (def->active)      flags |= BODY_FLAG_ACTIVE;
	if (def->lock_axis_x) flags |= BODY_FLAG_LOCK_X;
	if (def->lock_axis_y) flags |= BODY_FLAG_LOCK_Y;
	if (def->lock_axis_z) flags |= BODY_FLAG_LOCK_Z;

	shapes       = NULL;
	contact_list = NULL;
	next         = NULL;
	prev         = NULL;
	island_index = 0;

	inv_inertia_model = Matrix3::zero();
	inv_inertia_world = Matrix3::zero();
	mass              = 0;
	inv_mass          = 0;
	local_center      = Vector3::zero();
	world_center      = def->position;
}


/* Fill the admin fields of a freshly-allocated PhysicsShape and link it into
   the body's list. Type-specific geometry is copied by the caller. */
static PhysicsShape *rigidBody_attachShape(RigidBody *b, PhysicsShape *shape,
                                            const Transform &local, Fixed friction,
                                            Fixed restitution, Fixed density, int sensor)
{
	shape->local = local;
	/* Defs declared as zero-initialised globals have a zero rotation matrix;
	   treat that as identity so the mass inversion doesn't blow up. */
	Fixed rot_sum = shape->local.rotation.ex.squaredMagnitude()
	              + shape->local.rotation.ey.squaredMagnitude()
	              + shape->local.rotation.ez.squaredMagnitude();
	if (rot_sum < TOLERANCE) {
		shape->local.rotation = Matrix3::identity();
	}

	shape->friction    = friction;
	shape->restitution = restitution;
	shape->density     = density;
	shape->sensor      = sensor;
	shape->body        = b;
	shape->owner       = NULL;
	shape->broadphase_index = -1;
	shape->next        = b->shapes;
	b->shapes          = shape;
	shape->world       = b->tx * shape->local;

	AABB aabb = shape->computeAABB();

	b->calculateMassData();

	b->world->contact_manager.broadphase.insertShape(shape, aabb);
	b->world->markNewShape();

	return shape;
}


/* Type-agnostic entry point: the def carries its own geometry and offset, and
   the scale is applied on the way in. */
PhysicsShape *RigidBody::addShape(const PhysicsShapeDef *def, const Vector3 &scale)
{
	PhysicsShape *shape = world->allocShape();

	if (!shape->fromDef(def, scale)) return NULL;

	Transform local = shape->local;

	return rigidBody_attachShape(this, shape, local,
	                              shape->friction, shape->restitution,
	                              shape->density, shape->sensor);
}


PhysicsShape *RigidBody::addBox(const BoxDef *def)
{
	PhysicsShape *shape = world->allocShape();

	shape->type  = SHAPE_BOX;
	shape->box.e = def->e;

	return rigidBody_attachShape(this, shape, def->tx,
	                              def->friction, def->restitution,
	                              def->density, def->sensor);
}


PhysicsShape *RigidBody::addSphere(const SphereDef *def)
{
	PhysicsShape *shape = world->allocShape();

	shape->type          = SHAPE_SPHERE;
	shape->sphere.radius = def->radius;

	return rigidBody_attachShape(this, shape, def->tx,
	                              def->friction, def->restitution,
	                              def->density, def->sensor);
}


PhysicsShape *RigidBody::addCapsule(const CapsuleDef *def)
{
	PhysicsShape *shape = world->allocShape();

	shape->type                = SHAPE_CAPSULE;
	shape->capsule.radius      = def->radius;
	shape->capsule.half_height = def->half_height;

	return rigidBody_attachShape(this, shape, def->tx,
	                              def->friction, def->restitution,
	                              def->density, def->sensor);
}


void RigidBody::removeShape(const PhysicsShape *shape)
{
	psyqo::Kernel::assert(shape != NULL && shape->body == this, "physics: shape not on this body");

	PhysicsShape *node  = shapes;
	bool          found = false;

	if (node == shape) {
		shapes = node->next;
		found = true;
	} else {
		while (node) {
			if (node->next == shape) {
				node->next = shape->next;
				found = true;
				break;
			}
			node = node->next;
		}
	}
	psyqo::Kernel::assert(found, "physics: shape not on this body");

	/* The contacts of this one shape stay until the manager drops them:
	   removeAllShapes below is the body-wide purge. */

	world->contact_manager.broadphase.removeShape(shape);
	calculateMassData();
	world->freeShape((PhysicsShape *)shape);
}


void RigidBody::removeAllShapes()
{
	while (shapes) {
		PhysicsShape *next_shape = shapes->next;
		world->contact_manager.broadphase.removeShape(shapes);
		shapes->release();
		world->freeShape(shapes);
		shapes = next_shape;
	}
	world->contact_manager.removeContactsFromBody(this);
}


void RigidBody::applyLinearForce(const Vector3 &f)
{
	force += f * mass;
	setToAwake();
}


void RigidBody::applyForceAtWorldPoint(const Vector3 &f, const Vector3 &point)
{
	force  += f * mass;
	torque += (point - world_center).cross(f);
	setToAwake();
}


void RigidBody::applyLinearImpulse(const Vector3 &impulse)
{
	linear_velocity += impulse * inv_mass;
	setToAwake();
}


void RigidBody::applyLinearImpulseAtWorldPoint(const Vector3 &impulse, const Vector3 &point)
{
	linear_velocity += impulse * inv_mass;

	Vector3 arm = point - world_center;
	angular_velocity += inv_inertia_world.transform(arm.cross(impulse));
	setToAwake();
}


void RigidBody::applyTorque(const Vector3 &t)
{
	torque += t;
}


void RigidBody::setToAwake()
{
	if (!(flags & BODY_FLAG_AWAKE)) {
		flags |= BODY_FLAG_AWAKE;
		sleep_time = 0;
	}
}


void RigidBody::setToSleep()
{
	flags &= ~BODY_FLAG_AWAKE;
	sleep_time       = 0;
	linear_velocity  = Vector3::zero();
	angular_velocity = Vector3::zero();
	force            = Vector3::zero();
	torque           = Vector3::zero();
}


Vector3 RigidBody::getVelocityAtWorldPoint(const Vector3 &p) const
{
	return linear_velocity + angular_velocity.cross(p - world_center);
}


void RigidBody::setLinearVelocity(const Vector3 &v)
{
	psyqo::Kernel::assert(!(flags & BODY_FLAG_STATIC), "physics: velocity on a static body");
	if (v.dot(v) > 0) setToAwake();
	linear_velocity = v;
}


void RigidBody::setAngularVelocity(const Vector3 &v)
{
	psyqo::Kernel::assert(!(flags & BODY_FLAG_STATIC), "physics: velocity on a static body");
	if (v.dot(v) > 0) setToAwake();
	angular_velocity = v;
}


bool RigidBody::canCollide(const RigidBody *other) const
{
	if (this == other) return false;
	if (!(flags & BODY_FLAG_DYNAMIC) && !(other->flags & BODY_FLAG_DYNAMIC)) return false;
	if (!(layers & other->layers)) return false;
	return true;
}


void RigidBody::setTransformPosition(const Vector3 &position)
{
	world_center = position;
	synchronizeProxies();
}


void RigidBody::setTransformPositionAxisAngle(const Vector3 &position, const Vector3 &axis, Angle angle)
{
	world_center = position;
	q            = Quaternion::fromAxisAngle(axis, angle);
	tx.rotation  = q.toMatrix3();
	synchronizeProxies();
}


/* Yaw only, for a body that never tilts: the quaternion and the matrix come
   straight from the half angle's sine and cosine, the same values the
   axis-angle path would reach through the general quaternion-to-matrix
   expansion with every other term zero. */
void RigidBody::setTransformPositionYaw(const Vector3 &position, Angle yaw)
{
	Fixed s, c;
	Trig::sincos(yaw / 2, &s, &c);

	Fixed ss2 = s * s * 2;   /* 1 - cos(yaw) */
	Fixed sc2 = s * c * 2;   /* sin(yaw) */

	world_center = position;
	q            = { 0.0_fp, 0.0_fp, s, c };
	tx.rotation  = {
		{ 1.0_fp - ss2,  sc2,          0.0_fp },
		{ -sc2,          1.0_fp - ss2, 0.0_fp },
		{ 0.0_fp,        0.0_fp,       1.0_fp },
	};
	synchronizeProxies();
}


void RigidBody::calculateMassData()
{
	Matrix3 inertia   = Matrix3::zero();
	inv_inertia_model = Matrix3::zero();
	inv_inertia_world = Matrix3::zero();
	inv_mass          = 0;
	mass              = 0;
	Fixed total       = 0;

	if (flags & BODY_FLAG_STATIC || flags & BODY_FLAG_KINEMATIC) {
		local_center = Vector3::zero();
		world_center = tx.position;
		return;
	}

	Vector3 lc = Vector3::zero();

	for (PhysicsShape *shape = shapes; shape; shape = shape->next) {
		if (shape->density == 0) continue;

		MassData md;
		shape->computeMass(&md);
		total   += md.mass;
		inertia += md.inertia;
		lc      += md.center * md.mass;
	}

	if (total > 0) {
		mass     = total;
		inv_mass = 1.0_fp / total;
		lc       = lc * inv_mass;

		inertia -= (Matrix3::identity() * lc.dot(lc) - Matrix3::outerProduct(lc, lc)) * total;
		inv_inertia_model = inertia.inverse();

		if (flags & BODY_FLAG_LOCK_X) {
			/* Zero the row that governs X rotation. */
			inv_inertia_model.ex = Vector3::zero();
		}
		if (flags & BODY_FLAG_LOCK_Y) {
			inv_inertia_model.ey = Vector3::zero();
		}
		if (flags & BODY_FLAG_LOCK_Z) {
			inv_inertia_model.ez = Vector3::zero();
		}
	}
	else {
		inv_mass          = 1.0_fp;
		inv_inertia_model = Matrix3::zero();
		inv_inertia_world = Matrix3::zero();
	}

	local_center = lc;
	world_center = tx.mulVector(lc);
}


void RigidBody::synchronizeProxies()
{
	tx.position = world_center - tx.rotation.transform(local_center);

	PhysicsShape *shape = shapes;
	while (shape) {
		shape->world = tx * shape->local;
		AABB aabb = shape->computeAABB();
		world->contact_manager.broadphase.update(shape->broadphase_index, aabb);
		shape = shape->next;
	}
}
