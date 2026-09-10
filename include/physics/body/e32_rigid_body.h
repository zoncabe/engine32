/*
	Ported from qu3e q3Body.h — altered source, not the original software.

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
	Dynamic/static/kinematic body. Owns a linked list of PhysicsShape (Box /
	Sphere / Capsule).
*/
#ifndef ENGINE_32_RIGID_BODY_H
#define ENGINE_32_RIGID_BODY_H

#include <stdint.h>

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_matrix3.h"
#include "physics/math/e32_transform.h"
#include "physics/math/e32_quaternion.h"
#include "physics/shapes/e32_physics_shape.h"
#include "physics/shapes/e32_box.h"
#include "physics/shapes/e32_sphere.h"
#include "physics/shapes/e32_capsule.h"


struct PhysicsWorld;
struct ContactEdge;


enum BodyType {
	BODY_STATIC,
	BODY_DYNAMIC,
	BODY_KINEMATIC,
};


enum {
	BODY_FLAG_AWAKE       = 0x001,
	BODY_FLAG_ACTIVE      = 0x002,
	BODY_FLAG_ALLOW_SLEEP = 0x004,
	BODY_FLAG_ISLAND      = 0x010,
	BODY_FLAG_STATIC      = 0x020,
	BODY_FLAG_DYNAMIC     = 0x040,
	BODY_FLAG_KINEMATIC   = 0x080,
	BODY_FLAG_LOCK_X      = 0x100,
	BODY_FLAG_LOCK_Y      = 0x200,
	BODY_FLAG_LOCK_Z      = 0x400,
};


struct RigidBodyDef
{
	Vector3     axis;
	Angle       angle;
	Vector3     position;
	Vector3     linear_velocity;
	Vector3     angular_velocity;
	Fixed       gravity_scale;
	int32_t     layers;
	void       *owner;

	Fixed       linear_damping;
	Fixed       angular_damping;

	BodyType    body_type;

	int         allow_sleep;
	int         awake;
	int         active;
	int         lock_axis_x;
	int         lock_axis_y;
	int         lock_axis_z;

	void init();
};


struct RigidBody
{
	Matrix3       inv_inertia_model;
	Matrix3       inv_inertia_world;
	Fixed         mass;
	Fixed         inv_mass;
	Vector3       linear_velocity;
	Vector3       angular_velocity;
	Vector3       force;
	Vector3       torque;
	Transform     tx;
	Quaternion    q;
	Vector3       local_center;
	Vector3       world_center;
	Fixed         sleep_time;
	Fixed         gravity_scale;
	int32_t       layers;
	int32_t       flags;

	PhysicsShape *shapes;
	void         *owner;
	PhysicsWorld *world;
	RigidBody    *next;
	RigidBody    *prev;
	int32_t       island_index;

	Fixed         linear_damping;
	Fixed         angular_damping;

	ContactEdge  *contact_list;


	void init(const RigidBodyDef *def, PhysicsWorld *world);

	PhysicsShape *addShape  (const PhysicsShapeDef *def, const Vector3 &scale);
	PhysicsShape *addBox    (const BoxDef     *def);
	PhysicsShape *addSphere (const SphereDef  *def);
	PhysicsShape *addCapsule(const CapsuleDef *def);
	void          removeShape    (const PhysicsShape *shape);
	void          removeAllShapes();

	void applyLinearForce            (const Vector3 &force);
	void applyForceAtWorldPoint      (const Vector3 &force, const Vector3 &point);
	void applyLinearImpulse          (const Vector3 &impulse);
	void applyLinearImpulseAtWorldPoint(const Vector3 &impulse, const Vector3 &point);
	void applyTorque                 (const Vector3 &torque);

	void setToAwake();
	void setToSleep();
	bool isAwake() const { return (flags & BODY_FLAG_AWAKE) != 0; }

	Fixed getGravityScale() const     { return gravity_scale; }
	void  setGravityScale(Fixed scale) { gravity_scale = scale; }

	Vector3 getLocalPoint (const Vector3 &p) const { return tx.mulVectorTransposed(p); }
	Vector3 getLocalVector(const Vector3 &v) const { return tx.rotation.transformTransposed(v); }
	Vector3 getWorldPoint (const Vector3 &p) const { return tx.mulVector(p); }
	Vector3 getWorldVector(const Vector3 &v) const { return tx.rotation.transform(v); }

	Vector3 getLinearVelocity() const { return linear_velocity; }
	Vector3 getVelocityAtWorldPoint(const Vector3 &p) const;
	void    setLinearVelocity(const Vector3 &v);
	Vector3 getAngularVelocity() const { return angular_velocity; }
	void    setAngularVelocity(const Vector3 &v);

	bool canCollide(const RigidBody *other) const;

	Transform  getTransform() const   { return tx; }
	int32_t    getFlags() const       { return flags; }
	void       setLayers(int32_t l)   { layers = l; }
	int32_t    getLayers() const      { return layers; }
	Quaternion getQuaternion() const  { return q; }
	void      *getOwner() const       { return owner; }

	void  setLinearDamping(Fixed d)   { linear_damping = d; }
	Fixed getLinearDamping() const    { return linear_damping; }
	void  setAngularDamping(Fixed d)  { angular_damping = d; }
	Fixed getAngularDamping() const   { return angular_damping; }

	void setTransformPosition         (const Vector3 &position);
	void setTransformPositionAxisAngle(const Vector3 &position, const Vector3 &axis, Angle angle);
	void setTransformPositionYaw      (const Vector3 &position, Angle yaw);   /* about Z */

	Fixed getMass() const    { return mass; }
	Fixed getInvMass() const { return inv_mass; }

	void calculateMassData();
	void synchronizeProxies();
};


#endif
