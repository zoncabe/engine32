/*
	Polymorphic shape attached to a RigidBody.

	A RigidBody keeps one linked list of PhysicsShape. Each shape carries its
	local transform and admin fields (friction, restitution, etc.), and a
	tagged union with the concrete geometry (Box / Sphere / Capsule). The
	narrowphase dispatches on `type`.
*/
#ifndef ENGINE_32_PHYSICS_SHAPE_H
#define ENGINE_32_PHYSICS_SHAPE_H

#include <stdint.h>

#include "physics/math/e32_matrix3.h"
#include "physics/math/e32_transform.h"
#include "physics/math/e32_vector3.h"
#include "physics/geometry/e32_aabb.h"
#include "physics/geometry/e32_raycast.h"
#include "physics/shapes/e32_box.h"
#include "physics/shapes/e32_sphere.h"
#include "physics/shapes/e32_capsule.h"
#include "physics/collision/e32_collision_mesh.h"


struct RigidBody;
struct CollisionMesh;


enum ShapeType {
	SHAPE_BOX,
	SHAPE_SPHERE,
	SHAPE_CAPSULE,
	SHAPE_MESH,   /* static-only: no rigid body simulation */
};


/* What a sensor shape is a sensor *for*. Every value but NONE is truthy, so
   the code that only asks whether a shape collides keeps reading `sensor` as
   the flag it always was; the ones that consume a specific kind of volume
   match on the value. */
enum SensorType {
	SENSOR_NONE,
	SENSOR_VOLUME,      /* plain overlap volume: water, triggers */
	SENSOR_CLIMBABLE,   /* ladder: the character probe reads its frame */
};


struct MassData
{
	Matrix3 inertia;
	Vector3 center;
	Fixed   mass;
};


/* Authoring-side counterpart of PhysicsShape: the geometry plus its offset,
   before it is placed in a body or in the world. */
struct PhysicsShapeDef
{
	ShapeType type;
	union {
		BoxDef           box;
		SphereDef        sphere;
		CapsuleDef       capsule;
		CollisionMeshDef mesh;
	};

	/* Shape defs usually initialise .tx with a position only, leaving the
	   rotation matrix zeroed: this reads an all-zero rotation as identity. */
	static Transform localTransform(const Transform &tx);
};


struct PhysicsShape
{
	ShapeType     type;
	Transform     local;
	/* body->tx composed with local, kept current by attach and by
	   rigidBody_synchronizeProxies, which runs whenever the body moves. A
	   static shape used to recompose this in every query of every frame. */
	Transform     world;

	PhysicsShape *next;
	RigidBody    *body;
	Fixed         friction;
	Fixed         restitution;
	Fixed         density;
	int32_t       broadphase_index;
	void         *owner;
	int           sensor;

	union {
		Box     box;
		Sphere  sphere;
		Capsule capsule;
		CollisionMesh *mesh;
	};

	/* Builds the shape from its def, scaling both geometry and offset so one
	   def serves any prop size. The offset lands in local, for the caller to
	   compose with wherever the shape ends up. Returns false for types that
	   are not built from defs, so callers can skip them. */
	bool fromDef(const PhysicsShapeDef *def, const Vector3 &scale);

	/* Frees whatever the shape owns. Only the mesh case owns anything. */
	void release();

	/* Narrowphase / body dispatch: switches on type. The first three read
	   the cached world transform. */
	bool testPoint(const Vector3 &p) const;
	bool raycast(RaycastData *ray) const;
	AABB computeAABB() const;
	void computeMass(MassData *md) const;
};


#endif
