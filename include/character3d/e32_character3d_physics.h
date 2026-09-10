#ifndef ENGINE_32_CHARACTER3D_PHYSICS_H
#define ENGINE_32_CHARACTER3D_PHYSICS_H

#include "physics/math/e32_math.h"
#include "physics/shapes/e32_physics_shape.h"


struct Character3D;
struct CollisionMesh;
struct PhysicsWorld;
struct RigidBody;


struct KinematicBody
{
	Vector3     position;
	Vector3     velocity;
	Vector3     acceleration;
	EulerAngles rotation;

	/* Its standing in the physics world. The solver never moves it, the
	   fields above do, but registering it is what makes the broadphase pair
	   it with rigid bodies, so the character can shove them. */
	RigidBody *rigid;
};


struct Character3DColliderSettings
{
	Fixed radius;
	Fixed height;
};


struct Character3DCollider
{
	Capsule   shape;
	Transform world;    /* vertical capsule, position at the capsule center */

	void init(Fixed radius, Fixed half_height);
	void setVertical(const Vector3 &position);
};

/* Depenetrate against the world's static bodies — every contact classified as
   floor, wall or ceiling, one combined recovery per pass — then snap to the
   floor. Runs after the movement update, before the render sync. The character
   is not simulated by the solver: it reads those shapes and resolves on its
   own. */
void character3dPhysics_collide(Character3D *character, const PhysicsWorld *world);

/* The character's standing in the world: created once, written every frame
   after collide, so the solver sees where it ended up and how fast it got
   there. Without this the character passes through every rigid body. */
void character3dPhysics_createBody(Character3D *character, PhysicsWorld *world);
void character3dPhysics_syncBody  (Character3D *character);


#endif
