/*
	Narrowphase/body dispatchers. Each function builds the world transform
	(body_tx · shape->local) and routes to the concrete shape implementation
	based on shape->type.
*/
#include <stddef.h>

#include "physics/shapes/e32_physics_shape.h"
#include "physics/collision/e32_collision_mesh.h"


Transform PhysicsShapeDef::localTransform(const Transform &tx)
{
	Transform local = tx;

	if (local.rotation.ex.squaredMagnitude() == 0 &&
	    local.rotation.ey.squaredMagnitude() == 0 &&
	    local.rotation.ez.squaredMagnitude() == 0)
		local.rotation = Matrix3::identity();

	return local;
}


bool PhysicsShape::fromDef(const PhysicsShapeDef *def, const Vector3 &scale)
{
	const Transform *tx = NULL;

	*this = {};
	type  = def->type;

	switch (def->type) {
		case SHAPE_BOX:
			box.e = {
				def->box.e.x * scale.x,
				def->box.e.y * scale.y,
				def->box.e.z * scale.z,
			};
			friction    = def->box.friction;
			restitution = def->box.restitution;
			density     = def->box.density;
			sensor      = def->box.sensor;
			tx = &def->box.tx;
			break;

		/* A sphere cannot be squashed, so a non-uniform scale takes X. */
		case SHAPE_SPHERE:
			sphere.radius = def->sphere.radius * scale.x;
			friction    = def->sphere.friction;
			restitution = def->sphere.restitution;
			density     = def->sphere.density;
			sensor      = def->sphere.sensor;
			tx = &def->sphere.tx;
			break;

		/* The capsule runs along local Z: radius takes X, height takes Z. */
		case SHAPE_CAPSULE:
			capsule.radius      = def->capsule.radius * scale.x;
			capsule.half_height = def->capsule.half_height * scale.z;
			friction    = def->capsule.friction;
			restitution = def->capsule.restitution;
			density     = def->capsule.density;
			sensor      = def->capsule.sensor;
			tx = &def->capsule.tx;
			break;

		/* The mesh comes from an asset, so the def names a file instead of
		   describing a size. Scale is ignored: the triangles are already at
		   the size they were authored. */
		case SHAPE_MESH:
			mesh = CollisionMesh::load(def->mesh.path);
			if (mesh == NULL) return false;

			friction    = def->mesh.friction;
			restitution = def->mesh.restitution;
			density     = 0;
			tx = &def->mesh.tx;
			break;
	}

	local = PhysicsShapeDef::localTransform(*tx);
	local.position = {
		local.position.x * scale.x,
		local.position.y * scale.y,
		local.position.z * scale.z,
	};

	return true;
}


/* Counterpart of fromDef: only the mesh case allocates, and the asset it
   loaded dies with the shape that asked for it. */
void PhysicsShape::release()
{
	if (type != SHAPE_MESH || mesh == NULL) return;

	mesh->free();
	mesh = NULL;
}


bool PhysicsShape::testPoint(const Vector3 &p) const
{
	switch (type) {
		case SHAPE_BOX:     return box.testPoint    (world, p);
		case SHAPE_SPHERE:  return sphere.testPoint (world, p);
		case SHAPE_CAPSULE: return capsule.testPoint(world, p);
		case SHAPE_MESH:    break;   /* static-only, never on a rigid body */
	}
	return false;
}


bool PhysicsShape::raycast(RaycastData *ray) const
{
	switch (type) {
		case SHAPE_BOX:     return box.raycast    (world, ray);
		case SHAPE_SPHERE:  return sphere.raycast (world, ray);
		case SHAPE_CAPSULE: return capsule.raycast(world, ray);
		case SHAPE_MESH:    return mesh->raycast(world, ray);
	}
	return false;
}


AABB PhysicsShape::computeAABB() const
{
	switch (type) {
		case SHAPE_BOX:     return box.computeAABB    (world);
		case SHAPE_SPHERE:  return sphere.computeAABB (world);
		case SHAPE_CAPSULE: return capsule.computeAABB(world);

		/* The tree's root already bounds every triangle, in mesh-local space.
		   Taken as a box centred on that bound and carried by the mesh's
		   world transform, it is bounded again in the world, rotation
		   included. */
		case SHAPE_MESH: {
			if (mesh == NULL || mesh->tree.root < 0) break;

			AABB    root   = mesh->tree.fatAABB(mesh->tree.root);
			Vector3 center = (root.min + root.max) * 0.5_fp;

			Box bound;
			bound.e = root.max - center;

			Transform bound_tx = world;
			bound_tx.position  = world.mulVector(center);

			return bound.computeAABB(bound_tx);
		}
	}
	return { Vector3::zero(), Vector3::zero() };
}


void PhysicsShape::computeMass(MassData *md) const
{
	switch (type) {
		case SHAPE_BOX:     box.computeMass    (local, density, md); break;
		case SHAPE_SPHERE:  sphere.computeMass (local, density, md); break;
		case SHAPE_CAPSULE: capsule.computeMass(local, density, md); break;
		case SHAPE_MESH:    break;   /* static-only, never on a rigid body */
	}
}
