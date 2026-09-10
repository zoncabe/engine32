/*
	Static triangle mesh collision data.

	Loads the .collision the importer writes and builds a DynamicAABBTree
	with one leaf per triangle, in mesh-local space. File layout based on
	pyrite64's mesh collider (Max Bebök, Kevin Reier, MIT), with the
	numbers in the engine's own fixed point:

	    header      tri_count, vert_count, four reserved words (uint32 each)
	    indices     3 x uint16 per triangle
	    normals     3 x int16 per triangle, 20.12 raw (a unit fits in 16 bits)
	    vertices    Vector3 per vertex, 20.12 raw
	    edges       1 byte per triangle, see Triangle

	Every block after the header is aligned to 4 bytes.
*/
#ifndef ENGINE_32_COLLISION_MESH_H
#define ENGINE_32_COLLISION_MESH_H

#include <stdint.h>

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_transform.h"
#include "physics/geometry/e32_aabb.h"
#include "physics/geometry/e32_triangle.h"
#include "physics/broadphase/e32_dynamic_aabb_tree.h"


/* Authoring side, matching the other shape defs. There is no density: a mesh
   only ever hangs off a static body, so it has no mass to compute. */
struct CollisionMeshDef
{
	Transform   tx;
	const char *path;
	Fixed       friction;
	Fixed       restitution;
};


struct CollisionMesh
{
	uint16_t        triangle_count;
	uint16_t        vertex_count;

	const uint16_t *indices;         /* 3 per triangle */
	const int16_t  *packed_normals;  /* 3 per triangle, 20.12 raw */
	const Vector3  *vertices;
	const uint8_t  *active_edges;    /* 1 per triangle, see Triangle */

	DynamicAABBTree tree;            /* leaf per triangle, user_data = triangle index */

	/* The file's bytes stay where the resource module keeps them; only the
	   mesh and its tree are allocated, and free() gives those back. */
	static CollisionMesh *load(const char *path);
	void free();

	Triangle triangle(int32_t index) const;

	void queryAABB(void *cb, PhysicsQueryCallback callback, const AABB &aabb) const;

	bool raycast(const Transform &world, RaycastData *ray) const;
};


#endif
