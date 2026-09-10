#include <stdint.h>
#include <stddef.h>

#include "psyqo/alloc.h"
#include "psyqo/kernel.hh"

#include "physics/collision/e32_collision_mesh.h"
#include "resource/e32_resource.h"


struct RawCollisionHeader
{
	uint32_t tri_count;
	uint32_t vert_count;
	uint32_t reserved[4];   /* written as 0 by the importer */
};


static const uint8_t *alignPtr(const uint8_t *ptr, size_t alignment)
{
	return (const uint8_t *)(((uintptr_t)ptr + alignment - 1) & ~(alignment - 1));
}


CollisionMesh *CollisionMesh::load(const char *path)
{
	size_t size = 0;
	const uint8_t *buffer = Resource::find(path, &size);
	psyqo::Kernel::assert(buffer != NULL && size > sizeof(RawCollisionHeader), "collision: file not found");

	const RawCollisionHeader *header = (const RawCollisionHeader *)buffer;
	psyqo::Kernel::assert(header->tri_count > 0 && header->tri_count <= 0xFFFFu &&
	                      header->vert_count > 0 && header->vert_count <= 0xFFFFu, "collision: bad header");

	const uint8_t *data = (const uint8_t *)(header + 1);

	const uint16_t *indices = (const uint16_t *)data;
	data += header->tri_count * 3 * sizeof(uint16_t);
	data = alignPtr(data, 4);

	const int16_t *packed_normals = (const int16_t *)data;
	data += header->tri_count * 3 * sizeof(int16_t);
	data = alignPtr(data, 4);

	const Vector3 *vertices = (const Vector3 *)data;
	data += header->vert_count * sizeof(Vector3);
	data = alignPtr(data, 4);

	const uint8_t *active_edges = data;

	/* Trips on assets built before the active-edge block: re-import them. */
	psyqo::Kernel::assert(active_edges + header->tri_count <= buffer + size, "collision: file truncated");

	CollisionMesh *mesh = (CollisionMesh *)psyqo_malloc(sizeof(CollisionMesh));
	psyqo::Kernel::assert(mesh != NULL, "collision: out of memory");
	mesh->triangle_count = (uint16_t)header->tri_count;
	mesh->vertex_count   = (uint16_t)header->vert_count;
	mesh->indices        = indices;
	mesh->packed_normals = packed_normals;
	mesh->vertices       = vertices;
	mesh->active_edges   = active_edges;

	mesh->tree.init();

	for (int32_t t = 0; t < mesh->triangle_count; t++) {
		Triangle triangle = mesh->triangle(t);
		const Vector3 *v = triangle.vertices;

		AABB aabb;
		aabb.min = v[0];
		aabb.max = v[0];
		for (int i = 1; i < 3; i++) {
			if (v[i].x < aabb.min.x) aabb.min.x = v[i].x;
			if (v[i].y < aabb.min.y) aabb.min.y = v[i].y;
			if (v[i].z < aabb.min.z) aabb.min.z = v[i].z;
			if (v[i].x > aabb.max.x) aabb.max.x = v[i].x;
			if (v[i].y > aabb.max.y) aabb.max.y = v[i].y;
			if (v[i].z > aabb.max.z) aabb.max.z = v[i].z;
		}

		mesh->tree.insert(aabb, (void *)(intptr_t)t);
	}

	return mesh;
}

void CollisionMesh::free()
{
	tree.shutdown();
	psyqo_free(this);
}

Triangle CollisionMesh::triangle(int32_t index) const
{
	const uint16_t *idx = &indices[index * 3];
	const int16_t  *n   = &packed_normals[index * 3];

	Triangle out;
	out.vertices[0]  = vertices[idx[0]];
	out.vertices[1]  = vertices[idx[1]];
	out.vertices[2]  = vertices[idx[2]];
	out.normal       = { Fixed(n[0], Fixed::RAW), Fixed(n[1], Fixed::RAW), Fixed(n[2], Fixed::RAW) };
	out.active_edges = active_edges[index];
	return out;
}

void CollisionMesh::queryAABB(void *cb, PhysicsQueryCallback callback, const AABB &aabb) const
{
	tree.queryAABB(cb, callback, aabb);
}


struct MeshRaycast
{
	const CollisionMesh *mesh;
	RaycastData         *ray;
	bool                 hit;
};

/* Every leaf the ray crosses is a triangle. Shortening the ray on each hit
   leaves the closest one and lets the tree prune what is behind it. */
static int collisionMesh_raycastLeaf(void *cb, int32_t id)
{
	MeshRaycast *query = (MeshRaycast *)cb;

	Triangle triangle = query->mesh->triangle((int32_t)(intptr_t)query->mesh->tree.userData(id));

	if (triangle.raycast(query->ray)) {
		query->ray->t = query->ray->toi;
		query->hit    = true;
	}
	return 1;
}

/* The tree lives in mesh-local space: the ray goes in through the inverse of
   the mesh's world transform and the hit normal is rotated back out. The
   time of impact is a distance along the ray, so it survives as is. */
bool CollisionMesh::raycast(const Transform &world, RaycastData *ray) const
{
	RaycastData local = *ray;
	local.start = world.mulVectorTransposed(ray->start);
	local.dir   = world.rotation.transformTransposed(ray->dir);

	MeshRaycast query = { this, &local, false };
	tree.queryRay(&query, collisionMesh_raycastLeaf, &local);

	if (!query.hit) return false;

	ray->toi    = local.toi;
	ray->normal = world.rotation.transform(local.normal);
	return true;
}
