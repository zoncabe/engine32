/*
	OBJ to the engine's collision file, for the PlayStation.

	Reads the same OBJ the model importer reads, keeps only its positions
	and faces, welds the vertices by position and writes the triangle list
	with one unit normal and one active-edge byte per triangle, laid out
	the way CollisionMesh::load reads it (see e32_collision_mesh.h):

	    header      tri_count, vert_count, four reserved words (uint32 each)
	    indices     3 x uint16 per triangle
	    normals     3 x int16 per triangle, 20.12 raw
	    vertices    3 x int32 per vertex, 20.12 raw, meters
	    edges       1 byte per triangle

	Every block is aligned to 4 bytes; everything is little endian.

	The mesh deform matches these vertices against the model's by rounded
	position, so the two importers have to quantize the same way: the
	positions are rounded to 20.12 here and the model rounds to its 10.6,
	and both come from the same floats.

	Layout based on pyrite64's mesh collider (Max Bebök, Kevin Reier, MIT).
	Active-edge baking ported from JoltPhysics by Jorrit Rouwe, MIT
	licensed, https://github.com/jrouwe/JoltPhysics.
*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <array>
#include <map>
#include <string>
#include <vector>


struct Vec3 { float x, y, z; };

struct Mesh {
	std::vector<std::array<int32_t, 3>> vertices;   /* 20.12 */
	std::vector<Vec3>                   vertices_float;
	std::vector<uint16_t>               indices;    /* 3 per triangle */
	std::vector<std::array<int16_t, 3>> normals;    /* 20.12, 1 per triangle */
	std::vector<uint8_t>                active_edges;
};


static void fail(const char *message)
{
	fprintf(stderr, "Error: %s\n", message);
	exit(1);
}


/* --- the OBJ --------------------------------------------------------------- */

static std::string trim(const std::string &s)
{
	size_t a = s.find_first_not_of(" \t\r\n");
	size_t b = s.find_last_not_of(" \t\r\n");
	return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}

/* Blender writes Y up and -Z forward; the engine is Z up, same turn the
   model importer makes. */
static void toZUp(float v[3])
{
	float y = v[1], z = v[2];
	v[1] = -z;
	v[2] = y;
}

static int32_t toFixed(float v)
{
	return (int32_t)lrintf(v * 4096.0f);
}

/* Returns the index of the welded vertex, appending it if it is new. Two
   corners weld when they round to the same 20.12 position, which is the
   only position the console ever sees. */
static uint16_t weldVertex(Mesh *mesh, std::map<std::array<int32_t, 3>, uint16_t> *welded, const float p[3])
{
	std::array<int32_t, 3> key = { toFixed(p[0]), toFixed(p[1]), toFixed(p[2]) };

	auto it = welded->find(key);
	if (it != welded->end()) return it->second;

	if (mesh->vertices.size() >= 0x10000) fail("Too many vertices (>65535)!");

	uint16_t index = (uint16_t)mesh->vertices.size();
	mesh->vertices.push_back(key);
	mesh->vertices_float.push_back({ key[0] / 4096.0f, key[1] / 4096.0f, key[2] / 4096.0f });
	(*welded)[key] = index;
	return index;
}

/* The first number of a corner: v, v/t, v//n or v/t/n, one based,
   negative from the end. */
static bool parseCorner(const char *s, int nv, int *out)
{
	int v = 0;
	if (sscanf(s, "%d", &v) != 1) return false;
	*out = v < 0 ? nv + v : v - 1;
	return *out >= 0 && *out < nv;
}

static void readObj(const char *path, Mesh *mesh)
{
	FILE *f = fopen(path, "r");
	if (!f) fail("Cannot open the OBJ!");

	std::vector<std::array<float, 3>> positions;
	std::map<std::array<int32_t, 3>, uint16_t> welded;
	char line[4096];

	while (fgets(line, sizeof(line), f)) {
		std::string s = trim(line);
		if (s.empty() || s[0] == '#') continue;

		if (s.compare(0, 2, "v ") == 0) {
			float p[3];
			if (sscanf(s.c_str() + 2, "%f %f %f", &p[0], &p[1], &p[2]) < 3) continue;
			toZUp(p);
			positions.push_back({p[0], p[1], p[2]});
		} else if (s.compare(0, 2, "f ") == 0) {
			int corner[16];
			int count = 0;
			char *save = nullptr;
			char *tok = strtok_r(line + 2, " \t\r\n", &save);
			while (tok && count < 16) {
				if (!parseCorner(tok, (int)positions.size(), &corner[count])) break;
				count++;
				tok = strtok_r(nullptr, " \t\r\n", &save);
			}
			if (count < 3) continue;

			/* A fan from the first corner: quads and larger faces become
			   triangles, which is all the collision mesh holds. */
			uint16_t first = weldVertex(mesh, &welded, positions[corner[0]].data());
			for (int k = 1; k + 1 < count; k++) {
				mesh->indices.push_back(first);
				mesh->indices.push_back(weldVertex(mesh, &welded, positions[corner[k]].data()));
				mesh->indices.push_back(weldVertex(mesh, &welded, positions[corner[k + 1]].data()));
			}
		}
	}

	fclose(f);
}


/* --- normals --------------------------------------------------------------- */

static Vec3 vecSub(Vec3 a, Vec3 b)
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

static Vec3 vecCross(Vec3 a, Vec3 b)
{
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x,
	};
}

static float vecDot(Vec3 a, Vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/* Per-triangle unit normals, compacting away the triangles without area:
   welding can expose degenerate ones that were hidden behind split
   vertices, and they contribute nothing to collision. The float normals
   are kept for the active-edge fold test. */
static std::vector<Vec3> buildNormals(Mesh *mesh)
{
	std::vector<Vec3> face_normals;
	size_t write = 0;

	for (size_t v = 0; v < mesh->indices.size(); v += 3) {
		uint16_t ia = mesh->indices[v], ib = mesh->indices[v + 1], ic = mesh->indices[v + 2];

		if (ia == ib || ib == ic || ia == ic) continue;

		Vec3 a = mesh->vertices_float[ia];
		Vec3 b = mesh->vertices_float[ib];
		Vec3 c = mesh->vertices_float[ic];

		Vec3 normal = vecCross(vecSub(b, a), vecSub(c, a));
		float len = sqrtf(vecDot(normal, normal));

		if (len < 0.0001f) continue;   /* no area: contributes nothing to collision */

		mesh->indices[write * 3 + 0] = ia;
		mesh->indices[write * 3 + 1] = ib;
		mesh->indices[write * 3 + 2] = ic;

		Vec3 unit = { normal.x / len, normal.y / len, normal.z / len };
		face_normals.push_back(unit);

		/* 20.12 in an int16: a unit is 4096, which fits. */
		mesh->normals.push_back({
			(int16_t)lrintf(unit.x * 4096.0f),
			(int16_t)lrintf(unit.y * 4096.0f),
			(int16_t)lrintf(unit.z * 4096.0f),
		});
		write++;
	}

	mesh->indices.resize(write * 3);
	return face_normals;
}


/* --- active edges ---------------------------------------------------------- */

/*
	Ported from Jolt's MeshShape::sFindActiveEdges and
	ActiveEdges::IsEdgeActive (JoltPhysics, MIT licensed). An edge is active
	when a contact normal found on it is real: a border with no neighbour, a
	convex fold sharper than the threshold, or a non-manifold edge. Inactive
	edges are the internal seams of flat floors and continuous ramps; at
	runtime a contact normal landing on one is replaced by the face normal.
*/

/* Folds between two walkable faces must never produce a blocking normal, so
   the threshold is the character's walkable limit, not Jolt's 5-degree
   default, which is tuned for rigid-body fidelity. */
#define ACTIVE_EDGE_COS_THRESHOLD 0.6428f   /* cos(50 degrees) */

/* Port of ActiveEdges::IsEdgeActive: normal1 belongs to the triangle whose
   winding gives edge_direction, normal2 to its neighbour. */
static bool isEdgeActive(Vec3 normal1, Vec3 normal2, Vec3 edge_direction)
{
	/* If normals are opposite the edges are active (the triangles are back to back). */
	float cos_angle_normals = vecDot(normal1, normal2);
	if (cos_angle_normals < -0.999848f) return true;   /* cos(179 degrees) */

	/* Concave edge: not active. */
	if (vecDot(vecCross(normal1, normal2), edge_direction) < 0.0f) return false;

	/* Convex edge: active when the angle is bigger than the threshold. */
	return cos_angle_normals < ACTIVE_EDGE_COS_THRESHOLD;
}

struct EdgeSlot {
	uint8_t  count;     /* triangles seen on this edge, saturating at 3 */
	uint32_t tri[2];
	uint8_t  edge[2];   /* which edge of that triangle: 0 = v0v1, 1 = v1v2, 2 = v2v0 */
};

/* Port of MeshShape::sFindActiveEdges over the compacted triangle list. */
static void findActiveEdges(Mesh *mesh, const std::vector<Vec3> &face_normals)
{
	size_t tri_count = mesh->indices.size() / 3;
	mesh->active_edges.assign(tri_count, 0);

	std::map<uint32_t, EdgeSlot> edges;

	/* Pass 1: map every edge to the triangles that share it. From the third
	   triangle on an edge is non-manifold: active for that triangle on the
	   spot, and for the first two in pass 2. */
	for (size_t t = 0; t < tri_count; t++) {
		for (int e = 0; e < 3; e++) {
			uint16_t a = mesh->indices[t * 3 + e];
			uint16_t b = mesh->indices[t * 3 + (e + 1) % 3];
			uint32_t key = a < b ? ((uint32_t)a << 16) | b : ((uint32_t)b << 16) | a;

			EdgeSlot &slot = edges[key];
			if (slot.count < 2) {
				slot.tri[slot.count]  = (uint32_t)t;
				slot.edge[slot.count] = (uint8_t)e;
				slot.count++;
			} else {
				mesh->active_edges[t] |= 1 << e;
				slot.count = 3;
			}
		}
	}

	/* Pass 2: border edges are active; edges shared by two triangles follow
	   the fold test, with the edge direction as wound by the first one. */
	for (const auto &entry : edges) {
		const EdgeSlot &slot = entry.second;

		int num_active;
		if (slot.count == 1) {
			num_active = 1;
		} else if (slot.count == 2) {
			uint32_t t1 = slot.tri[0];
			Vec3 e1 = mesh->vertices_float[mesh->indices[t1 * 3 + slot.edge[0]]];
			Vec3 e2 = mesh->vertices_float[mesh->indices[t1 * 3 + (slot.edge[0] + 1) % 3]];
			num_active = isEdgeActive(face_normals[t1], face_normals[slot.tri[1]], vecSub(e2, e1)) ? 2 : 0;
		} else {
			num_active = 2;
		}

		for (int n = 0; n < num_active; n++)
			mesh->active_edges[slot.tri[n]] |= 1 << slot.edge[n];
	}
}


/* --- the file -------------------------------------------------------------- */

struct Writer {
	FILE *f;
	size_t written = 0;

	void put(const void *data, size_t size) { fwrite(data, 1, size, f); written += size; }
	void u16(uint16_t v) { put(&v, 2); }
	void i16(int16_t v)  { put(&v, 2); }
	void u32(uint32_t v) { put(&v, 4); }
	void i32(int32_t v)  { put(&v, 4); }
	void u8(uint8_t v)   { put(&v, 1); }
	void align() { while (written & 3) u8(0); }
};

static void writeFile(const Mesh &mesh, const char *path)
{
	Writer w;
	w.f = fopen(path, "wb");
	if (!w.f) fail("Cannot open output file!");

	size_t tri_count = mesh.indices.size() / 3;
	if (tri_count == 0) fail("No triangles!");

	w.u32((uint32_t)tri_count);
	w.u32((uint32_t)mesh.vertices.size());
	for (int i = 0; i < 4; i++) w.u32(0);   /* reserved */

	for (uint16_t index : mesh.indices) w.u16(index);
	w.align();

	for (const auto &n : mesh.normals) { w.i16(n[0]); w.i16(n[1]); w.i16(n[2]); }
	w.align();

	for (const auto &v : mesh.vertices) { w.i32(v[0]); w.i32(v[1]); w.i32(v[2]); }
	w.align();

	for (uint8_t e : mesh.active_edges) w.u8(e);
	w.align();

	fclose(w.f);
}

int main(int argc, char **argv)
{
	if (argc != 3) {
		fprintf(stderr, "usage: %s input.obj output.collision\n", argv[0]);
		return 1;
	}

	/* A machine writes the file in its own byte order; the console is
	   little endian, and so is every host this runs on. */
	uint16_t probe = 1;
	if (*(uint8_t *)&probe != 1) fail("Big endian host: the file would come out backwards!");

	Mesh mesh;
	readObj(argv[1], &mesh);
	if (mesh.indices.size() % 3 != 0) fail("Index count not divisible by 3!");

	std::vector<Vec3> face_normals = buildNormals(&mesh);
	findActiveEdges(&mesh, face_normals);
	writeFile(mesh, argv[2]);

	return 0;
}
