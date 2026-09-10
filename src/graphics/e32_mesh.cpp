#include "psyqo/alloc.h"
#include "psyqo/kernel.hh"
#include "psyqo/gte-kernels.hh"
#include "psyqo/gte-registers.hh"
#include "common/hardware/counters.h"

#include "animation/e32_model.h"

#include "graphics/e32_mesh.h"
#include "shaders/e32_mesh_deform.h"
#include "physics/math/e32_math_common.h"
#include "physics/math/e32_quaternion.h"
#include "scene3d/e32_lighting.h"
#include "render/e32_render.h"
#include "viewport/e32_viewport.h"


static bool same(const char *a, const char *b)
{
	while (*a && *a == *b) { a++; b++; }
	return *a == *b;
}

/* The vertices' units to 20.12. */
static Fixed fromUnits(int32_t v)
{
	return Fixed(v << MODEL_CPU_TO_UNITS, Fixed::RAW);
}

/* A skinned model's boxes are written per bone, so none of them says where the
   mesh ends up. Two things bound that for good: a vertex never leaves its own
   bone by more than the model box reaches, and a bone never leaves the root by
   more than its chain is long. The box that holds both holds every pose the
   rig can take, so it is measured once and never again. */
static void mesh_skinnedBound(Mesh *mesh, const SkeletonData *skeleton)
{
	Fixed reach = Fixed();
	for (int i = 0; i < 3; i++) {
		Fixed lo = fromUnits(-mesh->model->aabbMin[i]);
		Fixed hi = fromUnits( mesh->model->aabbMax[i]);
		if (lo > reach) reach = lo;
		if (hi > reach) reach = hi;
	}

	Fixed *chain = (Fixed *)psyqo_malloc(sizeof(Fixed) * skeleton->boneCount);
	psyqo::Kernel::assert(chain != NULL, "mesh: out of memory");
	Fixed longest = Fixed();

	for (int b = 0; b < skeleton->boneCount; b++) {
		const BoneData *bone = &skeleton->bones[b];

		chain[b] = bone->position.magnitude();
		if (bone->parentIdx < b) chain[b] += chain[bone->parentIdx];
		if (chain[b] > longest) longest = chain[b];
	}

	psyqo_free(chain);

	/* back to the vertices' units */
	int16_t half = (int16_t)((longest + reach).raw() >> MODEL_CPU_TO_UNITS);
	for (int i = 0; i < 3; i++) {
		mesh->local_min[i] = -half;
		mesh->local_max[i] =  half;
	}
}

void Mesh::initBounds()
{
	bound_count = 1 + model->objectCount;
	bound = (MeshBound *)psyqo_malloc(bound_count * sizeof(MeshBound));
	psyqo::Kernel::assert(bound != NULL, "mesh: out of memory");
	for (int i = 0; i < bound_count; i++) bound[i] = MeshBound();
	culled = false;

	const SkeletonData *skel = model->getSkeleton();
	if (skel) {
		mesh_skinnedBound(this, skel);
		return;
	}

	for (int i = 0; i < 3; i++) {
		local_min[i] = model->aabbMin[i];
		local_max[i] = model->aabbMax[i];
	}
}

/* Places a model-space box in the world without walking its corners: the
   centre goes through the transform, and the half-extent through its absolute
   value, which is the axis-aligned box that still contains it after any
   rotation. */
static void mesh_placeBound(MeshBound *bound, const int16_t *min, const int16_t *max, const Transform *m)
{
	Vector3 centre, extent;
	for (int i = 0; i < 3; i++) {
		centre[i] = fromUnits(max[i] + min[i]) / 2;
		extent[i] = fromUnits(max[i] - min[i]) / 2;
	}

	Vector3 c = m->mulVector(centre);

	/* row i of the rotation, absolute, dotted with the extent */
	for (int i = 0; i < 3; i++) {
		Fixed e = m->rotation.ex[i].abs() * extent.x
		        + m->rotation.ey[i].abs() * extent.y
		        + m->rotation.ez[i].abs() * extent.z;

		bound->min[i] = c[i] - e;
		bound->max[i] = c[i] + e;
	}
}

static void mesh_updateBounds(Mesh *mesh, const Transform *matrix)
{
	if (mesh->bound == NULL) return;

	mesh_placeBound(&mesh->bound[0], mesh->local_min, mesh->local_max, matrix);

	uint8_t i = 1;
	ModelIter it = mesh->model->iterCreate(CHUNK_TYPE_OBJECT);
	while (it.next() && i < mesh->bound_count)
		mesh_placeBound(&mesh->bound[i++], it.object->aabbMin, it.object->aabbMax, matrix);
}

void Mesh::cull(const Viewport *viewport)
{
	culled = !viewport->frustum.vsAabb(bound[0].min, bound[0].max);
	if (culled) return;

	/* Only the object path draws per object; recorded meshes cut at the
	   whole-mesh box. */
	if (dl_count != 0) return;

	uint8_t b = 1;
	ModelIter it = model->iterCreate(CHUNK_TYPE_OBJECT);
	while (it.next() && b < bound_count) {
		it.object->isVisible = viewport->frustum.vsAabb(bound[b].min, bound[b].max);
		b++;
	}
}


/* Scale, then rotate, then translate: the rotation's columns carry the
   scale, the way the GTE wants a single 3x3. */
static Transform transform_fromSrt(const Vector3 *scale, const Matrix3 *rotation, const Vector3 *position)
{
	Transform t;
	t.rotation = *rotation;
	t.rotation.ex *= scale->x;
	t.rotation.ey *= scale->y;
	t.rotation.ez *= scale->z;
	t.position = *position;
	return t;
}

/* A simulated body tumbles, and euler angles cannot describe that without
   picking an order and losing the tumble at the poles. The body already keeps
   a quaternion, so it goes straight to the matrix. */
void Mesh::setMatrixFromBody(const Vector3 *position, const Quaternion *rotation,
                             const Vector3 *scale, uint8_t fb_index)
{
	Transform *matrix = &matrix_buffer[fb_index];

	Matrix3 r = rotation->toMatrix3();
	*matrix = transform_fromSrt(scale, &r, position);

	mesh_updateBounds(this, matrix);
}


void Mesh::setMatrix(const RenderTransform *transform, uint8_t fb_index)
{
	Transform *matrix = &matrix_buffer[fb_index];

	Matrix3 r = Matrix3::fromEuler(transform->rotation.x, transform->rotation.y, transform->rotation.z);
	*matrix = transform_fromSrt(&transform->scale, &r, &transform->position);

	mesh_updateBounds(this, matrix);
}


bool Mesh::setDeform(const Vector3 *source, const Vector3 *source_normal,
                     const uint8_t *source_rgba, uint16_t source_count, Fixed scale)
{
	MeshDeform *d = (MeshDeform *)psyqo_malloc(sizeof(MeshDeform));
	psyqo::Kernel::assert(d != NULL, "mesh: out of memory");

	if (!d->bind(model, source, source_normal, source_count, scale)) {
		d->destroy();
		psyqo_free(d);
		return false;
	}
	d->source_rgba = source_rgba;

	deform = d;
	return true;
}


void Mesh::updateDeform(uint8_t fb_index)
{
	if (deform) deform->apply(fb_index);
}

/* Separate from the update because it has to run at draw time, in front of
   the draw, not when the vertices are written. */
void Mesh::bindDeformFrame(uint8_t fb_index)
{
	if (deform) deform->bindFrame(fb_index);
}

const RenderVertex *Mesh::vertices(uint32_t object) const
{
	if (deform) return deform->vertices(object);
	return model->objects[object].vertices;
}


void Mesh::recordObjects()
{
	object_part = NULL;
	dl_count    = 0;
	dl_buffers  = 1;
	palette     = NULL;
	visible     = 1;
}

/* Part recording: objects named in the list get their own part, every other
   object lands together in part 0. */
void Mesh::recordParts(const char *const *names, uint8_t count)
{
	uint16_t bones = 0;

	/* A skinned mesh reads a palette per frame buffer, each over its own
	   run of the transforms. */
	palette    = NULL;
	dl_buffers = 1;
	if (skeleton) {
		bones = skeleton->skeletonRef->boneCount;
		palette = (Transform *)psyqo_malloc(sizeof(Transform) * FB_COUNT * bones);
		psyqo::Kernel::assert(palette != NULL, "mesh: out of memory");
		dl_buffers = FB_COUNT;
	}

	dl_count = 1 + count;
	object_part = (uint8_t *)psyqo_malloc(sizeof(uint8_t) * model->objectCount);
	psyqo::Kernel::assert(object_part != NULL, "mesh: out of memory");

	for (int o = 0; o < model->objectCount; o++) {
		const Object *obj = &model->objects[o];
		object_part[o] = 0;
		for (int i = 0; i < count; i++)
			if (obj->name && same(obj->name, names[i])) { object_part[o] = (uint8_t)(i + 1); break; }
	}

	visible = 1;
}

void Mesh::updatePalette(const Viewport *viewport, uint8_t fb_index)
{
	if (palette == NULL) return;

	const Armature *armature = skeleton;
	uint16_t bones = armature->skeletonRef->boneCount;

	/* The frame's part of the product is shared by every bone: the view
	   over the model transform, once. Then one product per bone. The bone
	   transforms are model space already (Armature::update composed the
	   parent chain). */
	const Transform *model_matrix = &matrix_buffer[fb_index];
	Transform view_model = viewport->view * *model_matrix;

	Transform *entry = palette + fb_index * bones;

	for (uint16_t b = 0; b < bones; b++)
		entry[b] = view_model * armature->bones[b].matrix;
}


/* ---- Drawing ----------------------------------------------------------
   An object is drawn face by face. The face's corners go through the GTE
   with the element's matrix, to camera space and to the screen, lit; a
   face wholly within the view goes to the table from there. One that
   comes close is cut into an even grid first, over the face as it is.
   Then every piece that crosses the near plane or an edge of the screen
   is cut to the view on the CPU; its new points, already in camera
   space, are projected through the identity. The importer splits
   vertices per face, so there is little to share between faces and
   nothing to keep per object. */

using namespace psyqo::GTE;

/* A vertex in camera space, in the model's units, with its projection,
   lit and fogged, as a primitive takes it: the first pass leaves both,
   so a face that needs no cutting goes to the table straight from here.
   The depth for the table is z itself. 24 bytes: what the cutting
   copies around, and it lives in the scratchpad, which holds little. */
struct CamVertex
{
	int32_t       x, y, z;
	psyqo::Vertex point;
	uint32_t      color;
	uint8_t       u, v;
	uint8_t       out;   /* the edges of the screen it is out of, as bits */
};

/* The sides of the view, as bits: the near plane and the screen's four
   edges. A piece with a corner out of one is cut on that side: the
   corner goes, and where the piece's edges cross the side, on their own
   line in camera space, new vertices come in. So every edge keeps its
   direction, instead of bending toward where the GTE stops the corner,
   1024 pixels out. A corner nearer than the near plane carries that bit
   with the sides it is out of: the side planes pass through the camera,
   so they still tell, and a piece out of one side whole is dropped. */
#define MESH_OUT_NEAR   1
#define MESH_OUT_LEFT   2
#define MESH_OUT_RIGHT  4
#define MESH_OUT_TOP    8
#define MESH_OUT_BOTTOM 16
#define MESH_OUT_NEW    128   /* a vertex cut on a side */
#define MESH_OUT_CORNER 64    /* a corner of the piece being cut, its index in the low bits */

/* A triangle cut on the five sides has at most this many vertices. */
#define MESH_POLY_MAX 8

/* The GPU maps a texture linearly across a face, and that is only right
   where the depth does not change, so what comes close is cut into pieces.
   A face with a vertex within the range is in range: it is cut into MAX
   pieces along every edge, an even grid with its corners untouched. A
   face out of range is left whole; an edge it shares with a face in
   range gets zero-area triangles over the neighbour's points, which take
   up the pixels the T junction leaves open. Nothing here depends on
   where the camera looks, only on how far things are. */
#define MESH_SUBDIV_MAX   4   /* the most pieces along an edge a level may ask */

/* Chains of points along an edge and grids over a face are this many. */
#define MESH_GRID (MESH_SUBDIV_MAX + 1)

/* The levels in force, nearest first, and their ranges in the models'
   units. Two built in, until the prefab brings its own. */
static const MeshLod mesh_lod_default[2] = { { 12.0_fp, 4 }, { 24.0_fp, 2 } };
static const MeshLod *mesh_lod       = mesh_lod_default;
static uint8_t        mesh_lod_count = 2;
static int32_t        mesh_lod_range[8];

void mesh_setLod(const MeshLod *levels, uint8_t count)
{
	psyqo::Kernel::assert(count <= 8, "mesh: too many lod levels");
	for (int i = 0; i < count; i++)
		psyqo::Kernel::assert(levels[i].pieces >= 1 && levels[i].pieces <= MESH_SUBDIV_MAX
		                      && (levels[i].pieces & (levels[i].pieces - 1)) == 0,
		                      "mesh: lod pieces must be a power of two up to the cap");
	mesh_lod       = levels;
	mesh_lod_count = count;
}

/* The scratch, one entry per vertex of the largest object drawn so far:
   what the first pass leaves for the faces to read. And one byte per
   face: its level's pieces, which its neighbours read. */
static CamVertex *mesh_cam;
static uint32_t   mesh_cam_size;
static uint8_t   *mesh_pieces;       /* per face: pieces along its edges, 1 whole */
static uint32_t   mesh_pieces_size;

/* What a face's cutting works on, in the CPU's scratchpad: the 1 KB at
   0x1f800000 that answers a load in one cycle where main memory takes
   six. A piece being cut to the view and what it makes of it; and the
   grid of a subdivided quad, which a subdivided triangle's rows and a
   covered edge's chain take over, since they never run at the same
   time. 984 bytes of the 1024; the module's state below takes the rest. */
struct MeshScratch
{
	CamVertex poly[2][MESH_POLY_MAX];
	union {
		CamVertex grid[MESH_GRID][MESH_GRID];
		CamVertex row[MESH_GRID][MESH_GRID];
		CamVertex chain[MESH_GRID];
	};
};
#define MESH_SCRATCH __attribute__((section(".scratchpad")))
static MeshScratch mesh_scratch MESH_SCRATCH;

/* The module's state while it draws, in the scratchpad as well: the
   code that runs per point and per piece reads these on every call, and
   a read from RAM costs six cycles where the arithmetic around it
   costs one. */

/* The material of the object being drawn, whether its mesh cuts its
   faces, and whether the frame has fog. */
static const Material *mesh_material  MESH_SCRATCH;
static bool            mesh_subdivide MESH_SCRATCH;

/* This frame's texture scroll of the object being drawn, base and glow. */
static uint8_t mesh_scroll_u      MESH_SCRATCH;
static uint8_t mesh_scroll_v      MESH_SCRATCH;
static uint8_t mesh_glow_scroll_u MESH_SCRATCH;
static uint8_t mesh_glow_scroll_v MESH_SCRATCH;
static bool            mesh_fog       MESH_SCRATCH;
static bool            mesh_wireframe MESH_SCRATCH;

static const psyqo::Color mesh_wire_color = {{255, 255, 255}};

/* Whether the GTE holds the identity (for projecting cut faces' points,
   already in camera space) instead of the element's matrix. */
static bool mesh_identity MESH_SCRATCH;

/* The frame, taken once per draw: what the pieces go into and the depth
   they are dropped past. */
static Render                               *mesh_render     MESH_SCRATCH;
static psyqo::OrderingTable<RENDER_OT_SIZE> *mesh_table      MESH_SCRATCH;
static int32_t                               mesh_near       MESH_SCRATCH;
static uint32_t                              mesh_far        MESH_SCRATCH;
static int32_t                               mesh_projection MESH_SCRATCH;   /* the GTE's H */

uint32_t mesh_profile_faces;
uint32_t mesh_profile_transform;
uint32_t mesh_profile_emit;

/* Adds the ticks of root counter 2 spent in a scope to a total. */
struct MeshProfile
{
	uint16_t  began;
	uint32_t *total;
	__attribute__((always_inline)) MeshProfile(uint32_t *t) : began(COUNTERS[2].value), total(t) {}
	__attribute__((always_inline)) ~MeshProfile() { *total += (COUNTERS[2].value - began) & 0xFFFF; }
};


/* The instruction cache is 4 KB, direct mapped: code 4 KB apart evicts
   each other. Everything that runs per piece (the face loop, the grid,
   the cutting, the emission and their helpers) goes together in the
   linker's hot group, in this file's order, so it takes one run of
   addresses and only its two ends can alias. What runs once per element,
   only for triangles, only at the range's border, or only in debug stays
   out of line, elsewhere. */
#define MESH_HOT __attribute__((section(".text.hot.mesh")))
#define MESH_COLD __attribute__((noinline))

/* Which sides of the view a camera space point is out of: from its
   position, since the GTE's screen point stops at 1024 pixels. The
   comparison is scaled by the depth, so nothing is divided. The side
   planes pass through the camera, so the test holds behind the near
   plane too: a piece with every corner out of one side is out of the
   view wherever its corners are in depth, and is dropped without being
   cut. */
MESH_HOT
static inline void mesh_outcode(CamVertex *c)
{
	int32_t xh = c->x * mesh_projection, yh = c->y * mesh_projection;
	int32_t gx = (SCREEN_WIDTH / 2) * c->z, gy = (SCREEN_HEIGHT / 2) * c->z;

	uint8_t out = 0;
	if (c->z < mesh_near) out |= MESH_OUT_NEAR;
	if (xh >  gx) out |= MESH_OUT_RIGHT;
	if (xh < -gx) out |= MESH_OUT_LEFT;
	if (yh >  gy) out |= MESH_OUT_BOTTOM;
	if (yh < -gy) out |= MESH_OUT_TOP;
	c->out = out;
}



/* Loads one transform into the GTE: the rotation as 4.12, the translation
   in the vertices' units. */
static void mesh_loadMatrix(const Transform *t)
{
	const Matrix3 &r = t->rotation;

	psyqo::Matrix33 m = {{
		{ r.ex.x, r.ey.x, r.ez.x },
		{ r.ex.y, r.ey.y, r.ez.y },
		{ r.ex.z, r.ey.z, r.ez.z },
	}};
	writeUnsafe<PseudoRegister::Rotation>(m);

	/* 20.12 meters to the vertices' units. */
	write<Register::TRX, Unsafe>((uint32_t)(t->position.x.raw() >> MODEL_CPU_TO_UNITS));
	write<Register::TRY, Unsafe>((uint32_t)(t->position.y.raw() >> MODEL_CPU_TO_UNITS));
	write<Register::TRZ, Safe>((uint32_t)(t->position.z.raw() >> MODEL_CPU_TO_UNITS));
}

/* The identity: the second pass feeds camera space vertices. */
static void mesh_loadIdentity()
{
	psyqo::Matrix33 m = {{
		{ 1.0_fp, Fixed(), Fixed() },
		{ Fixed(), 1.0_fp, Fixed() },
		{ Fixed(), Fixed(), 1.0_fp },
	}};
	writeUnsafe<PseudoRegister::Rotation>(m);

	clear<Register::TRX, Unsafe>();
	clear<Register::TRY, Unsafe>();
	clear<Register::TRZ, Safe>();
}


/* Vector 0 and the color, written straight into the GTE. PSYQo's write<>
   comes out as a call at this optimization level, and these run for every
   corner of every face. The two idle cycles are the GTE's load delay
   before a command reads what was written. */
static inline void mesh_gteVector(uint32_t xy, uint32_t z)
{
#ifndef PS1_PC_PORT
	asm volatile("mtc2 %0, $0\n\tmtc2 %1, $1\n\tnop\n\tnop" : : "r"(xy), "r"(z));
#endif
}

static inline void mesh_gteColor(uint32_t rgb)
{
#ifndef PS1_PC_PORT
	asm volatile("mtc2 %0, $6\n\tnop\n\tnop" : : "r"(rgb));
#endif
}

/* What the corners of an object's faces share: how their color is made
   and whether it fades. Held in registers across the face loop. */
struct MeshShade
{
	bool     textured;
	bool     fog;
	uint32_t r, g, b;   /* the material color, for untextured faces */
};

/* One model vertex through the GTE, with the element's matrix loaded:
   to camera space and to the screen, and lit. */
static inline void mesh_vertex(const RenderVertex *v, const MeshShade *shade, CamVertex *c)
{
	/* The projection leaves the camera space point in the accumulators,
	   the screen point in the FIFO and the depth factor for the fog. */
	mesh_gteVector(*(const uint32_t *)&v->x, (uint16_t)v->z);
	Kernels::rtps();
	c->x = (int32_t)readRaw<Register::MAC1>();
	c->y = (int32_t)readRaw<Register::MAC2>();
	c->z = (int32_t)readRaw<Register::MAC3>();
	c->point.packed = readRaw<Register::SXY2>();
	mesh_outcode(c);

	/* A textured face's color modulates the texture, with 128 as one; a
	   flat one carries the material color itself. */
	uint32_t color;
	if (shade->textured) {
		color = ((uint32_t)v->r >> 1) | (((uint32_t)v->g >> 1) << 8) | (((uint32_t)v->b >> 1) << 16);
	} else {
		color = (((uint32_t)v->r * shade->r) >> 8)
		      | ((((uint32_t)v->g * shade->g) >> 8) << 8)
		      | ((((uint32_t)v->b * shade->b) >> 8) << 16);
	}

	mesh_gteVector(*(const uint32_t *)&v->nx, (uint16_t)v->nz);
	mesh_gteColor(color);
	if (shade->fog) Kernels::ncds();
	else            Kernels::nccs();
	c->color = readRaw<Register::RGB2>();

	c->u = v->u;
	c->v = v->v;
}

/* First pass: every vertex of the object into the scratch. The element's
   matrix is loaded here; a skinned object reloads it per bone. */
MESH_COLD
static void mesh_transform(const Element3D *element, const Object *object,
                           const RenderVertex *verts, const Transform *view_model)
{
	const uint8_t *bones = element->palette ? object->boneIndices : NULL;
	int last_bone = -1;

	MeshShade shade = {
		mesh_material->textured, mesh_fog,
		mesh_material->color.r, mesh_material->color.g, mesh_material->color.b,
	};

	if (!bones) mesh_loadMatrix(view_model);

	for (uint32_t i = 0; i < object->vertexCount; i++) {
		if (bones && bones[i] != last_bone) {
			last_bone = bones[i];
			mesh_loadMatrix(&element->palette[last_bone]);
		}
		mesh_vertex(&verts[i], &shade, &mesh_cam[i]);
	}
}


/* The point at 't' (0..4096) between two camera space vertices. */
MESH_HOT
static void mesh_lerp(const CamVertex *a, const CamVertex *b, int32_t t, CamVertex *out)
{
	out->x = a->x + (((b->x - a->x) * t) >> 12);
	out->y = a->y + (((b->y - a->y) * t) >> 12);
	out->z = a->z + (((b->z - a->z) * t) >> 12);

	uint32_t color = 0;
	for (int shift = 0; shift < 24; shift += 8) {
		int32_t ca = (a->color >> shift) & 0xFF;
		int32_t cb = (b->color >> shift) & 0xFF;
		color |= (uint32_t)(ca + (((cb - ca) * t) >> 12)) << shift;
	}
	out->color = color;

	out->u = (uint8_t)(a->u + ((((int32_t)b->u - a->u) * t) >> 12));
	out->v = (uint8_t)(a->v + ((((int32_t)b->v - a->v) * t) >> 12));
}

/* The middle of two camera space vertices. Unlike a lerp it does not care
   which end comes first, so two faces walking their shared edge in
   opposite directions land on the same points and leave no seam. */
MESH_HOT
static void mesh_mid(const CamVertex *a, const CamVertex *b, CamVertex *out)
{
	out->x = (a->x + b->x) >> 1;
	out->y = (a->y + b->y) >> 1;
	out->z = (a->z + b->z) >> 1;

	/* byte channels, no carry between them: the top bit of each is
	   masked before the halving */
	out->color = ((a->color & 0xFEFEFE) >> 1) + ((b->color & 0xFEFEFE) >> 1)
	               + (a->color & b->color & 0x010101);

	out->u = (uint8_t)(((int)a->u + b->u) >> 1);
	out->v = (uint8_t)(((int)a->v + b->v) >> 1);
}

/* Fills 'chain', every 'stride' entries, with a to b in 'n' (a power of
   two) steps by halving: a column of the grid as well as a row. An end
   already in place is left there. */
MESH_HOT
static void mesh_halve(CamVertex *chain, int stride, int lo, int hi)
{
	if (hi - lo < 2) return;
	int mid = (lo + hi) / 2;
	mesh_mid(&chain[lo * stride], &chain[hi * stride], &chain[mid * stride]);
	mesh_halve(chain, stride, lo, mid);
	mesh_halve(chain, stride, mid, hi);
}

MESH_HOT
static void mesh_bisect(const CamVertex *a, const CamVertex *b, int n, CamVertex *chain, int stride)
{
	CamVertex *last = &chain[n * stride];
	if (chain != a) *chain = *a;
	if (last  != b) *last  = *b;
	mesh_halve(chain, stride, 0, n);
}


/* One point of a cut face: projected, through the identity, being in
   camera space already. The color, fog and all, was interpolated between
   the face's vertices with the point. */
MESH_HOT
static inline void mesh_projectOutcoded(CamVertex *c)
{
	/* nearer than the near plane, it is never drawn as it is: no point
	   projecting it, and the GTE could not */
	if (c->out & MESH_OUT_NEAR) return;

	if (!mesh_identity) {
		mesh_loadIdentity();
		mesh_identity = true;
	}

	int32_t x = c->x, y = c->y;
	if (x >  32767) x =  32767;
	if (x < -32768) x = -32768;
	if (y >  32767) y =  32767;
	if (y < -32768) y = -32768;

	mesh_gteVector(((uint32_t)y << 16) | ((uint32_t)x & 0xFFFF), (uint16_t)c->z);
	Kernels::rtps();
	c->point.packed = readRaw<Register::SXY2>();
}

MESH_HOT
static inline void mesh_project(CamVertex *c)
{
	mesh_outcode(c);
	mesh_projectOutcoded(c);
}


/* The outline of a piece in wireframe, 'd' NULL for a triangle. Debug:
   out of the hot group. */
MESH_COLD
static void mesh_emitWire(const CamVertex *a, const CamVertex *b,
                          const CamVertex *c, const CamVertex *d, uint32_t otz)
{
	Render &render = *mesh_render;

	if (d) {
		WireQuadFragment *f = render.alloc<WireQuadFragment>();
		if (!f) return;

		psyqo::Prim::PolyLine<4> &l = f->primitive;
		l.setColor(mesh_wire_color);
		l.points[0] = a->point;
		l.points[1] = b->point;
		l.points[2] = c->point;
		l.points[3] = d->point;
		l.points[4] = a->point;

		mesh_table->insert(*f, otz);
	} else {
		WireTriFragment *f = render.alloc<WireTriFragment>();
		if (!f) return;

		psyqo::Prim::PolyLine<3> &l = f->primitive;
		l.setColor(mesh_wire_color);
		l.points[0] = a->point;
		l.points[1] = b->point;
		l.points[2] = c->point;
		l.points[3] = a->point;

		mesh_table->insert(*f, otz);
	}
}

/* A piece of an untextured material, carrying its color, 'd' NULL for a
   triangle. The quad in contour order: the GPU takes it as two triangles
   ABC and BCD, so the contour's third and fourth swap. Out of the hot
   group: the scene's materials are textured. */
MESH_COLD
static void mesh_emitFlat(const CamVertex *a, const CamVertex *b,
                          const CamVertex *c, const CamVertex *d, uint32_t otz)
{
	Render &render = *mesh_render;

	if (d) {
		FlatQuadFragment *f = render.alloc<FlatQuadFragment>();
		if (!f) return;

		psyqo::Prim::GouraudQuad &q = f->primitive;
		q.setColorA({ .packed = a->color });
		q.colorB.packed = b->color;
		q.colorC.packed = d->color;
		q.colorD.packed = c->color;
		q.pointA = a->point;
		q.pointB = b->point;
		q.pointC = d->point;
		q.pointD = c->point;

		mesh_table->insert(*f, otz);
	} else {
		FlatTriFragment *f = render.alloc<FlatTriFragment>();
		if (!f) return;

		psyqo::Prim::GouraudTriangle &t = f->primitive;
		t.setColorA({ .packed = a->color });
		t.colorB.packed = b->color;
		t.colorC.packed = c->color;
		t.pointA = a->point;
		t.pointB = b->point;
		t.pointC = c->point;

		mesh_table->insert(*f, otz);
	}
}

/* A triangle into the table. */
MESH_HOT
static void mesh_emitTri(const CamVertex *a, const CamVertex *b, const CamVertex *c)
{
	MeshProfile profile(&mesh_profile_emit);
	const Material *mat = mesh_material;

	/* Average depth to a bucket, one unit each. */
	uint32_t otz = ((a->z + b->z + c->z) * 1365) >> 12;
	if (otz == 0 || otz >= RENDER_OT_SIZE || otz > mesh_far) return;

	if (mesh_wireframe)  { mesh_emitWire(a, b, c, NULL, otz); return; }
	if (!mat->textured) { mesh_emitFlat(a, b, c, NULL, otz); return; }

	TexturedTriFragment *f = mesh_render->alloc<TexturedTriFragment>();
	if (!f) return;
	f->primitive.window = mat->slot.window;

	uint8_t su = mat->slot.u + mesh_scroll_u;
	uint8_t sv = mat->slot.v + mesh_scroll_v;

	psyqo::Prim::GouraudTexturedTriangle &t = f->primitive.tri;
	t.setColorA({ .packed = a->color });
	t.colorB.packed = b->color;
	t.colorC.packed = c->color;
	t.pointA = a->point;
	t.pointB = b->point;
	t.pointC = c->point;
	t.uvA = { (uint8_t)(su + a->u), (uint8_t)(sv + a->v) };
	t.uvB = { (uint8_t)(su + b->u), (uint8_t)(sv + b->v) };
	t.uvC = { (uint8_t)(su + c->u), (uint8_t)(sv + c->v), 0 };
	t.tpage = mat->slot.tpage;
	if (mat->color.user < 255) t.setSemiTrans();

	/* The glow goes in first: within a bucket the last fragment in is the
	   first the GPU draws, and the glow has to land over the base. */
	if (mat->glow) {
		TexturedTriFragment *g = mesh_render->alloc<TexturedTriFragment>();
		if (g) {
			g->primitive = f->primitive;
			g->primitive.window = mat->glow_slot.window;

			uint8_t gu = mat->glow_slot.u + mesh_glow_scroll_u;
			uint8_t gv = mat->glow_slot.v + mesh_glow_scroll_v;

			psyqo::Prim::GouraudTexturedTriangle &gt = g->primitive.tri;
			gt.uvA = { (uint8_t)(gu + a->u), (uint8_t)(gv + a->v) };
			gt.uvB = { (uint8_t)(gu + b->u), (uint8_t)(gv + b->v) };
			gt.uvC = { (uint8_t)(gu + c->u), (uint8_t)(gv + c->v), 0 };
			gt.tpage = mat->glow_slot.tpage;
			gt.setSemiTrans();

			mesh_table->insert(*g, otz);
		}
	}

	mesh_table->insert(*f, otz);
}

/* A quad into the table, in contour order. The GPU takes it as two
   triangles ABC and BCD, so the contour's third and fourth swap. */
MESH_HOT
static void mesh_emitQuad(const CamVertex *a, const CamVertex *b,
                          const CamVertex *c, const CamVertex *d)
{
	MeshProfile profile(&mesh_profile_emit);
	const Material *mat = mesh_material;

	uint32_t otz = (a->z + b->z + c->z + d->z) >> 2;
	if (otz == 0 || otz >= RENDER_OT_SIZE || otz > mesh_far) return;

	if (mesh_wireframe)  { mesh_emitWire(a, b, c, d, otz); return; }
	if (!mat->textured) { mesh_emitFlat(a, b, c, d, otz); return; }

	TexturedQuadFragment *f = mesh_render->alloc<TexturedQuadFragment>();
	if (!f) return;
	f->primitive.window = mat->slot.window;

	uint8_t su = mat->slot.u + mesh_scroll_u;
	uint8_t sv = mat->slot.v + mesh_scroll_v;

	psyqo::Prim::GouraudTexturedQuad &q = f->primitive.quad;
	q.setColorA({ .packed = a->color });
	q.colorB.packed = b->color;
	q.colorC.packed = d->color;
	q.colorD.packed = c->color;
	q.pointA = a->point;
	q.pointB = b->point;
	q.pointC = d->point;
	q.pointD = c->point;
	q.uvA = { (uint8_t)(su + a->u), (uint8_t)(sv + a->v) };
	q.uvB = { (uint8_t)(su + b->u), (uint8_t)(sv + b->v) };
	q.uvC = { (uint8_t)(su + d->u), (uint8_t)(sv + d->v), 0 };
	q.uvD = { (uint8_t)(su + c->u), (uint8_t)(sv + c->v), 0 };
	q.tpage = mat->slot.tpage;
	if (mat->color.user < 255) q.setSemiTrans();

	/* The glow goes in first: within a bucket the last fragment in is the
	   first the GPU draws, and the glow has to land over the base. */
	if (mat->glow) {
		TexturedQuadFragment *g = mesh_render->alloc<TexturedQuadFragment>();
		if (g) {
			g->primitive = f->primitive;
			g->primitive.window = mat->glow_slot.window;

			uint8_t gu = mat->glow_slot.u + mesh_glow_scroll_u;
			uint8_t gv = mat->glow_slot.v + mesh_glow_scroll_v;

			psyqo::Prim::GouraudTexturedQuad &gq = g->primitive.quad;
			gq.uvA = { (uint8_t)(gu + a->u), (uint8_t)(gv + a->v) };
			gq.uvB = { (uint8_t)(gu + b->u), (uint8_t)(gv + b->v) };
			gq.uvC = { (uint8_t)(gu + d->u), (uint8_t)(gv + d->v), 0 };
			gq.uvD = { (uint8_t)(gu + c->u), (uint8_t)(gv + c->v), 0 };
			gq.tpage = mat->glow_slot.tpage;
			gq.setSemiTrans();

			mesh_table->insert(*g, otz);
		}
	}

	mesh_table->insert(*f, otz);
}


/* Whether the face being drawn was already found to look at the camera.
   A face with a corner off the screen cannot be judged from its screen
   points, which the GTE stops at 1024 pixels: its pieces judge
   themselves once cut to the screen. */
static bool mesh_face_tested MESH_SCRATCH;

/* The winding of a polygon on screen: the signed area comes out negative
   when the outside looks at the camera, the screen's y growing downward.
   Zero is not away: a sliver a pixel wide cut off a piece comes out at
   zero in whole pixels, and dropped it would leave a line of background
   between its neighbours. */
MESH_HOT
static bool mesh_facesAway(const CamVertex *const *v, int n)
{
	int32_t area = 0;
	for (int i = 0; i < n; i++) {
		const psyqo::Vertex &a = v[i]->point, &b = v[i + 1 == n ? 0 : i + 1]->point;
		area += a.x * b.y - b.x * a.y;
	}
	return area > 0;
}

/* How far a camera space point is inside one side of the view, negative
   outside: the near plane by depth, the screen's edges by the screen
   coordinate scaled by the depth, nothing divided. Per side, in bit
   order: what multiplies the depth, and what multiplies x and y before
   the projection distance. */
static const int32_t mesh_side_coef[5][3] = {
	{ 1,                 0,  0 },   /* near: the depth, less the plane's */
	{ SCREEN_WIDTH  / 2,  1,  0 },   /* left */
	{ SCREEN_WIDTH  / 2, -1,  0 },   /* right */
	{ SCREEN_HEIGHT / 2,  0,  1 },   /* top */
	{ SCREEN_HEIGHT / 2,  0, -1 },   /* bottom */
};

MESH_HOT
static int32_t mesh_inside(const CamVertex *c, int side)
{
	const int32_t *k = mesh_side_coef[side];
	int32_t d = k[0] * c->z + (k[1] * c->x + k[2] * c->y) * mesh_projection;
	return side == 0 ? d - mesh_near : d;
}

/* Where an edge crosses a side, 0..4096 from its first end: the first
   end's distance over the two ends' span, both brought down to 18 bits
   first so the division fits 32. */
MESH_HOT
static int32_t mesh_crossing(int32_t da, int32_t db)
{
	int32_t num = da, den = da - db;
	while (num > 0x3FFFF || num < -0x3FFFF || den > 0x3FFFF || den < -0x3FFFF) {
		num /= 2;
		den /= 2;
	}
	if (den == 0) return 0;
	return (num << 12) / den;
}

/* Cuts a piece to the view, one side at a time, the near plane first
   (Sutherland-Hodgman): a corner inside stays as it is, an edge of the
   piece that crosses gets a new vertex where it meets the side, on its
   own line in camera space, and a corner outside goes. The result is in
   the scratchpad, its new vertices projected; 'result' gets a pointer
   per vertex. Zero when nothing is left. */
MESH_HOT
static int mesh_clip(const CamVertex *const *v, int n, uint8_t sides, const CamVertex **result)
{
	CamVertex *in = mesh_scratch.poly[0], *out = mesh_scratch.poly[1];
	for (int i = 0; i < n; i++) {
		in[i] = *v[i];
		in[i].out = MESH_OUT_CORNER | i;   /* marked as the piece's own, for the fan */
	}

	for (int side = 0; side < 5; side++) {
		if (!(sides & (1 << side))) continue;

		int count = 0;
		for (int i = 0; i < n; i++) {
			const CamVertex *a = &in[i], *b = &in[i + 1 == n ? 0 : i + 1];
			int32_t da = mesh_inside(a, side), db = mesh_inside(b, side);

			if (da >= 0) out[count++] = *a;
			if ((da >= 0) != (db >= 0)) {
				/* the crossing walked from the same end whichever piece
				   the edge belongs to, so both land on the same point and
				   leave no seam */
				CamVertex *cut = &out[count++];
				bool a_first = a->x != b->x ? a->x < b->x
				             : a->y != b->y ? a->y < b->y : a->z < b->z;
				if (a_first) mesh_lerp(a, b, mesh_crossing(da, db), cut);
				else         mesh_lerp(b, a, mesh_crossing(db, da), cut);
				/* on the near plane, not a rounding short of it: nearer,
				   it would count as behind and go unprojected */
				if (cut->z < mesh_near) cut->z = mesh_near;
				cut->out = MESH_OUT_NEW;
			}
		}
		n = count;
		if (n < 3) return 0;

		/* The vertex cut on the near plane can land off any edge of the
		   screen, whatever the corners it came from were out of: what it
		   is out of joins the sides to cut. */
		if (side == 0) {
			for (int i = 0; i < n; i++) {
				if (!(out[i].out & MESH_OUT_NEW)) continue;
				mesh_outcode(&out[i]);
				sides |= out[i].out;
				out[i].out = MESH_OUT_NEW;
			}
		}

		CamVertex *swap = in; in = out; out = swap;
	}

	for (int i = 0; i < n; i++) {
		if (in[i].out & MESH_OUT_NEW) {
			mesh_project(&in[i]);
			in[i].out = MESH_OUT_NEW;   /* on the side by construction; the mark stays */
		}
		result[i] = &in[i];
	}
	return n;
}

/* A triangle cut to the view into the table. Three or four vertices go
   as they are, a triangle or a quad. More are a fan of triangles from
   one of the triangle's own corners, never from a vertex cut on a side,
   so every inner edge runs from a real corner to a point on a real
   edge. */
MESH_HOT
static void mesh_emitPoly(const CamVertex *const *p, int m)
{
	if (m == 3) {
		mesh_emitTri(p[0], p[1], p[2]);
		return;
	}
	if (m == 4) {
		mesh_emitQuad(p[0], p[1], p[2], p[3]);
		return;
	}

	int apex = 0;
	for (int i = 0; i < m; i++)
		if (p[i]->out & MESH_OUT_CORNER) { apex = i; break; }

	for (int i = 1; i + 1 < m; i++)
		mesh_emitTri(p[apex], p[(apex + i) % m], p[(apex + i + 1) % m]);
}

/* A triangle cut to the view: dropped when nothing is left, or when it
   looks away and the face was not judged whole. */
MESH_HOT
static void mesh_cutTri(const CamVertex *const *v, uint8_t sides)
{
	const CamVertex *poly[MESH_POLY_MAX];
	int m = mesh_clip(v, 3, sides, poly);
	if (m < 3) return;
	if (!mesh_face_tested && mesh_facesAway(poly, m)) return;
	mesh_emitPoly(poly, m);
}

/* One piece, a triangle or a quad in camera space, into the table: as it
   is when wholly within the view, dropped when wholly out of one side,
   cut to the view otherwise. A quad to be cut is split first along its
   real diagonal, the one the GPU splits a whole quad on, into its two
   triangles, and each is cut on its own: the only inner edge is that
   diagonal, pointing where the far corner falls, on screen or off. */
MESH_HOT
static void mesh_piece(const CamVertex *const *v, int n)
{
	uint8_t any = 0, all = 0xFF;
	for (int k = 0; k < n; k++) {
		any |= v[k]->out;
		all &= v[k]->out;
	}
	if (all) return;

	if (!any) {
		if (!mesh_face_tested && mesh_facesAway(v, n)) return;
		if (n == 3) mesh_emitTri(v[0], v[1], v[2]);
		else        mesh_emitQuad(v[0], v[1], v[2], v[3]);
		return;
	}

	if (n == 3) {
		mesh_cutTri(v, any);
		return;
	}

	const CamVertex *first[3]  = { v[0], v[1], v[3] };
	const CamVertex *second[3] = { v[1], v[2], v[3] };
	mesh_cutTri(first, any);
	mesh_cutTri(second, any);
}

/* Whether a cell is wholly out of one side of the view, and so dropped
   before its points are projected: the outcode comes from the camera
   space position, without the GTE. */
MESH_HOT
static inline bool mesh_cellOut(const CamVertex *const *cell, int n)
{
	uint8_t all = 0xFF;
	for (int k = 0; k < n; k++) all &= cell[k]->out;
	return all != 0;
}

/* Projects a cell's points, each once: the grids mark what they have
   projected in 'done', by the point's index. */
MESH_HOT
static inline void mesh_projectCell(const CamVertex *const *cell, int n, CamVertex *base, uint8_t *done)
{
	for (int k = 0; k < n; k++) {
		CamVertex *c = (CamVertex *)cell[k];
		int index = c - base;
		if (done[index]) continue;
		done[index] = 1;
		mesh_projectOutcoded(c);
	}
}

/* A quad in range, cut 'n' pieces along every edge: an even grid over
   its bilinear surface, the corners its own and the edges bisected the
   same way as from the other side; then cell by cell. Every point gets
   its outcode, but only the points of the cells that are not dropped
   whole go through the GTE, once each: a face mostly off screen would
   otherwise project a grid to throw it away. */
MESH_HOT
static void mesh_subdivideQuad(const CamVertex *const *v, int n)
{
	CamVertex (*grid)[MESH_GRID] = mesh_scratch.grid;
	uint8_t done[MESH_GRID * MESH_GRID] = {};

	/* the two edges into the grid's first and last columns, then each
	   row between its two ends, all in place */
	mesh_bisect(v[0], v[1], n, &grid[0][0], MESH_GRID);
	mesh_bisect(v[3], v[2], n, &grid[0][n], MESH_GRID);

	for (int i = 0; i <= n; i++) {
		mesh_bisect(&grid[i][0], &grid[i][n], n, grid[i], 1);
		for (int j = 0; j <= n; j++)
			mesh_outcode(&grid[i][j]);
	}

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			const CamVertex *cell[4] = { &grid[i][j], &grid[i + 1][j], &grid[i + 1][j + 1], &grid[i][j + 1] };
			if (mesh_cellOut(cell, 4)) continue;
			mesh_projectCell(cell, 4, &grid[0][0], done);
			mesh_piece(cell, 4);
		}
	}
}

/* A triangle in range, cut 'n' pieces along every edge: rows of points
   from the first vertex toward the opposite edge, every edge bisected
   the same way as from the other side; then n squared triangles, the
   middle of each row pointing back at the first vertex. */
MESH_COLD
static void mesh_subdivideTri(const CamVertex *const *v, int n)
{
	CamVertex (*row)[MESH_GRID] = mesh_scratch.row;
	uint8_t done[MESH_GRID * MESH_GRID] = {};

	/* the two edges from the first vertex down the rows' first and last
	   entries: row i starts at its left edge point and ends at its right
	   one */
	mesh_bisect(v[0], v[1], n, &row[0][0], MESH_GRID);
	mesh_bisect(v[0], v[2], n, &row[0][0], MESH_GRID + 1);

	for (int i = 1; i <= n; i++) {
		/* row i has i steps; the last row is the far edge itself */
		if ((i & (i - 1)) == 0) {
			mesh_bisect(&row[i][0], &row[i][i], i, row[i], 1);
		} else {
			for (int j = 1; j < i; j++)
				mesh_lerp(&row[i][0], &row[i][i], (j << 12) / i, &row[i][j]);
		}
		for (int j = 0; j <= i; j++)
			mesh_outcode(&row[i][j]);
	}

	for (int i = 1; i <= n; i++) {
		for (int j = 0; j < i; j++) {
			const CamVertex *up[3] = { &row[i - 1][j], &row[i][j], &row[i][j + 1] };
			if (!mesh_cellOut(up, 3)) {
				mesh_projectCell(up, 3, &row[0][0], done);
				mesh_piece(up, 3);
			}
			if (j < i - 1) {
				const CamVertex *down[3] = { &row[i - 1][j + 1], &row[i - 1][j], &row[i][j + 1] };
				if (!mesh_cellOut(down, 3)) {
					mesh_projectCell(down, 3, &row[0][0], done);
					mesh_piece(down, 3);
				}
			}
		}
	}
}

/* Covers edge 'k' of a face cut 'own' pieces (1 for a whole one) where
   the neighbour across is cut 'fine', more: over each of the face's own
   pieces of the edge, a zero-area triangle from its start over the
   neighbour's points inside it, bisected the same way as from the other
   side (Hoppe's geometry clipmaps). Collinear in the world, the triangle
   takes the pixel or two of area that the rounding gives the neighbour's
   point off the straight edge: exactly the pixels neither face paints,
   since it shares its projected points with both. Its winding means
   nothing, so it is never culled. */
MESH_COLD
static void mesh_cover(const CamVertex *const *v, int n, int k, int own, int fine)
{
	bool tested = mesh_face_tested;
	mesh_face_tested = true;

	CamVertex *chain = mesh_scratch.chain;
	mesh_bisect(v[k], v[k + 1 == n ? 0 : k + 1], fine, chain, 1);
	for (int i = 1; i < fine; i++) mesh_project(&chain[i]);

	int r = fine / own;   /* the neighbour's pieces per piece of the face */
	for (int s = 0; s < fine; s += r) {
		for (int i = 1; i < r; i++) {
			const CamVertex *tri[3] = { &chain[s], &chain[s + i], &chain[s + i + 1] };
			mesh_piece(tri, 3);
		}
	}

	mesh_face_tested = tested;
}


/* One triangle or quad in camera space, as the model has it: culled by
   its winding, and drawn whole or in pieces, 'own' of them along each
   edge, then covered on the edges 'cover' names, where a finer neighbour
   has more points; every piece is cut to the view on its own. */
MESH_HOT
static void mesh_prim(const CamVertex *const *v, int n, int own, const int *cover)
{
	/* Winding, from the screen points when every corner is on screen; a
	   face with a corner off it is judged piece by piece, once cut. */
	uint8_t any = 0;
	for (int k = 0; k < n; k++) any |= v[k]->out;
	mesh_face_tested = any == 0;
	if (mesh_face_tested && mesh_facesAway(v, n)) return;

	if (own == 1)     mesh_piece(v, n);
	else if (n == 3)  mesh_subdivideTri(v, own);
	else              mesh_subdivideQuad(v, own);

	/* the edges a finer neighbour puts more points on */
	if (cover)
		for (int k = 0; k < n; k++)
			if (cover[k]) mesh_cover(v, n, k, own, cover[k]);
}


/* One object of one element into the table. */
MESH_HOT MESH_COLD
static void mesh_drawObject(const Mesh *mesh, const Element3D *element,
                            const Transform *view_model, uint32_t index)
{
	const Object *object = &mesh->model->objects[index];
	mesh_material  = object->material;
	mesh_subdivide = mesh->subdivide;
	if (!mesh_material) return;

	const ModelDrawConf *conf = element->conf;
	mesh_scroll_u      = conf ? conf->scroll_u      : 0;
	mesh_scroll_v      = conf ? conf->scroll_v      : 0;
	mesh_glow_scroll_u = conf ? conf->glow_scroll_u : 0;
	mesh_glow_scroll_v = conf ? conf->glow_scroll_v : 0;

	if (object->vertexCount > mesh_cam_size) {
		if (mesh_cam) psyqo_free(mesh_cam);
		mesh_cam = (CamVertex *)psyqo_malloc(sizeof(CamVertex) * object->vertexCount);
		psyqo::Kernel::assert(mesh_cam != NULL, "mesh: out of memory");
		mesh_cam_size = object->vertexCount;
	}

	if (mesh_subdivide && object->faceCount > mesh_pieces_size) {
		if (mesh_pieces) psyqo_free(mesh_pieces);
		mesh_pieces = (uint8_t *)psyqo_malloc(object->faceCount);
		psyqo::Kernel::assert(mesh_pieces != NULL, "mesh: out of memory");
		mesh_pieces_size = object->faceCount;
	}

	uint32_t faces_began = COUNTERS[1].value;

	{
		MeshProfile profile(&mesh_profile_transform);
		mesh_transform(element, object, mesh->vertices(index), view_model);
	}

	/* From here the GTE only projects the points of cut faces, already in
	   camera space. */
	mesh_identity = false;

	int32_t far = (int32_t)mesh_far;

	/* Every face's level, before any is drawn: a face asks its neighbours'
	   wherever they come in the list. The nearest level whose range holds
	   the face's nearest vertex. */
	if (mesh_subdivide) {
		for (int l = 0; l < mesh_lod_count; l++)
			mesh_lod_range[l] = mesh_lod[l].range.raw() >> MODEL_CPU_TO_UNITS;

		for (uint32_t i = 0; i < object->faceCount; i++) {
			const Face *face = &object->faces[i];
			int n = face->v[3] == MODEL_NO_VERTEX ? 3 : 4;
			int32_t minz = 0x7FFFFFFF;
			for (int k = 0; k < n; k++)
				if (mesh_cam[face->v[k]].z < minz) minz = mesh_cam[face->v[k]].z;

			uint8_t pieces = 1;
			for (int l = 0; l < mesh_lod_count; l++)
				if (minz <= mesh_lod_range[l]) { pieces = mesh_lod[l].pieces; break; }
			mesh_pieces[i] = pieces;
		}
	}

	for (uint32_t i = 0; i < object->faceCount; i++) {
		const Face *face = &object->faces[i];
		int n = face->v[3] == MODEL_NO_VERTEX ? 3 : 4;

		const CamVertex *p[4];
		int32_t minz = 0x7FFFFFFF, maxz = -0x7FFFFFFF;
		uint8_t all = 0xFF;
		for (int k = 0; k < n; k++) {
			p[k] = &mesh_cam[face->v[k]];
			if (p[k]->z < minz) minz = p[k]->z;
			if (p[k]->z > maxz) maxz = p[k]->z;
			all &= p[k]->out;
		}

		/* Whole face out of one side of the view, past the far plane, or
		   too deep for the GTE's 16 bits. */
		if (all || minz > far || maxz > 32767) continue;

		/* its own pieces, and per edge the neighbour's when finer */
		int own = 1, cover[4], any_cover = 0;
		if (mesh_subdivide) {
			own = mesh_pieces[i];
			const uint16_t *across = &object->adjacent[i * 4];
			for (int k = 0; k < n; k++) {
				int nb = across[k] != MODEL_NO_FACE ? mesh_pieces[across[k]] : 1;
				cover[k] = nb > own ? nb : 0;
				any_cover |= cover[k];
			}
		}

		mesh_prim(p, n, own, any_cover ? cover : NULL);
	}

	mesh_profile_faces += (COUNTERS[1].value - faces_began) & 0xFFFF;
}


void Mesh::draw(const Element3D *element) const
{
	Render &render = Render::get();
	mesh_render = &render;
	mesh_table  = render.table();
	mesh_near   = (int32_t)render.near();
	mesh_far    = render.far();
	mesh_fog    = render.fog();
	mesh_wireframe  = render.wireframe();
	mesh_projection = render.projection();

	/* The element's placement, folded with the view: what the GTE turns
	   every vertex through. A skinned element swaps it per bone instead. */
	Transform view_model = element->matrix ? *render.view() * *element->matrix : *render.view();

	static const Transform identity = Transform::identity();
	Lighting::set(Lighting::get(), element->matrix ? element->matrix : &identity);

	for (uint32_t o = 0; o < model->objectCount; o++) {
		const Object *object = &model->objects[o];

		if (dl_count == 0) {
			if (!object->isVisible) continue;
			if (element->conf && element->conf->filterCb
			    && !element->conf->filterCb(element->conf->userData, object)) continue;
		} else {
			if (object_part && object_part[o] != element->part) continue;
		}

		mesh_drawObject(this, element, &view_model, o);
	}
}
