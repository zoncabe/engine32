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

/* The part matrices of one frame buffer: the mesh's own matrix with each
   part's offset carried through its rotation, so a displaced part follows the
   entity the way its geometry would have if it had been modelled in place. */
static void mesh_updatePartMatrices(Mesh *mesh, const Transform *matrix, uint8_t fb_index)
{
	if (mesh->part_matrix == NULL) return;

	Transform *out = mesh->part_matrix + fb_index * mesh->dl_count;

	/* Part 0 is everything the prefab did not name: it has no offset of its
	   own and is drawn with the mesh's matrix. */
	out[0] = *matrix;

	for (uint8_t i = 1; i < mesh->dl_count; i++) {
		out[i] = *matrix;
		const Vector3 &offset = mesh->part_offset[i - 1];
		if (offset.x.raw() == 0 && offset.y.raw() == 0 && offset.z.raw() == 0) continue;
		out[i].position += matrix->rotation.transform(offset);
	}
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

	mesh_updatePartMatrices(this, matrix, fb_index);
	mesh_updateBounds(this, matrix);
}


void Mesh::setMatrix(const RenderTransform *transform, uint8_t fb_index)
{
	Transform *matrix = &matrix_buffer[fb_index];

	Matrix3 r = Matrix3::fromEuler(transform->rotation.x, transform->rotation.y, transform->rotation.z);
	*matrix = transform_fromSrt(&transform->scale, &r, &transform->position);

	mesh_updatePartMatrices(this, matrix, fb_index);
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

const RenderPosition *Mesh::positions(uint32_t object) const
{
	if (deform) return deform->positions(object);
	return model->objects[object].positions;
}

const RenderShade *Mesh::shades(uint32_t object) const
{
	if (deform) return deform->shades(object);
	return model->objects[object].shades;
}


void Mesh::setPartOffsets(const Vector3 *offsets, uint8_t count)
{
	psyqo::Kernel::assert(dl_count == (uint8_t)(count + 1), "mesh: part offsets do not match the parts recorded");

	part_offset = offsets;
	part_matrix = (Transform *)psyqo_malloc(sizeof(Transform) * FB_COUNT * dl_count);
	psyqo::Kernel::assert(part_matrix != NULL, "mesh: out of memory");

	/* Placed before the first draw writes a matrix, so they start as the
	   identity the mesh itself starts with. */
	for (int fb = 0; fb < FB_COUNT; fb++)
		for (uint8_t i = 0; i < dl_count; i++)
			part_matrix[fb * dl_count + i] = Transform::identity();
}


void Mesh::recordObjects()
{
	object_part = NULL;
	dl_count    = 0;
	dl_buffers  = 1;
	palette     = NULL;
	part_name   = NULL;
	part_count  = 0;
	part_offset = NULL;
	part_matrix = NULL;
	visible     = 1;
}


/* The part a name stands for, the way the prefab wrote it. */
uint8_t Mesh::findPart(const char *name) const
{
	for (uint8_t i = 0; i < part_count; i++)
		if (part_name[i] && same(part_name[i], name)) return (uint8_t)(1 + i);
	return 0;
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

	dl_count    = 1 + count;
	part_name   = names;
	part_count  = count;
	part_offset = NULL;
	part_matrix = NULL;
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

/* Where the sides are cut, one column out of the screen.

   Cut exactly on the edge, a new vertex projects to the first column past
   the screen, and the rounding of the GTE's division can bring it back one
   column short: the face is painted up to 318 and the last column of the
   screen stays background. It shows on the right edge, where the bias falls
   inward, and not on the left, where it falls out. One column of margin is
   what the rounding moves, so one column is what it takes.

   Nothing is lost by cutting off the screen: PSYQo sets the GPU's drawing
   area to the frame buffer on every flip, so the hardware discards what
   lands outside for free.

   This is only where a piece is cut. Whether a piece is dropped is a
   different question, and mesh_outcode answers it against the screen
   itself: a piece wholly off it is not drawn, and the rounding has nothing
   to do with that. */
#define MESH_GUARD  1
#define MESH_CLIP_W (SCREEN_WIDTH  / 2 + MESH_GUARD)
#define MESH_CLIP_H (SCREEN_HEIGHT / 2 + MESH_GUARD)

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


/* What the projection of a position leaves, before any corner claims it:
   the camera space point, the screen point and which sides of the view it
   is out of. The corners that share the position copy this. */
struct PosVertex
{
	int32_t       x, y, z;
	psyqo::Vertex point;
	uint8_t       out;
};

/* The answers of the first two passes, one entry per position and one per
   shade of the largest object drawn so far. */
static PosVertex *mesh_pos;
static uint32_t   mesh_pos_size;
static uint32_t  *mesh_lit;          /* per shade: the lit color, as a primitive takes it */
static uint32_t   mesh_lit_size;

/* Per position: its depth in camera space, while the levels are settled. */
static int32_t   *mesh_posz;
static uint32_t   mesh_posz_size;


/* Whether that shade has been lit yet this object. The direct path lights
   a shade the first time a face that is actually drawn asks for it, so a
   face dropped for facing away, for its depth or for being off screen
   costs no lighting at all, and a shade shared by many corners is still
   lit once. */
static uint8_t   *mesh_lit_done;

/* What a face's cutting works on, in the CPU's scratchpad: the 1 KB at
   0x1f800000 that answers a load in one cycle where main memory takes
   six. A piece being cut to the view and what it makes of it; and the
   grid of a subdivided quad, which a subdivided triangle's rows and a
   covered edge's chain take over, since they never run at the same
   time. 984 bytes of the 1024; the module's state below takes the rest. */
/* The points the cutting makes: two per side at most, a convex piece
   crossing a plane in two places, over the five sides. */
#define MESH_CLIP_POOL 10

struct MeshScratch
{
	CamVertex        poly[MESH_CLIP_POOL];
	const CamVertex *ring[2][MESH_POLY_MAX];
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

/* Per-vertex lighting while this object draws. Off, the lights are set once
   for the whole object and nothing here is read. The placement they are
   measured against rides as an argument: the scratchpad is full. */
static bool mesh_light_per_vertex MESH_SCRATCH;

/* Where the lights are measured from while this element draws. Plain memory:
   the scratchpad is full to the byte, and this is read once per object. */
static const Transform *mesh_light_placement;

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
uint32_t mesh_profile_pieces;
uint32_t mesh_profile_cuts;
uint32_t mesh_profile_walked;
uint32_t mesh_profile_clip;
uint32_t mesh_profile_prim;
uint32_t mesh_profile_direct;
uint32_t mesh_profile_cut;
uint32_t mesh_profile_lerp;
uint32_t mesh_profile_project;
uint32_t mesh_profile_level;
uint32_t mesh_profile_object;
uint32_t mesh_profile_draw;
uint32_t mesh_profile_prep;
uint32_t mesh_profile_cut2;
/* The range of buckets one element wrote into, to see whether two meshes
   that fight are landing in the same place in the table or in different
   ones. Reset per element by Mesh::draw. */
uint32_t mesh_otz_min;
uint32_t mesh_otz_max;

uint32_t mesh_profile_slow;
uint32_t mesh_profile_back;
uint32_t mesh_profile_far;
uint32_t mesh_profile_whole;
uint32_t mesh_profile_sub;
uint32_t mesh_profile_cover;
uint32_t mesh_profile_light;
uint32_t mesh_profile_elements;

/* Adds the ticks of root counter 2 spent in a scope to a total. */
struct MeshProfile
{
	uint16_t  began;
	uint32_t *total;
	__attribute__((always_inline)) MeshProfile(uint32_t *t) : began(COUNTERS[2].value), total(t) {}
	__attribute__((always_inline)) ~MeshProfile() { *total += (COUNTERS[2].value - began) & 0xFFFF; }
};

/* Timing the emission is asked for by name, not left on. It is the one
   scope that opens per primitive, and it costs there: two reads of the
   counter, which lives in uncached I/O space, and a stack frame to hold
   the scope in a function that would otherwise need none. */
#ifdef E32_MESH_PROFILE
#define MESH_PROFILE_EMIT MeshProfile profile(&mesh_profile_emit)
#else
#define MESH_PROFILE_EMIT ((void)0)
#endif


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
static inline uint8_t mesh_outcodeOf(int32_t x, int32_t y, int32_t z)
{
	int32_t xh = x * mesh_projection, yh = y * mesh_projection;
	int32_t gx = (SCREEN_WIDTH / 2) * z, gy = (SCREEN_HEIGHT / 2) * z;

	uint8_t out = 0;
	if (z < mesh_near) out |= MESH_OUT_NEAR;
	if (xh >  gx) out |= MESH_OUT_RIGHT;
	if (xh < -gx) out |= MESH_OUT_LEFT;
	if (yh >  gy) out |= MESH_OUT_BOTTOM;
	if (yh < -gy) out |= MESH_OUT_TOP;
	return out;
}

MESH_HOT
static inline void mesh_outcode(CamVertex *c)
{
	c->out = mesh_outcodeOf(c->x, c->y, c->z);
}

/* The same sides, read off what the GTE already produced: the screen point
   it projected and the depth it kept. The projection divided by the depth
   already, so the comparisons are against the screen's own edges and
   nothing is multiplied.

   It says the same as mesh_outcodeOf about the same point: a corner is out
   on the right when x * H > (W/2) * z, which is the screen x past W once
   the center is added, and the GTE adds the center. The screen point
   saturates at 1024 pixels, which is past the edge either way, so a corner
   far out still reads as out. */
MESH_HOT
static inline uint8_t mesh_outcodeScreen(uint32_t packed, int32_t z)
{
	int32_t sx = (int16_t)(packed & 0xFFFF);
	int32_t sy = (int16_t)(packed >> 16);

	uint8_t out = 0;
	if (z < mesh_near) out |= MESH_OUT_NEAR;
	if (sx > SCREEN_WIDTH)  out |= MESH_OUT_RIGHT;
	if (sx < 0)             out |= MESH_OUT_LEFT;
	if (sy > SCREEN_HEIGHT) out |= MESH_OUT_BOTTOM;
	if (sy < 0)             out |= MESH_OUT_TOP;
	return out;
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
	/* Tried and dropped: writing the five packed words of the rotation as
	   constants, to skip building a Matrix33 on the stack. Six hundred ticks
	   worse; the compiler already folds the stack copy away. */
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

/* Vectors 0, 1 and 2 at once, for the triple commands: three points go in
   and RTPT projects them in 23 cycles where three RTPS take 45. The two
   idle cycles are the load delay before the command reads what went in. */
static inline void mesh_gteVector3(const RenderPosition *a, const RenderPosition *b,
                                   const RenderPosition *c)
{
#ifndef PS1_PC_PORT
	asm volatile("mtc2 %0, $0\n\tmtc2 %1, $1\n\t"
	             "mtc2 %2, $2\n\tmtc2 %3, $3\n\t"
	             "mtc2 %4, $4\n\tmtc2 %5, $5\n\tnop\n\tnop"
	             : : "r"(*(const uint32_t *)&a->x), "r"((uint32_t)(uint16_t)a->z),
	                 "r"(*(const uint32_t *)&b->x), "r"((uint32_t)(uint16_t)b->z),
	                 "r"(*(const uint32_t *)&c->x), "r"((uint32_t)(uint16_t)c->z));
#endif
}

/* What the corners of an object's faces share: how their color is made
   and whether it fades. Held in registers across the face loop. */
/* What the corners of an object's faces share was a struct passed by
   address, which put it on the stack and had the loop read it back out of
   main memory on every vertex. It rides as scalars instead: the compiler
   keeps those in registers for the whole object, and the PSX has no data
   cache to make a stack read cheap. */

/* One model vertex through the GTE, with the element's matrix loaded:
   to camera space and to the screen, and lit. */
/* One model vertex through the GTE, with the element's matrix loaded: to
   camera space and to the screen, and lit.

   Tried and dropped: holding the last normal and vertex color with their
   lit answer, to skip the lighting command for the corners of a flat
   shaded face, which arrive in a row carrying the same normal. Measured
   two thousand ticks worse, with the state in a struct and again with it
   in plain locals. The command is seventeen cycles and the three
   comparisons, the branch and the register pressure cost more. Folding
   this function into its caller's loop to hold that state cost a further
   thousand, so it stays a function of its own. */
static inline void mesh_position(const RenderPosition *p, PosVertex *c)
{
	/* The projection leaves the camera space point in the accumulators,
	   the screen point in the FIFO and the depth factor for the fog. */
	mesh_gteVector(*(const uint32_t *)&p->x, (uint16_t)p->z);
	Kernels::rtps();
	c->x = (int32_t)readRaw<Register::MAC1>();
	c->y = (int32_t)readRaw<Register::MAC2>();
	c->z = (int32_t)readRaw<Register::MAC3>();
	c->point.packed = readRaw<Register::SXY2>();
	c->out = mesh_outcodeOf(c->x, c->y, c->z);
}

/* The color a shade ends up with: the vertex color that came with the
   normal, folded into the material, then lit by the GTE.

   The vertex's color is one aligned word, and a read of main memory costs
   six cycles where the arithmetic on what came back costs one. A textured
   face's color modulates the texture, with 128 as one, so every channel is
   halved: shifting the three at once carries the low bit of each into the
   one below, and the mask clears it again. A flat face carries the
   material color itself, channel by channel. */
static inline uint32_t mesh_shadeColor(const RenderShade *s, bool textured,
                                       bool fog, uint32_t mat_rgb)
{
	uint32_t rgba = *(const uint32_t *)&s->r;

	uint32_t color;
	if (textured) {
		color = (rgba >> 1) & 0x7F7F7F;
	} else {
		color = ((( rgba        & 0xFF) * ( mat_rgb        & 0xFF)) >> 8)
		      | ((((rgba >>  8) & 0xFF) * ((mat_rgb >>  8) & 0xFF)) & 0xFF00)
		      | ((((rgba >> 16) & 0xFF) * ((mat_rgb >> 16) & 0xFF)) << 8 & 0xFF0000);
	}

	mesh_gteVector(*(const uint32_t *)&s->nx, (uint16_t)s->nz);
	mesh_gteColor(color);
	if (fog) Kernels::ncds();
	else     Kernels::nccs();
	return readRaw<Register::RGB2>();
}

/* One corner from the two answers, for the paths that cannot separate them:
   a skinned object, whose projection depends on the corner's bone, and
   per-vertex lighting, whose shading depends on where the corner stands. */
static inline void mesh_vertex(const RenderVertex *v, const RenderPosition *positions,
                               const RenderShade *shades, bool textured, bool fog,
                               uint32_t mat_rgb, CamVertex *c)
{
	PosVertex p;
	mesh_position(&positions[v->position], &p);
	c->x = p.x; c->y = p.y; c->z = p.z;
	c->point = p.point;
	c->out = p.out;

	c->color = mesh_shadeColor(&shades[v->shade], textured, fog, mat_rgb);
	c->u = v->u;
	c->v = v->v;
}

/* First pass: every vertex of the object into the scratch. The element's
   matrix is loaded here; a skinned object reloads it per bone.

   Three passes where there used to be one. The GTE's projection of a point
   answers the position alone, and its lighting answers the normal and the
   color that modulates it alone, because the light matrices are written
   once per element. So each table is walked once, and the corners then copy
   from both: the scene of this example holds 1188 corners over 482
   positions and 99 shades, and the two commands are fifteen and seventeen
   cycles each.

   Two paths cannot do that and stay the same. A skinned object projects
   each corner with its bone's matrix, so the position alone does not decide
   the answer; per-vertex lighting rebuilds the light matrix from where the
   corner stands, so the normal alone does not either. Both walk the corners
   one by one, as before. */
MESH_COLD
static void mesh_transform(const Element3D *element, const Object *object,
                           const RenderVertex *verts, const RenderPosition *positions,
                           const RenderShade *shades, const Transform *view_model,
                           const Transform *placement)
{
	const uint8_t *bones = element->palette ? object->boneIndices : NULL;
	int last_bone = -1;

	/* Read once for the whole object, and packed so the untextured path
	   takes its three channels apart with shifts instead of reading the
	   material again. */
	const bool textured = mesh_material->textured;
	const bool fog      = mesh_fog;
	const uint32_t mat_rgb = (uint32_t)mesh_material->color.r
	                       | ((uint32_t)mesh_material->color.g << 8)
	                       | ((uint32_t)mesh_material->color.b << 16);

	/* Fog is the third thing the tables cannot separate. NCDS fades the lit
	   color towards the fog's by the depth factor the GTE left in IR0 when
	   it projected that very vertex, so the answer belongs to the corner and
	   not to the normal: two corners of one normal at different distances
	   fade differently. With fog on, the corners go one by one and each is
	   lit right after it is projected, which is what puts its own factor in
	   IR0. */
	if (!bones && !mesh_light_per_vertex && !fog) {
		mesh_loadMatrix(view_model);

		for (uint32_t i = 0; i < object->positionCount; i++)
			mesh_position(&positions[i], &mesh_pos[i]);

		for (uint32_t i = 0; i < object->shadeCount; i++)
			mesh_lit[i] = mesh_shadeColor(&shades[i], textured, fog, mat_rgb);

		for (uint32_t i = 0; i < object->vertexCount; i++) {
			const PosVertex *p = &mesh_pos[verts[i].position];
			CamVertex *c = &mesh_cam[i];

			c->x = p->x; c->y = p->y; c->z = p->z;
			c->point = p->point;
			c->out   = p->out;
			c->color = mesh_lit[verts[i].shade];
			c->u = verts[i].u;
			c->v = verts[i].v;
		}
		return;
	}

	if (!bones) mesh_loadMatrix(view_model);

	for (uint32_t i = 0; i < object->vertexCount; i++) {
		if (bones && bones[i] != last_bone) {
			last_bone = bones[i];
			mesh_loadMatrix(&element->palette[last_bone]);
		}

		/* Per-vertex lighting: every point is measured from where it stands,
		   so the light matrix is rebuilt before the GTE shades it. The
		   vertex's units back to metres, which is the space the lights are
		   declared in. */
		if (mesh_light_per_vertex) {
			const RenderPosition *p = &positions[verts[i].position];
			Vector3 local = { fromUnits(p->x), fromUnits(p->y), fromUnits(p->z) };
			Lighting::setAt(Lighting::get(), placement, local);
		}

		mesh_vertex(&verts[i], positions, shades, textured, fog, mat_rgb, &mesh_cam[i]);
	}

	/* The last vertex left its own lights in the GTE. They are set once per
	   element, not per object, so anything drawn after this would inherit
	   them: put the object's own back. */
	if (mesh_light_per_vertex) Lighting::set(Lighting::get(), placement);
}


/* The point at 't' (0..4096) between two camera space vertices. Out of the
   hot group: its three callers, the cutting, the triangle grid and the edge
   covering, are all out of it, and the quad grid reaches its points by
   halving instead. */
MESH_COLD
static void mesh_lerp(const CamVertex *a, const CamVertex *b, int32_t t, CamVertex *out)
{
#ifdef E32_MESH_PROFILE
	MeshProfile lp(&mesh_profile_lerp);
#endif
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

#ifdef E32_MESH_PROFILE
	/* The scope opens before the matrix is swapped on purpose: the swap is
	   part of what projecting a cut point costs, and counted after it, it
	   was invisible. */
	MeshProfile prj(&mesh_profile_project);
#endif

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
/* A material with no texture is drawn as the colour alone. Its alpha still
   asks for the blend the same way a textured one does: the flag is on the
   primitive, not on the texture, and without it a glass declared half
   transparent comes out solid. */
static void mesh_emitFlat(const CamVertex *a, const CamVertex *b,
                          const CamVertex *c, const CamVertex *d, uint32_t otz)
{
	Render &render = *mesh_render;
	const bool semi = mesh_material->color.user < 255;

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
		if (semi) q.setSemiTrans();

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
		if (semi) t.setSemiTrans();

		mesh_table->insert(*f, otz);
	}
}

/* The glow pass of a face already built: the same polygon once more with
   the emission map over it, semi transparent. It goes into the bucket
   before the base, because within a bucket the last fragment in is the
   first the GPU draws and the glow has to land over it.

   Out of the hot group, and out of line: it rebuilds a whole primitive,
   which is half the code of an emitter, and only a material that declares
   an emission map ever reaches it. */
MESH_COLD
static void mesh_emitGlowTri(const TexturedTriFragment *f, const CamVertex *a,
                             const CamVertex *b, const CamVertex *c, uint32_t otz)
{
	const Material *mat = mesh_material;

	TexturedTriFragment *g = mesh_render->alloc<TexturedTriFragment>();
	if (!g) return;

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

MESH_COLD
static void mesh_emitGlowQuad(const TexturedQuadFragment *f, const CamVertex *a,
                              const CamVertex *b, const CamVertex *c,
                              const CamVertex *d, uint32_t otz)
{
	const Material *mat = mesh_material;

	TexturedQuadFragment *g = mesh_render->alloc<TexturedQuadFragment>();
	if (!g) return;

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


/* One face into the table, a triangle or a quad. The GPU takes a quad as
   the triangles ABC and BCD, so the contour's third and fourth corners
   swap on the way in; a triangle goes as it comes.

   One function for both, because everything around the primitive is the
   same for the two and was written twice: the depth bucket, the material's
   dispatch, the texture scroll, the glow pass and the insertion. What
   differs is only the writes into the primitive, and only because PSYQo's
   quad and triangle are separate types.

   A quad's corners are three words each, laid out the same as a triangle's
   with one more block at the end, so the two bodies below are the same
   writes with one corner more. */
MESH_HOT
static void mesh_emit(const CamVertex *const *v, int n)
{
	MESH_PROFILE_EMIT;
	const Material *mat = mesh_material;

	const CamVertex *a = v[0], *b = v[1], *c = v[2];
	const CamVertex *d = n == 4 ? v[3] : NULL;

	/* The farthest corner, in the models' units. The far plane is judged on
	   it, and the table's bucket comes out of it.

	   Not the average, and not the nearest corner either. A surface seen at
	   a slant, a floor above all, reaches from under the camera to the back
	   of the room: by its nearest corner it lands in front of everything
	   standing on it, and by its average it still lands in front of whatever
	   stands on its near half. By its farthest corner it goes to the back,
	   where a floor belongs, and what stands on it is painted over it.

	   It is also how the working ports do it. */
	int32_t depth = a->z;
	if (b->z > depth) depth = b->z;
	if (c->z > depth) depth = c->z;
	if (d && d->z > depth) depth = d->z;
	if (depth <= 0 || depth > (int32_t)mesh_far) return;

	uint32_t otz = (uint32_t)depth >> RENDER_OT_SHIFT;
	if (otz == 0 || otz >= RENDER_OT_SIZE) return;
	if (otz < mesh_otz_min) mesh_otz_min = otz;
	if (otz > mesh_otz_max) mesh_otz_max = otz;

	if (mesh_wireframe) { mesh_emitWire(a, b, c, d, otz); return; }
	if (!mat->textured) { mesh_emitFlat(a, b, c, d, otz); return; }

	uint8_t su = mat->slot.u + mesh_scroll_u;
	uint8_t sv = mat->slot.v + mesh_scroll_v;
	const bool semi = mat->color.user < 255;

	if (d) {
		TexturedQuadFragment *f = mesh_render->alloc<TexturedQuadFragment>();
		f->primitive.window = mat->slot.window;

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
		if (semi) q.setSemiTrans();

		if (mat->glow) mesh_emitGlowQuad(f, a, b, c, d, otz);

		mesh_table->insert(*f, otz);
	} else {
		TexturedTriFragment *f = mesh_render->alloc<TexturedTriFragment>();
		f->primitive.window = mat->slot.window;

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
		if (semi) t.setSemiTrans();

		if (mat->glow) mesh_emitGlowTri(f, a, b, c, otz);

		mesh_table->insert(*f, otz);
	}
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
/* The five sides, each written out. This was one formula over a table of
   coefficients, and every call read three of them and multiplied by all
   three, when four of the five sides have two coefficients that are zero
   and one that is plus or minus one. It runs twice per edge for every side
   a piece is cut on, up to thirty times for one triangle. */

MESH_HOT
static int32_t mesh_inside(const CamVertex *c, int side)
{
	switch (side) {
	case 0:  return c->z - mesh_near;
	case 1:  return MESH_CLIP_W * c->z + c->x * mesh_projection;
	case 2:  return MESH_CLIP_W * c->z - c->x * mesh_projection;
	case 3:  return MESH_CLIP_H * c->z + c->y * mesh_projection;
	default: return MESH_CLIP_H * c->z - c->y * mesh_projection;
	}
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
	/* The walk carries pointers, not vertices. A corner that survives a side
	   used to be copied whole, twenty four bytes, once for every side the
	   piece is cut on; now only the points where an edge crosses a side are
	   written, into a pool, and everything else is a pointer moved.

	   A point of the pool is told from one of the caller's by its address,
	   which is what marks the piece's own corners for the fan. */
	CamVertex *pool = mesh_scratch.poly;
	const CamVertex *pool_end = pool + MESH_CLIP_POOL;
	int pool_n = 0;

	const CamVertex **in = mesh_scratch.ring[0], **out = mesh_scratch.ring[1];
	for (int i = 0; i < n; i++) in[i] = v[i];

	for (int side = 0; side < 5; side++) {
		if (!(sides & (1 << side))) continue;

		/* How far inside this side each corner is, once per corner. The
		   walk used to ask it for both ends of every edge, and the far end
		   of one edge is the near end of the next, so every corner but the
		   first was measured twice. */
		int32_t dist[MESH_POLY_MAX];
		for (int i = 0; i < n; i++) dist[i] = mesh_inside(in[i], side);

		int count = 0;
		for (int i = 0; i < n; i++) {
			int j = i + 1 == n ? 0 : i + 1;
			const CamVertex *a = in[i], *b = in[j];
			int32_t da = dist[i], db = dist[j];

			if (da >= 0) out[count++] = a;
			if ((da >= 0) != (db >= 0)) {
				if (pool_n == MESH_CLIP_POOL) return 0;

				/* the crossing walked from the same end whichever piece
				   the edge belongs to, so both land on the same point and
				   leave no seam */
				CamVertex *cut = &pool[pool_n++];
				bool a_first = a->x != b->x ? a->x < b->x
				             : a->y != b->y ? a->y < b->y : a->z < b->z;
				if (a_first) mesh_lerp(a, b, mesh_crossing(da, db), cut);
				else         mesh_lerp(b, a, mesh_crossing(db, da), cut);
				/* on the near plane, not a rounding short of it: nearer,
				   it would count as behind and go unprojected */
				if (cut->z < mesh_near) cut->z = mesh_near;
				cut->out = MESH_OUT_NEW;
				out[count++] = cut;
			}
		}
		n = count;
		if (n < 3) return 0;

		/* The vertex cut on the near plane can land off any edge of the
		   screen, whatever the corners it came from were out of: what it
		   is out of joins the sides to cut. */
		if (side == 0) {
			for (int i = 0; i < n; i++) {
				if (out[i] < pool || out[i] >= pool_end) continue;
				CamVertex *c = (CamVertex *)out[i];
				mesh_outcode(c);
				sides |= c->out;
				c->out = MESH_OUT_NEW;
			}
		}

		const CamVertex **swap = in; in = out; out = swap;
	}

	for (int i = 0; i < n; i++) {
		if (in[i] >= pool && in[i] < pool_end) {
			CamVertex *c = (CamVertex *)in[i];
			mesh_project(c);
			c->out = MESH_OUT_NEW;   /* on the side by construction; the mark stays */
		}
		result[i] = in[i];
	}
	return n;
}

/* Whether this came out of the cutting rather than from the piece itself:
   the fan hangs from a corner of the piece, and a point on a side is not
   one. Told by the address, the cut points living in the pool. */
static inline bool mesh_isCut(const CamVertex *c)
{
	const CamVertex *pool = mesh_scratch.poly;
	return c >= pool && c < pool + MESH_CLIP_POOL;
}

/* A triangle cut to the view into the table. Three or four vertices go
   as they are, a triangle or a quad. More are a fan of triangles from
   one of the triangle's own corners, never from a vertex cut on a side,
   so every inner edge runs from a real corner to a point on a real
   edge. */
MESH_HOT
static void mesh_emitPoly(const CamVertex *const *p, int m)
{
	if (m <= 4) {
		mesh_emit(p, m);
		return;
	}

	int apex = 0;
	for (int i = 0; i < m; i++)
		if (!mesh_isCut(p[i])) { apex = i; break; }

	/* Walking the ring by subtracting once instead of by remainder: the
	   apex plus the step never reaches twice the count, and the R3000A's
	   divide is thirty five cycles where a compare is one. */
	int j = apex + 1;
	if (j >= m) j -= m;

	/* Two of the fan's triangles at a time, as one quad: the GPU splits a
	   quad ABCD into ABC and BCD, which are the two the fan would have sent
	   on its own, so the same pixels are painted by half the primitives. A
	   cut polygon is convex by construction, so the quad is well formed.
	   The last triangle goes alone when the count is odd. */
	int left = m - 2;   /* triangles the fan owes */
	while (left > 0) {
		int k = j + 1;
		if (k >= m) k -= m;

		if (left >= 2) {
			int l = k + 1;
			if (l >= m) l -= m;

			const CamVertex *quad[4] = { p[apex], p[j], p[k], p[l] };
			mesh_emit(quad, 4);

			j = l;
			left -= 2;
		} else {
			const CamVertex *tri[3] = { p[apex], p[j], p[k] };
			mesh_emit(tri, 3);

			j = k;
			left--;
		}
	}
}

/* A triangle cut to the view: dropped when nothing is left, or when it
   looks away and the face was not judged whole.

   Out of the hot group, and it takes mesh_clip, mesh_inside, mesh_crossing
   and mesh_emitPoly with it, all inlined here: only a face crossing an edge
   of the view reaches any of them, and together they are a quarter of the
   group, which the 4 KB instruction cache does not have to spare. */
MESH_COLD
static void mesh_cutTri(const CamVertex *const *v, int n, uint8_t sides)
{
#ifdef E32_MESH_PROFILE
	mesh_profile_cuts++;
	MeshProfile profile(&mesh_profile_clip);
#endif
	const CamVertex *poly[MESH_POLY_MAX];
	int m;
	{
#ifdef E32_MESH_PROFILE
		MeshProfile inner(&mesh_profile_cut);
#endif
		m = mesh_clip(v, n, sides, poly);
	}
#ifdef E32_MESH_CUT_TWICE
	{
		MeshProfile second(&mesh_profile_cut2);
		m = mesh_clip(v, n, sides, poly);
	}
#endif
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
#ifdef E32_MESH_PROFILE
	mesh_profile_pieces++;
#endif
	uint8_t any = 0, all = 0xFF;
	for (int k = 0; k < n; k++) {
		any |= v[k]->out;
		all &= v[k]->out;
	}
	if (all) return;

	if (!any) {
		if (!mesh_face_tested && mesh_facesAway(v, n)) return;
		mesh_emit(v, n);
		return;
	}

	/* Tried and dropped: leaving whole a piece that hangs off an edge of the
	   screen, since the GPU's drawing area is the frame buffer and it throws
	   away the pixels outside it. Thirty five horizontal blanks cheaper and
	   wrong on the console: the texture is interpolated across the whole
	   polygon without perspective correction, so a piece that reaches past
	   the edge spreads its texels over what is never drawn and the part that
	   is drawn comes out stretched. Cutting puts the corners on the edge and
	   the interpolation runs across what is seen. */

	/* The quad is cut as one polygon of four sides, not split into its two
	   triangles first. Splitting ran the whole Sutherland-Hodgman walk
	   twice, over three corners each time, where one walk over four corners
	   does the same work once. The GPU splits what comes out on its own
	   diagonal, as it does with any quad. */
	mesh_cutTri(v, n, any);
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
#ifdef E32_MESH_PROFILE
	MeshProfile profile(&mesh_profile_prim);
#endif
	/* Winding, from the screen points when every corner is on screen; a
	   face with a corner off it is judged piece by piece, once cut. */
	uint8_t any = 0;
	for (int k = 0; k < n; k++) any |= v[k]->out;
	mesh_face_tested = any == 0;
	if (mesh_face_tested && mesh_facesAway(v, n)) return;

	{
#ifdef E32_MESH_PROFILE
		MeshProfile sub(own == 1 ? &mesh_profile_whole : &mesh_profile_sub);
#endif
		if (own == 1)     mesh_piece(v, n);
		else if (n == 3)  mesh_subdivideTri(v, own);
		else              mesh_subdivideQuad(v, own);
	}

	/* the edges a finer neighbour puts more points on */
	if (cover) {
#ifdef E32_MESH_PROFILE
		MeshProfile cov(&mesh_profile_cover);
#endif
		for (int k = 0; k < n; k++)
			if (cover[k]) mesh_cover(v, n, k, own, cover[k]);
	}
}


/* What the direct path needs of the object, held across its loop. */
struct MeshDirect
{
	const RenderVertex   *verts;
	const RenderPosition *positions;
	const RenderShade    *shades;
	const Transform      *view_model;
	const Material       *mat;
	uint32_t mat_rgb;
	uint8_t  su, sv;
	bool     textured, fog, semi;
	bool     subdivide;
	const uint16_t *adjacent;   /* the neighbour across each edge, when subdividing */
	uint32_t face_base;         /* where this object's faces start in the model's numbering */
};

/* Every face's level of subdivision, over the whole model, before any of it
   is drawn.

   It has to be the whole model and not one object: a face reads its
   neighbours' level to know which of its edges need covering, and an object
   draws with one material, so a continuous surface arrives split and the
   neighbour across the seam belongs to another object, drawn later or
   already done.

   The level answers the depth of the face's nearest corner, and nothing
   else, so the depth is all this needs. It comes off the CPU rather than
   the GTE: the third row of the view times the point. Three multiplies per
   position, which is what a pass over the vertices would have cost before
   projecting any of them.

   The translation goes in before the shift, where the GTE puts it, MAC3
   being (TRZ*4096 + R31*x + R32*y + R33*z) >> 12. Added after it rounds
   differently, and the level a face lands on decides whether its edge
   matches its neighbour's. */
MESH_COLD
static void mesh_levelPass(const Model *model, const Transform *view_model)
{
	const Matrix3 &r = view_model->rotation;
	int32_t m20 = r.ex.z.raw(), m21 = r.ey.z.raw(), m22 = r.ez.z.raw();
	int32_t tz12 = (view_model->position.z.raw() >> MODEL_CPU_TO_UNITS) << 12;

	if (model->faceCount > mesh_pieces_size) {
		if (mesh_pieces) psyqo_free(mesh_pieces);
		mesh_pieces = (uint8_t *)psyqo_malloc(model->faceCount);
		psyqo::Kernel::assert(mesh_pieces != NULL, "mesh: out of memory");
		mesh_pieces_size = model->faceCount;
	}

	for (int l = 0; l < mesh_lod_count; l++)
		mesh_lod_range[l] = mesh_lod[l].range.raw() >> MODEL_CPU_TO_UNITS;

	for (int o = 0; o < model->objectCount; o++) {
		const Object *obj = &model->objects[o];

		if (obj->positionCount > mesh_posz_size) {
			if (mesh_posz) psyqo_free(mesh_posz);
			mesh_posz = (int32_t *)psyqo_malloc(sizeof(int32_t) * obj->positionCount);
			psyqo::Kernel::assert(mesh_posz != NULL, "mesh: out of memory");
			mesh_posz_size = obj->positionCount;
		}

		for (uint32_t i = 0; i < obj->positionCount; i++) {
			const RenderPosition *p = &obj->positions[i];
			mesh_posz[i] = (tz12 + (int32_t)p->x * m20 + (int32_t)p->y * m21
			              + (int32_t)p->z * m22) >> 12;
		}

		uint8_t *pieces = mesh_pieces + obj->faceBase;
		for (uint32_t i = 0; i < obj->faceCount; i++) {
			const Face *face = &obj->faces[i];
			int n = face->v[3] == MODEL_NO_VERTEX ? 3 : 4;
			int32_t minz = 0x7FFFFFFF;
			for (int k = 0; k < n; k++) {
				int32_t z = mesh_posz[obj->vertices[face->v[k]].position];
				if (z < minz) minz = z;
			}

			uint8_t p = 1;
			for (int l = 0; l < mesh_lod_count; l++)
				if (minz <= mesh_lod_range[l]) { p = mesh_lod[l].pieces; break; }
			pieces[i] = p;
		}
	}
}


/* The color of one shade, lit the first time it is asked for. */
MESH_HOT
static inline uint32_t mesh_litColor(const MeshDirect *d, uint16_t shade)
{
	if (mesh_lit_done[shade]) return mesh_lit[shade];

	uint32_t color = mesh_shadeColor(&d->shades[shade], d->textured, d->fog, d->mat_rgb);
	mesh_lit[shade] = color;
	mesh_lit_done[shade] = 1;
	return color;
}

/* A face the direct path cannot finish: it crosses the view. Its corners
   are brought back into camera space, one RTPS each, and it goes down the
   path that cuts.

   The cutting projects its new points through the identity, so it takes the
   GTE's matrix from under the loop; the caller puts the element's back when
   it sees the flag moved. */
MESH_COLD
static void mesh_faceSlow(const MeshDirect *d, const Face *face, int n,
                          int own, const int *cover)
{
	CamVertex corner[4];
	const CamVertex *p[4];

	int32_t minz = 0x7FFFFFFF, maxz = -0x7FFFFFFF;
	uint8_t all = 0xFF;

	for (int k = 0; k < n; k++) {
		const RenderVertex *v = &d->verts[face->v[k]];
		PosVertex q;
		mesh_position(&d->positions[v->position], &q);

		corner[k].x = q.x; corner[k].y = q.y; corner[k].z = q.z;
		corner[k].point = q.point;
		corner[k].out   = q.out;
		corner[k].u = v->u;
		corner[k].v = v->v;
		p[k] = &corner[k];

		if (q.z < minz) minz = q.z;
		if (q.z > maxz) maxz = q.z;
		all &= q.out;
	}

	/* The same three the old face loop threw away before doing any work:
	   the face wholly out of one side of the view, past the far plane, or
	   too deep for the GTE's sixteen bits. Leaving them out was what made
	   the first attempts at bringing a subdivided mesh through here cost
	   more than the pass it replaced: every face went to the cutting. */
	if (all || minz > (int32_t)mesh_far || maxz > 32767) return;

	/* Lit only now, once the face is known to be drawn. */
	for (int k = 0; k < n; k++)
		corner[k].color = mesh_litColor(d, d->verts[face->v[k]].shade);

	mesh_prim(p, n, own, cover);
}

/* Every face of one object into the table, the way the console's own
   library does it: the face's three points go into the GTE together and
   come out projected, the coprocessor says whether the face looks away and
   how deep it is, and what it leaves is written straight into the GPU
   primitive. Nothing is kept per vertex, so no vertex is read back out of
   main memory, which has no cache in front of it.

   The order is what makes it cheap. RTPT projects three points in 23
   cycles. NCLIP gives the face's signed area on screen in 8, from the
   points already in the coprocessor, and a face that looks away is dropped
   before anything else happens to it. AVSZ3 averages the three depths in 5
   and that average is the table's bucket, so a face past the far plane is
   dropped next. Only what survives both is projected in its fourth corner,
   tested against the screen, lit, and written out.

   A face that crosses an edge of the view or belongs to a subdivided mesh
   cannot be finished here: it goes to mesh_faceSlow, which puts its corners
   back into camera space and hands them to the cutting. */
MESH_HOT MESH_COLD
static void mesh_drawFacesDirect(const Object *object, const MeshDirect *d, int32_t far)
{
#ifdef E32_MESH_PROFILE
	mesh_profile_walked += object->faceCount;
	MeshProfile whole(&mesh_profile_direct);
#endif
	for (uint32_t i = 0; i < object->faceCount; i++) {
		const Face *face = &object->faces[i];
		int n = face->v[3] == MODEL_NO_VERTEX ? 3 : 4;

		/* Its own pieces, and per edge the neighbour's when finer. Settled
		   before anything is projected: a face that comes out in pieces, or
		   that covers an edge where a finer neighbour put more points, goes
		   to the path that subdivides, and that path projects the corners
		   itself. Deciding it after the projection projected them twice. */
		if (d->subdivide) {
			/* The levels are numbered across the model, so the neighbour
			   the array names may belong to another object. */
			int own = mesh_pieces[d->face_base + i], cover[4], any_cover = 0;
			const uint16_t *across = &d->adjacent[i * 4];
			for (int k = 0; k < n; k++) {
				int nb = across[k] != MODEL_NO_FACE ? mesh_pieces[across[k]] : 1;
				cover[k] = nb > own ? nb : 0;
				any_cover |= cover[k];
			}

			if (own > 1 || any_cover) {
#ifdef E32_MESH_PROFILE
				mesh_profile_slow++;
#endif
				mesh_faceSlow(d, face, n, own, any_cover ? cover : NULL);
				if (mesh_identity) { mesh_loadMatrix(d->view_model); mesh_identity = false; }
				continue;
			}
		}

		const RenderVertex *va = &d->verts[face->v[0]];
		const RenderVertex *vb = &d->verts[face->v[1]];
		const RenderVertex *vc = &d->verts[face->v[2]];

		mesh_gteVector3(&d->positions[va->position], &d->positions[vb->position],
		                &d->positions[vc->position]);
		Kernels::rtpt();

		/* Looks away: the signed area of the screen points, positive when
		   the outside points away, the screen's y growing downward. NCLIP
		   gives it in eight cycles from the points the coprocessor already
		   holds, where the CPU would read three screen points back out of
		   memory and multiply.

		   Over three points, which for a triangle is the same sum
		   mesh_facesAway takes. A quad used to be judged over its four, so
		   a warped one could in principle go the other way; counted over
		   every quad of every frame of this scene, the two answers never
		   differed. */
		Kernels::nclip();
		int32_t area = (int32_t)readRaw<Register::MAC0>();

		int32_t za = (int32_t)readRaw<Register::SZ1>();
		int32_t zb = (int32_t)readRaw<Register::SZ2>();
		int32_t zc = (int32_t)readRaw<Register::SZ3>();

		uint32_t sa = readRaw<Register::SXY0>();
		uint32_t sb = readRaw<Register::SXY1>();
		uint32_t sc = readRaw<Register::SXY2>();

		/* Off the screen or behind the near plane: the GTE stops a screen
		   point at 1024 pixels and cannot divide behind the camera, and the
		   texture is interpolated across the polygon without perspective
		   correction, so a face that reaches past an edge is cut to the edge
		   rather than drawn whole and clipped by the GPU per pixel. */
		uint8_t out = mesh_outcodeScreen(sa, za) | mesh_outcodeScreen(sb, zb)
		            | mesh_outcodeScreen(sc, zc);

		/* Looks away, and its three judged corners are on the screen, so the
		   area NCLIP gave is the real one. Dropped here, before the bucket is
		   averaged and before a quad's fourth corner is projected: a face
		   that goes no further has no use for either.

		   NCLIP reads the three corners the RTPT left, so this is the whole
		   verdict for a triangle. For a quad whose fourth corner falls off
		   the screen it used to go to the cutting instead, which then dropped
		   every piece for looking away once each piece was on screen: the
		   same nothing drawn, by a longer road. */
		if (!out && area > 0) {
#ifdef E32_MESH_PROFILE
			mesh_profile_back++;
#endif
			continue;
		}

		/* The bucket: the farthest corner, the same rule mesh_emit follows.
		   AVSZ3 and AVSZ4 are not used any more; the depths are already
		   read. */
		int32_t  zd = za;
		uint32_t sd = 0;

		if (n == 4) {
			const RenderVertex *vd = &d->verts[face->v[3]];
			const RenderPosition *p = &d->positions[vd->position];
			mesh_gteVector(*(const uint32_t *)&p->x, (uint16_t)p->z);
			Kernels::rtps();
			sd = readRaw<Register::SXY2>();
			zd = (int32_t)readRaw<Register::SZ3>();

			out |= mesh_outcodeScreen(sd, zd);
		}

		int32_t depth = za;
		if (zb > depth) depth = zb;
		if (zc > depth) depth = zc;
		if (zd > depth) depth = zd;

		if (depth <= 0 || depth > (int32_t)mesh_far) {
#ifdef E32_MESH_PROFILE
			mesh_profile_far++;
#endif
			continue;
		}

		uint32_t otz = (uint32_t)depth >> RENDER_OT_SHIFT;
		if (otz == 0 || otz >= RENDER_OT_SIZE) {
#ifdef E32_MESH_PROFILE
			mesh_profile_far++;
#endif
			continue;
		}
		if (otz < mesh_otz_min) mesh_otz_min = otz;
		if (otz > mesh_otz_max) mesh_otz_max = otz;

		/* Crossing the view: its corners go back into camera space, one RTPS
		   each, and it takes the path that cuts. That path projects the
		   points it makes through the identity and loads it itself, so the
		   element's matrix goes back in right after.

		   Setting these aside and serving them together, with one change of
		   matrix for the lot, was measured worse: their corners then have to
		   be written out and read back, which costs more than the dozen
		   writes to the coprocessor it saves. */
		if (out || mesh_wireframe) {
#ifdef E32_MESH_PROFILE
			mesh_profile_slow++;
#endif
			mesh_faceSlow(d, face, n, 1, NULL);
			if (mesh_identity) { mesh_loadMatrix(d->view_model); mesh_identity = false; }
			continue;
		}

		/* Straight into the primitive. The GPU takes a quad as the triangles
		   ABC and BCD, so the contour's third and fourth corners swap.

		   The four bodies below sit inline although together they make this
		   function 7628 bytes against a 4 KB instruction cache. Moving them
		   out to a function of their own was measured twelve horizontal
		   blanks worse: the cache fetches sixteen bytes at a time on demand,
		   so a body that never runs never occupies it, and only one of the
		   four runs for a given object. What the split added was a call with
		   eight arguments on every face. */
		const Material *mat = d->mat;
		uint8_t su = d->su, sv = d->sv;

		{
		if (!d->textured) {
			/* The color alone, no texture and so no window in front. */
			if (n == 4) {
				const RenderVertex *vd = &d->verts[face->v[3]];

				FlatQuadFragment *f = mesh_render->alloc<FlatQuadFragment>();
				if (!f) continue;

				psyqo::Prim::GouraudQuad &q = f->primitive;
				q.setColorA({ .packed = mesh_litColor(d, va->shade) });
				q.colorB.packed = mesh_litColor(d, vb->shade);
				q.colorC.packed = mesh_litColor(d, vd->shade);
				q.colorD.packed = mesh_litColor(d, vc->shade);
				q.pointA.packed = sa;
				q.pointB.packed = sb;
				q.pointC.packed = sd;
				q.pointD.packed = sc;
				if (d->semi) q.setSemiTrans();

				mesh_table->insert(*f, otz);
			} else {
				FlatTriFragment *f = mesh_render->alloc<FlatTriFragment>();
				if (!f) continue;

				psyqo::Prim::GouraudTriangle &t = f->primitive;
				t.setColorA({ .packed = mesh_litColor(d, va->shade) });
				t.colorB.packed = mesh_litColor(d, vb->shade);
				t.colorC.packed = mesh_litColor(d, vc->shade);
				t.pointA.packed = sa;
				t.pointB.packed = sb;
				t.pointC.packed = sc;
				if (d->semi) t.setSemiTrans();

				mesh_table->insert(*f, otz);
			}
			continue;
		}

		if (n == 4) {
			const RenderVertex *vd = &d->verts[face->v[3]];

			TexturedQuadFragment *f = mesh_render->alloc<TexturedQuadFragment>();
			f->primitive.window = mat->slot.window;

			psyqo::Prim::GouraudTexturedQuad &q = f->primitive.quad;
			q.setColorA({ .packed = mesh_litColor(d, va->shade) });
			q.colorB.packed = mesh_litColor(d, vb->shade);
			q.colorC.packed = mesh_litColor(d, vd->shade);
			q.colorD.packed = mesh_litColor(d, vc->shade);
			q.pointA.packed = sa;
			q.pointB.packed = sb;
			q.pointC.packed = sd;
			q.pointD.packed = sc;
			q.uvA = { (uint8_t)(su + va->u), (uint8_t)(sv + va->v) };
			q.uvB = { (uint8_t)(su + vb->u), (uint8_t)(sv + vb->v) };
			q.uvC = { (uint8_t)(su + vd->u), (uint8_t)(sv + vd->v), 0 };
			q.uvD = { (uint8_t)(su + vc->u), (uint8_t)(sv + vc->v), 0 };
			q.tpage = mat->slot.tpage;
			if (d->semi) q.setSemiTrans();

			if (mat->glow) {
				CamVertex ga, gb, gc, gd;
				ga.point.packed = sa; ga.u = va->u; ga.v = va->v;
				gb.point.packed = sb; gb.u = vb->u; gb.v = vb->v;
				gc.point.packed = sc; gc.u = vc->u; gc.v = vc->v;
				gd.point.packed = sd; gd.u = vd->u; gd.v = vd->v;
				mesh_emitGlowQuad(f, &ga, &gb, &gc, &gd, otz);
			}

			mesh_table->insert(*f, otz);
		} else {
			TexturedTriFragment *f = mesh_render->alloc<TexturedTriFragment>();
			f->primitive.window = mat->slot.window;

			psyqo::Prim::GouraudTexturedTriangle &t = f->primitive.tri;
			t.setColorA({ .packed = mesh_litColor(d, va->shade) });
			t.colorB.packed = mesh_litColor(d, vb->shade);
			t.colorC.packed = mesh_litColor(d, vc->shade);
			t.pointA.packed = sa;
			t.pointB.packed = sb;
			t.pointC.packed = sc;
			t.uvA = { (uint8_t)(su + va->u), (uint8_t)(sv + va->v) };
			t.uvB = { (uint8_t)(su + vb->u), (uint8_t)(sv + vb->v) };
			t.uvC = { (uint8_t)(su + vc->u), (uint8_t)(sv + vc->v), 0 };
			t.tpage = mat->slot.tpage;
			if (d->semi) t.setSemiTrans();

			if (mat->glow) {
				CamVertex ga, gb, gc;
				ga.point.packed = sa; ga.u = va->u; ga.v = va->v;
				gb.point.packed = sb; gb.u = vb->u; gb.v = vb->v;
				gc.point.packed = sc; gc.u = vc->u; gc.v = vc->v;
				mesh_emitGlowTri(f, &ga, &gb, &gc, otz);
			}

			mesh_table->insert(*f, otz);
		}
		}
	}
}


/* Every face of one object into the table: the one loop that runs per face,
   and so the only part of the drawing that has to stay resident.

   Its own function because what used to sit around it in mesh_drawObject —
   the buffers, the profiling, the level pass — runs once per object and is
   over a kilobyte of code. In a 4 KB direct mapped cache that kilobyte
   lands on the same lines as what this loop calls, and evicts it on every
   face.

   Never inlined: it has one caller, and left alone the compiler folds it
   back into that caller and the section it was put in goes with it. */
MESH_HOT MESH_COLD
static void mesh_drawFaces(const Object *object, int32_t far)
{
#ifdef E32_MESH_PROFILE
	mesh_profile_walked += object->faceCount;
#endif
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

		/* its own pieces, and per edge the neighbour's when finer, the
		   levels being numbered across the model */
		int own = 1, cover[4], any_cover = 0;
		if (mesh_subdivide) {
			own = mesh_pieces[object->faceBase + i];
			const uint16_t *across = &object->adjacent[i * 4];
			for (int k = 0; k < n; k++) {
				int nb = across[k] != MODEL_NO_FACE ? mesh_pieces[across[k]] : 1;
				cover[k] = nb > own ? nb : 0;
				any_cover |= cover[k];
			}
		}

		mesh_prim(p, n, own, any_cover ? cover : NULL);
	}
}


/* One object of one element into the table: what it takes to have the
   vertices in the scratch and every face's level settled, and then the
   loop above. All of it once per object. */
MESH_COLD
static void mesh_drawObject(const Mesh *mesh, const Element3D *element,
                            const Transform *view_model, uint32_t index)
{
	uint16_t object_began = (uint16_t)COUNTERS[2].value;

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

	if (object->positionCount > mesh_pos_size) {
		if (mesh_pos) psyqo_free(mesh_pos);
		mesh_pos = (PosVertex *)psyqo_malloc(sizeof(PosVertex) * object->positionCount);
		psyqo::Kernel::assert(mesh_pos != NULL, "mesh: out of memory");
		mesh_pos_size = object->positionCount;
	}

	if (object->shadeCount > mesh_lit_size) {
		if (mesh_lit) psyqo_free(mesh_lit);
		if (mesh_lit_done) psyqo_free(mesh_lit_done);
		mesh_lit = (uint32_t *)psyqo_malloc(sizeof(uint32_t) * object->shadeCount);
		mesh_lit_done = (uint8_t *)psyqo_malloc(object->shadeCount);
		psyqo::Kernel::assert(mesh_lit != NULL && mesh_lit_done != NULL, "mesh: out of memory");
		mesh_lit_size = object->shadeCount;
	}

	mesh_profile_object += (uint16_t)((uint16_t)COUNTERS[2].value - object_began);

	/* On root counter 2, not on the horizontal blanks: the scope closes once
	   per object, and a blank is 63 us, so every object used to drop what it
	   spent below one of them. Ten objects hid most of a millisecond. */
	uint16_t faces_began = (uint16_t)COUNTERS[2].value;

	int32_t far = (int32_t)mesh_far;

	/* The direct path: no pass over the vertices at all, the faces go
	   through the coprocessor one by one and straight into the primitives.

	   It cannot serve a subdivided mesh. Bringing one through was tried
	   twice and measured 417 and then 528 horizontal blanks against 268: the
	   levels can be settled on the CPU without projecting, but every face
	   that comes out in pieces then drops to the path that subdivides, and
	   that path wants its corners in camera space, which is the pass this
	   one exists to avoid. It also cannot serve a skinned mesh, whose
	   projection depends on the corner's bone and not on the position alone;
	   per-vertex lighting, whose shading depends on where the corner stands;
	   or fog, which fades a corner by the depth factor its own projection
	   left behind and so cannot be answered once per normal. */
	if (!element->palette && !mesh_light_per_vertex && !mesh_fog) {
		MeshDirect direct;
		direct.verts      = mesh->vertices(index);
		direct.positions  = mesh->positions(index);
		direct.shades     = mesh->shades(index);
		direct.view_model = view_model;
		direct.mat        = mesh_material;
		direct.mat_rgb    = (uint32_t)mesh_material->color.r
		                  | ((uint32_t)mesh_material->color.g << 8)
		                  | ((uint32_t)mesh_material->color.b << 16);
		direct.su        = mesh_material->slot.u + mesh_scroll_u;
		direct.sv        = mesh_material->slot.v + mesh_scroll_v;
		direct.textured  = mesh_material->textured;
		direct.fog       = mesh_fog;
		direct.semi      = mesh_material->color.user < 255;
		direct.subdivide = mesh_subdivide;
		direct.adjacent  = object->adjacent;
		direct.face_base = object->faceBase;

		__builtin_memset(mesh_lit_done, 0, object->shadeCount);

		mesh_loadMatrix(view_model);
		mesh_identity = false;

		mesh_drawFacesDirect(object, &direct, far);

		mesh_profile_faces += (uint16_t)((uint16_t)COUNTERS[2].value - faces_began);
		return;
	}

	{
		MeshProfile profile(&mesh_profile_transform);
		mesh_transform(element, object, mesh->vertices(index), mesh->positions(index),
		               mesh->shades(index), view_model, mesh_light_placement);
	}

	/* From here the GTE only projects the points of cut faces, already in
	   camera space. */
	mesh_identity = false;

	/* The levels are settled for the whole model in mesh_levelPass, before
	   any object is drawn, so that a face can read the level of a neighbour
	   the importer put in another object. */

	mesh_drawFaces(object, far);

	mesh_profile_faces += (uint16_t)((uint16_t)COUNTERS[2].value - faces_began);
}


void Mesh::draw(const Element3D *element) const
{
	MeshProfile whole_draw(&mesh_profile_draw);
	uint16_t prep_began = (uint16_t)COUNTERS[2].value;

	mesh_otz_min = 0xFFFFFFFF;
	mesh_otz_max = 0;

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
	const Transform *placement = element->matrix ? element->matrix : &identity;

	/* Per vertex only where it changes anything: a scene of directional lights
	   has no point to measure from, so the flag costs nothing there. */
	mesh_light_placement  = placement;
	mesh_light_per_vertex = ((vertex_lighting >> element->part) & 1)
	                     && Lighting::hasPositional(Lighting::get());

	/* The object's own set stands for the whole of it, and stays in force for
	   every vertex when the per-vertex path is off. */
	{
		MeshProfile lgt(&mesh_profile_light);
		Lighting::set(Lighting::get(), placement);
	}
	mesh_profile_elements++;

	/* Every face's level of subdivision, over the whole model, before any
	   object is drawn: a face reads its neighbours' to know which edges to
	   cover, and the neighbour can be in another object, drawn later or
	   already done. */
	mesh_profile_prep += (uint16_t)((uint16_t)COUNTERS[2].value - prep_began);

	if (subdivide) {
		MeshProfile lvl(&mesh_profile_level);
		mesh_levelPass(model, &view_model);
	}

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
