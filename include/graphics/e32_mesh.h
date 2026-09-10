#ifndef ENGINE_32_MESH_H
#define ENGINE_32_MESH_H

#include "physics/math/e32_math.h"
#include "animation/e32_model.h"
#include "animation/e32_armature.h"

#include "render/e32_render.h"
#include "physics/math/e32_vector3.h"
#include "physics/math/e32_quaternion.h"


struct MeshDeform;

struct MeshBound
{
	Vector3 min, max;
};

/* The four primitives a mesh emits. A textured one carries its texture
   window in front: the table interleaves materials by depth, so no window
   set once would hold. */
struct TexturedTriPrim
{
	TextureWindow window;
	psyqo::Prim::GouraudTexturedTriangle tri;

	/* Not an aggregate: value-initializing one would clear it whole (a
	   memset per primitive) before the command word goes in. Only the
	   command word is built; the rest is written by whoever emits it. */
	TexturedTriPrim() {}
};

struct TexturedQuadPrim
{
	TextureWindow window;
	psyqo::Prim::GouraudTexturedQuad quad;

	TexturedQuadPrim() {}
};

typedef psyqo::Fragments::SimpleFragment<TexturedTriPrim>              TexturedTriFragment;
typedef psyqo::Fragments::SimpleFragment<TexturedQuadPrim>             TexturedQuadFragment;
typedef psyqo::Fragments::SimpleFragment<psyqo::Prim::GouraudTriangle> FlatTriFragment;
typedef psyqo::Fragments::SimpleFragment<psyqo::Prim::GouraudQuad>     FlatQuadFragment;

/* The outlines in wireframe: the sides and back to the start. */
typedef psyqo::Fragments::SimpleFragment<psyqo::Prim::PolyLine<3>>     WireTriFragment;
typedef psyqo::Fragments::SimpleFragment<psyqo::Prim::PolyLine<4>>     WireQuadFragment;

/* One level of the subdivision: a face with a vertex within 'range'
   (metres) is cut 'pieces' along every edge, a power of two up to the
   mesh's cap. Levels go nearest first; past the last, faces stay whole. */
struct MeshLod
{
	Fixed   range;
	uint8_t pieces;
};

/* The levels in force for every subdivided mesh. Two are built in; a
   table given here replaces them and must outlive its use. */
void mesh_setLod(const MeshLod *levels, uint8_t count);

/* Callbacks for custom drawing. */
typedef bool (*ModelFilterCb)(void* userData, const Object *obj);

/* Defines settings and callbacks for custom drawing */
struct ModelDrawConf
{
	void* userData;
	ModelFilterCb filterCb; /* callback to filter parts */

	/* Texels added to every vertex's coordinates this frame, on the base
	   texture and on the glow: the texture window wraps them, so a value
	   that keeps growing scrolls the texture. Written by whoever animates
	   the material, the water for one. */
	uint8_t scroll_u, scroll_v;
	uint8_t glow_scroll_u, glow_scroll_v;
};

struct Mesh
{
	/* Recorded parts: the part each object belongs to, part-major. A static
	   mesh records one set of parts; a skinned one draws the same parts
	   over a palette per frame buffer. */
	uint8_t       *object_part;   /* part index per object; NULL until parts are recorded */
	uint8_t        dl_count;      /* parts */
	uint8_t        dl_buffers;    /* palettes per part: 1, or FB_COUNT when skinned */
	uint8_t        visible;       /* bitmask: parts to render */
	Transform     *matrix_buffer; /* model transform per fb; NULL = identity */
	Model         *model;
	Armature      *skeleton;      /* NULL = static mesh (set by character3d_create) */
	bool           subdivide;     /* cut big faces when drawn; see Prefab3D */

	/* Bone palette of a skinned mesh: FB_COUNT runs of one transform per
	   bone, composed on the CPU each frame (updatePalette) and read by the
	   render through the recorded parts. NULL = not skinned. */
	Transform *palette;

	/* Where the vertices come from when something else drives them. The
	   binding lives in its own module, so the mesh only needs to know it is
	   there. NULL = vertices come straight from the model. */
	MeshDeform *deform;

	/* Handed to the per-frame material setup when the mesh draws through the
	   object path; lets the owner scroll tiles or swap textures. NULL for
	   everything that has no business there. */
	ModelDrawConf *draw_conf;

	/* Model-space box of the whole mesh, taken once. */
	int16_t local_min[3];
	int16_t local_max[3];

	/* World box per model object, rebuilt with the matrix. Index 0 is the
	   whole mesh. */
	MeshBound *bound;
	uint8_t    bound_count;
	bool       culled;


	void initBounds();

	/* Tests the world boxes against the frustum and writes the mesh's own
	   visibility: culled for the whole mesh, isVisible per model object for
	   meshes drawn through the object path. */
	void cull(const Viewport *viewport);

	void setMatrix(const RenderTransform *transform, uint8_t fb_index);

	/* Same, but from a simulated body: position in metres and a quaternion,
	   which is what a tumbling body actually has. */
	void setMatrixFromBody(const Vector3 *position, const Quaternion *rotation,
	                       const Vector3 *scale, uint8_t fb_index);

	/* Records part 0 (every object not in the list) plus one part per named
	   object, in list order. With a skeleton set, allocates the bone palette
	   and draws every part over its run of it per frame buffer. */
	void recordParts(const char *const *names, uint8_t count);

	/* The palette run a part reads this frame. */
	const Transform *partPalette(uint8_t fb_index) const
	{
		if (!palette) return NULL;
		return palette + (dl_buffers > 1 ? fb_index : 0) * skeleton->skeletonRef->boneCount;
	}

	/* Writes this frame's palette run: for every bone, the frame's view over
	   the model transform over the bone's model-space transform, which is
	   what the GTE takes per bone. Nothing for a mesh without a palette. */
	void updatePalette(const Viewport *viewport, uint8_t fb_index);

	/* Draws through the object path: every object on its own, culled one by
	   one. */
	void recordObjects();

	/* Hands the vertices over to an external set of points, matched by rest
	   position. `scale` converts source units to the vertices' units. Pass
	   source_normal to have the shading follow the deformation, and
	   source_rgba to drive the vertex colors too; NULL keeps the model's own. */
	bool setDeform(const Vector3 *source, const Vector3 *source_normal,
	               const uint8_t *source_rgba, uint16_t source_count, Fixed scale);

	/* Pushes the current source positions and normals into the vertex buffer.
	   No-op when the mesh is not deformed. */
	void updateDeform(uint8_t fb_index);

	/* Points the deformed mesh at this frame's vertices. Call right before it
	   is drawn. No-op when the mesh is not deformed. */
	void bindDeformFrame(uint8_t fb_index);

	/* The vertices the render reads for an object this frame: the deform's
	   copy when there is one, the model's otherwise. */
	const RenderVertex *vertices(uint32_t object) const;

	/* Draws one element of the mesh into the frame: every object of the
	   part (or every visible object, through the object path) goes through
	   the GTE to camera space and is lit, clipped against the near plane,
	   subdivided when close, and emitted as triangles and quads into the
	   render's table. */
	void draw(const Element3D *element) const;
};

/* The drawing's time per frame, in horizontal blanks: every face through
   the GTE, clipped, cut, and emitted. Reset by the render at the start of
   the frame. */
extern uint32_t mesh_profile_faces;

/* Of that, the vertices through the GTE and the primitives into the
   table, in ticks of root counter 2 (0.24 us). The rest is the cutting. */
extern uint32_t mesh_profile_transform;
extern uint32_t mesh_profile_emit;


#endif
