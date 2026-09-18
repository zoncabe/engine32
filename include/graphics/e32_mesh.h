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

/* Declaring a model's parts and where each one is drawn, for a prefab:
   MESH_PARTS lists the object names as they are named in the model,
   MESH_PART_POSITIONS one position per name, in the same order. */
#define MESH_PARTS(...)          (const char *const[]){ __VA_ARGS__ }
#define MESH_PART_POSITIONS(...) (const Vector3[])   { __VA_ARGS__ }

/* One flag per name, same order: which parts light a vertex at a time. */
#define MESH_PART_LIGHTING(...)  (const bool[])      { __VA_ARGS__ }


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

	/* A part drawn away from where it was modelled. The offsets are the
	   prefab's, in the entity's own space; the matrices are the model
	   transform with each one folded in, rebuilt per frame buffer whenever
	   the mesh's matrix is written. Both NULL when no part is displaced,
	   which is every mesh that does not ask for it. */
	const Vector3 *part_offset;   /* one per named part, part 1 first */
	Transform     *part_matrix;   /* dl_count per frame buffer */

	/* The names the parts were recorded with, so the game can address one by
	   the name it wrote in the prefab. The caller's array, part 1 first. */
	const char *const *part_name;
	uint8_t            part_count;   /* named parts; dl_count is one more */
	Model         *model;
	Armature      *skeleton;      /* NULL = static mesh (set by character3d_create) */
	bool           subdivide;       /* cut big faces when drawn; see Prefab3D */

	/* Which parts light a vertex at a time, a bit per part, part 0 first.
	   A mesh with no parts recorded reads bit 0. See Prefab3D. */
	uint8_t        vertex_lighting;

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

	/* Where each named part is drawn inside the entity, one entry per name and
	   in the same order, so count is the number of names, not the parts. Part
	   0, everything the names left out, is always drawn where it was modelled.
	   The offsets are the caller's and must outlive the mesh, the way the
	   names do. A part left at zero stays where it was modelled. Call after
	   recordParts. */
	void setPartOffsets(const Vector3 *offsets, uint8_t count);

	/* The part a name stands for, 1 based, or 0 when the mesh has no part by
	   that name. Part 0 is the unnamed remainder and has no name to find. */
	uint8_t findPart(const char *name) const;

	void setPartVisible(uint8_t part, bool shown)
	{
		if (part >= dl_count) return;
		if (shown) visible |=  (uint8_t)(1u << part);
		else       visible &= (uint8_t)~(1u << part);
	}

	bool isPartVisible(uint8_t part) const { return (visible & (1u << part)) != 0; }

	/* The matrix this part is drawn with: the mesh's own, or the one carrying
	   its offset. Never NULL once a matrix buffer exists. */
	const Transform *partMatrix(uint8_t part, uint8_t fb_index) const
	{
		if (!part_matrix) return matrix_buffer ? &matrix_buffer[fb_index] : NULL;
		return &part_matrix[fb_index * dl_count + part];
	}

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

	/* The three tables the render reads for an object this frame: the
	   deform's copy when there is one, the model's otherwise. A vertex is a
	   corner, and holds an index into each of the other two. */
	const RenderVertex   *vertices(uint32_t object) const;
	const RenderPosition *positions(uint32_t object) const;
	const RenderShade    *shades(uint32_t object) const;

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

/* How many times a piece is offered for drawing: once per face of a whole
   mesh, once per cell of a subdivided one. Against the primitives the
   frame ends up with, it says how much of the work is thrown away. */
extern uint32_t mesh_profile_pieces;

/* How many pieces had to be cut to the view, and how many faces the loop
   walked at all. */
extern uint32_t mesh_profile_cuts;
extern uint32_t mesh_profile_walked;

/* The time the cutting takes, and the time everything a face goes through
   after the loop has picked it takes, in ticks of root counter 2. */
extern uint32_t mesh_profile_clip;
extern uint32_t mesh_profile_prim;

/* The cutting broken down, same ticks. 'cut' is the Sutherland-Hodgman walk
   alone, without what it emits; within it, 'lerp' is the interpolation of
   the points where an edge crosses a side, and 'project' is putting those
   new points on the screen through the GTE. */
extern uint32_t mesh_profile_direct;
extern uint32_t mesh_profile_cut;
extern uint32_t mesh_profile_lerp;
extern uint32_t mesh_profile_project;

/* What the drawing costs outside the face loops, same ticks. 'level' is the
   pass that settles every face's subdivision over the whole model, 'object'
   is what each object takes before its faces start: its material, the scroll,
   and the growth of the scratch buffers. */
extern uint32_t mesh_profile_level;
extern uint32_t mesh_profile_object;

/* The whole of Mesh::draw, per element, same ticks. */
extern uint32_t mesh_profile_draw;

/* What every element costs before any of its objects is looked at: folding
   the view with its placement, and putting the lights in its space. */
extern uint32_t mesh_profile_prep;

/* Within it, putting the lights in the element's space alone, and how many
   elements the frame drew. */
/* The same cut walked a second time, right after the first, so that the
   second finds the code in the instruction cache and the first does not.
   Built only with E32_MESH_CUT_TWICE, which makes the engine do the work
   twice: a measurement, never a build to ship. */
extern uint32_t mesh_profile_cut2;

/* mesh_prim broken down, same ticks: a face drawn whole, a face cut in
   pieces, and the cover triangles along an edge a finer neighbour meets.
   The cutting and the emission live inside all three. */
/* Counts, not ticks: of the faces the direct loop walks, the ones handed to
   the path that cuts, the ones dropped for looking away, and the ones
   dropped for their depth. */
/* The lowest and highest bucket of the ordering table the element being
   drawn wrote into. Read right after a Mesh::draw returns. */
extern uint32_t mesh_otz_min;
extern uint32_t mesh_otz_max;

extern uint32_t mesh_profile_slow;
extern uint32_t mesh_profile_back;
extern uint32_t mesh_profile_far;

extern uint32_t mesh_profile_whole;
extern uint32_t mesh_profile_sub;
extern uint32_t mesh_profile_cover;

extern uint32_t mesh_profile_light;
extern uint32_t mesh_profile_elements;


#endif
