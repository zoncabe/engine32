/*
	Verlet cloth with distance constraints solved by relaxation.

	Implements the method in "Advanced Character Physics" (Thomas Jakobsen,
	IO Interactive, GDC 2001), the cloth and ragdoll solver of the fysix
	engine used in Hitman.

	Two properties make it cheap enough to run on the CPU here. Verlet keeps
	the previous position instead of a velocity, so integration is one
	multiply-add per axis. And constraints are satisfied by projecting
	positions with Jakobsen's square-root approximation: a satisfied
	constraint returns exactly the rest length, so expanding the root to
	first order around it costs one division per constraint and no sqrt.

	The particle array is the simulation state and doubles as the source for
	meshDeform_apply, so the rendered mesh follows without a copy in between.
*/
#ifndef ENGINE_32_CLOTH_H
#define ENGINE_32_CLOTH_H

#include <stdint.h>

#include "physics/math/e32_vector3.h"
#include "physics/collision/e32_collision_mesh.h"


/* Authoring-side description. The particles are seeded from a welded
   collision mesh of the same model, so the topology comes from the asset
   instead of being spelled out here. Wind is not part of it: it is written
   into the cloth by the world at runtime, frame by frame. */
struct ClothDef
{
	const char *mesh_path;     /* welded collision mesh that seeds the particles */
	Fixed       damping;       /* 0 = none, bleeds off the Verlet step */
	uint8_t     iterations;    /* relaxation passes */
	Fixed       pin_max_x;     /* particles at or below this X are held in place */
};


struct ClothConstraint
{
	uint16_t a, b;
	Fixed    rest_length_sq;   /* the approximation consumes it squared */
};


struct Cloth
{
	Vector3  *position;        /* current positions; the mesh reads these */
	Vector3  *previous;        /* Verlet history, one step back */
	Vector3  *force;           /* per-particle accumulator, refilled every step */

	/* Rebuilt after every step from the faces around each particle. Without
	   this the cloth ripples but keeps shading as the flat sheet it started
	   as, because the model's own normals never move. */
	Vector3  *normal;
	Vector3  *render_position; /* previous/current step blend; the mesh reads these */
	uint8_t  *pinned;          /* non-zero = held in place by the solver */
	uint16_t  particle_count;

	ClothConstraint *constraint;
	uint16_t         constraint_count;

	/* Wind acts on the surface, so the triangles have to survive past setup:
	   a face edge-on to the wind catches nothing, and that is what lets the
	   cloth turn to face it. */
	uint16_t *triangle;        /* 3 particle indices per triangle */
	uint16_t  triangle_count;

	/* What the world does to the cloth, written from outside every frame.
	   Gravity is uniform; wind is not, it is resolved per triangle. */
	Vector3   gravity;
	Vector3   wind;
	Fixed     damping;         /* 0 = none, bleeds off the Verlet step */
	uint8_t   iterations;      /* relaxation passes; Jakobsen: 3-4 is plenty */

	/* Set by whoever draws the cloth. Out of view it holds its pose instead of
	   stepping: Verlet carries the velocity in the current and previous
	   positions, so leaving both untouched resumes the motion where it
	   stopped. NULL = always steps. */
	const bool *culled;

	Cloth *next;               /* the world keeps these in one list */


	/* Builds particles from the mesh vertices and one constraint per unique
	   mesh edge, then applies the def. The mesh must be welded, or shared
	   edges become free borders. */
	bool create(const CollisionMesh *mesh, const ClothDef *def);

	/* Pins every particle the predicate accepts, in mesh-local space: used to
	   hold an edge against a pole. */
	void pinWhere(bool (*predicate)(const Vector3 &position, void *user), void *user);

	/* True while the body standing for the cloth is out of view. */
	bool isCulled() const { return culled != NULL && *culled; }

	void step(Fixed dt);

	/* Fills render_position with the previous/current step blend: t is how
	   far into the next step the frame sits, accumulator over timestep. */
	void blendRenderState(Fixed t);

	/* Gives back the particle buffers. The Cloth itself belongs to the world. */
	void free();
};


#endif
