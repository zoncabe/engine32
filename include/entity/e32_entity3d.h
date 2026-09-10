#ifndef ENGINE_32_ENTITY3D_H
#define ENGINE_32_ENTITY3D_H

#include <stdbool.h>
#include <stdint.h>

#include "render/e32_render.h"
#include "graphics/e32_mesh.h"
#include "physics/math/e32_transform.h"
#include "physics/math/e32_quaternion.h"


/* From the modules that are not ported yet: only pointed at from here. */
struct RigidBody;
struct RigidBodyDef;
struct KinematicBody;
struct PhysicsShapeDef;
struct PhysicsWorld;
struct ClothDef;
struct WaterDef;
struct Sound;
struct SoundDef;
struct Character3DDef;


struct Entity3D
{
	RenderTransform transform;
	Mesh *mesh;

	/* Set when the entity has a rigid body. If that body is simulated, it is
	   what places the mesh each frame instead of the transform above. */
	RigidBody *body;

	/* Test the model's bounding box against the view frustum before drawing
	   it. Off means the entity is drawn every frame, no questions asked. */
	bool cull;

	/* The prefab's sounds, open: one per entry, in the same order, from
	   create to delete. The looping ones play from the entity for as long
	   as it exists; the rest wait for whoever fires them, the character
	   its own, the game the prop's. */
	Sound   *sound;
	uint8_t  sound_count;
};


/* A collider is one or more primitives, each carrying its own offset in its
   .tx. The entity transform and scale apply to all of them, so the group
   stays consistent at any prop size. */
struct Entity3DColliderDef
{
	const PhysicsShapeDef *shape;
	uint8_t                count;
};


struct Entity3DDef
{
	const char *model_path;   /* NULL: nothing to draw, a sound placed alone */
	bool        subdivide;    /* cut big faces when drawn; see Prefab3D */
	const SoundDef *const *sound;
	uint8_t                sound_count;
	Vector3     position;
	EulerAngles rotation;
	Vector3     scale;
	const Character3DDef *character;
	const RigidBodyDef      *body;
	const Entity3DColliderDef *collider;
	const ClothDef          *cloth;
	const WaterDef          *water;
	bool cull;
};


void entity3d_init(Entity3D *entity, const Entity3DDef *def);
Entity3D *entity3d_create(const Entity3DDef *def);
void entity3d_delete(Entity3D *entity);
void entity3d_setTransform(Entity3D *entity, const KinematicBody *body);
void entity3d_setMatrix(Entity3D *entity, uint8_t fb_index);
void entity3d_setMatrixFromBody(Entity3D *entity, uint8_t fb_index);

/* Fires a trigger on the entity: one of its sounds declared with that trigger
   plays, picked at random. position NULL plays from where the entity is; a
   given one is in meters. Nothing declared, nothing plays. */
void entity3d_playSound(const Entity3D *entity, uint8_t trigger, const Vector3 *position, Fixed volume_scale);

/* Def → physics wiring. The caller owns the destinations. */
Transform entity3d_colliderTransform(const Entity3DDef *def);

/* Builds the entity's body and hangs its collider off it. Without a .body def
   the body comes out static, which is what scenery wants. */
RigidBody *entity3d_attachPhysics(Entity3D *entity, const Entity3DDef *def, PhysicsWorld *world);

#endif
