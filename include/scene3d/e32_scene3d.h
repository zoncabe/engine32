#ifndef ENGINE_32_SCENE3D_H
#define ENGINE_32_SCENE3D_H

#include "prefab/e32_prefab3d.h"
#include "entity/e32_entity3d.h"
#include "scene3d/e32_lighting.h"
#include "scene3d/e32_fog.h"
#include "camera/e32_camera.h"
#include "camera/e32_spring_arm.h"
#include "render/e32_render.h"

#define SCENE_MAX_CHARACTERS 6

#define SCENE_MAX_ENTITIES 64
/* Counts primitives, not entities: one compound collider takes several. */


/* From the modules that are not ported yet: only pointed at from here. */
struct Character3D;
struct PhysicsWorld;


/* One prefab placed in a scene. A scene's content is an array of these; the
   load walks it in order. A zero scale means identity. */
struct Scene3DPrefab
{
	const Prefab3D *prefab;
	Vector3     position;
	EulerAngles rotation;
	Vector3     scale;
};

struct Scene3DDef
{
	const LightDef *light;
	const FogDef *fog;
	const CameraDef *camera;
	Vector3 wind;

	const Scene3DPrefab *prefab;
	uint8_t prefab_count;
};


struct Scene3D
{
	Entity3D *entity[SCENE_MAX_ENTITIES];
	uint8_t entity_count;

	Character3D *character[SCENE_MAX_CHARACTERS];
	uint8_t character3d_count;
};

Scene3D *scene3d_get(void);

/* The physics world the scene loaded its bodies and cloths into. */
PhysicsWorld *scene3d_getPhysics(void);

/* The entities come out in placement order: the scene's entity list reads
   by the same index as the def's prefab list. */
void scene3d_load(const Scene3DDef *def);
void scene3d_clear(void);
void scene3d_unload(void);
/* Collides every character and carries the result to what draws it. Call
   after physics_update, with the frame's buffer index. */
void scene3d_updateCharacters(uint8_t fb_index);

void scene3d_addEntity(Entity3D *entity);
Character3D *scene3d_getCharacter3D(uint8_t index);

/* Pushes one Element3D per visible mesh part into the frame's context. The
   culling runs here, against the frame's frustum, before each mesh is read. */
void scene3d_setRenderContext(const Scene3D *scene, RenderContext *ctx, const Viewport *viewport);

#endif
