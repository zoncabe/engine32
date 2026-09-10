#include "psyqo/kernel.hh"

#include "physics/math/e32_math.h"

#include "shaders/e32_mesh_deform.h"
#include "viewport/e32_viewport.h"
#include "render/e32_render.h"
#include "graphics/e32_texture.h"
#include "scene3d/e32_lighting.h"
#include "scene3d/e32_fog.h"
#include "entity/e32_entity3d.h"
#include "scene3d/e32_scene3d.h"
#ifdef ENGINE_32_PHYSICS
#include "physics/world/e32_physics_world.h"
#include "physics/shapes/e32_physics_shape.h"
#include "physics/body/e32_rigid_body.h"
#include "physics/e32_physics_settings.h"
#endif
#ifdef ENGINE_32_WATER
#include "shaders/e32_water.h"
#endif
#ifdef ENGINE_32_CHARACTER
#include "character3d/e32_character3d.h"
#include "character3d/e32_character3d_physics.h"
#endif


static Scene3D scene;
#ifdef ENGINE_32_PHYSICS
static PhysicsWorld g_physics;
#endif

Scene3D        *scene3d_get(void)        { return &scene; }
#ifdef ENGINE_32_PHYSICS
PhysicsWorld *scene3d_getPhysics(void) { return &g_physics; }
PhysicsWorld *physics_getWorld(void) { return &g_physics; }
#else
PhysicsWorld *scene3d_getPhysics(void) { return NULL; }
#endif

void scene3d_load(const Scene3DDef *def)
{
	/* A scene with no light def is left pitch black rather than lit by
	   something the game never asked for. */
	static const LightDef unlit = {};
	static const FogDef   clear = {};

	const LightDef *light = def->light ? def->light : &unlit;
	const FogDef   *fog   = def->fog   ? def->fog   : &clear;

	Lighting::init(light);
	FogState::init(fog);

	Camera *camera = &Viewport::get().camera;
	camera->reset();
	if (def->camera) {
		camera->target_field_of_view = def->camera->field_of_view;
		camera->field_of_view        = def->camera->field_of_view;
		camera->near_clipping        = def->camera->near_clipping;
		camera->far_clipping         = def->camera->far_clipping;
		camera->base_near_clipping   = def->camera->near_clipping;
		camera->base_far_clipping    = def->camera->far_clipping;
		camera->auto_clipping        = def->camera->auto_clipping;
	}
	switch (def->camera ? def->camera->type : CAMERA_TYPE_NONE) {
		case CAMERA_TYPE_SPRING_ARM:
			CameraSpringArm::init(camera, &def->camera->spring_arm);
			break;
		case CAMERA_TYPE_NONE:
		case CAMERA_TYPE_COUNT:
			break;
	}

	psyqo::Kernel::assert(scene.entity_count == 0, "scene3d: load over a loaded scene");
	scene = Scene3D();

#ifdef ENGINE_32_PHYSICS
	Vector3 gravity = { 0.0_fp, 0.0_fp, -9.8_fp };
	/* Bodies and cloths are only ever made here, one per prefab at most. */
	g_physics.init(PHYSICS_TIMESTEP, gravity, PHYSICS_SOLVER_ITERATIONS, def->prefab_count);
	g_physics.setWind(def->wind);
#endif

	for (int i = 0; i < def->prefab_count; i++) {
		const Scene3DPrefab *placed = &def->prefab[i];
		const Prefab3D *prefab = placed->prefab;

		Vector3 scale = placed->scale;
		if (scale.x == 0 && scale.y == 0 && scale.z == 0)
			scale = { 1.0_fp, 1.0_fp, 1.0_fp };

		/* entity3d.cpp builds from a flat parameter block: filled here
		   straight from the prefab and its placement, and gone after the load. */
		Entity3DDef entity_def = {};
		entity_def.model_path  = prefab->model;
		entity_def.subdivide   = prefab->subdivide;
		entity_def.sound       = prefab->sound;
		entity_def.sound_count = prefab->sound_count;
		entity_def.position    = placed->position;
		entity_def.rotation    = placed->rotation;
		entity_def.scale       = scale;
		entity_def.collider    = prefab->collider;
		entity_def.cull        = true;

		switch (prefab->type) {
			case PREFAB3D_CHARACTER:
				entity_def.character = prefab->character;
				break;
			case PREFAB3D_PROP:
				entity_def.body = prefab->prop;
				break;
			case PREFAB3D_CLOTH:
				entity_def.cloth = prefab->cloth;
				break;
			case PREFAB3D_WATER:
				entity_def.water = prefab->water;
				break;
		}

		Entity3D *entity = entity3d_create(&entity_def);

#ifdef ENGINE_32_PHYSICS
		if (entity_def.collider)
			entity3d_attachPhysics(entity, &entity_def, &g_physics);

		if (entity_def.cloth) {
			Cloth *cloth = g_physics.createCloth(entity_def.cloth);
			/* The cloth runs in meters, the vertex buffer in the model's units. */
			if (cloth) {
				cloth->culled = &entity->mesh->culled;
				entity->mesh->setDeform(cloth->render_position, cloth->normal,
				                        NULL, cloth->particle_count, Fixed(MODEL_UNITS_PER_METER, 0));
			}
		}
#endif

#ifdef ENGINE_32_WATER
		if (entity_def.water) {
			Water *water = water_create(entity_def.water);
			/* Same contract as the cloth: points in meters, buffer in the
			   model's units. The draw conf is what scrolls the texture
			   layers, so it only works through the per-frame material path. */
			if (water) {
				water->culled = &entity->mesh->culled;
				entity->mesh->setDeform(water->position, water->normal,
				                        water->rgba, water->count, Fixed(MODEL_UNITS_PER_METER, 0));
				entity->mesh->draw_conf = &water->conf;

				/* The entity's collider is the water's sensor volume: bind
				   them and the bodies inside it start floating. */
				if (entity->body)
					water_bindPhysics(water, entity->body, &g_physics);
			}
		}
#endif

#ifdef ENGINE_32_CHARACTER
		if (entity_def.character) {
			psyqo::Kernel::assert(scene.character3d_count < SCENE_MAX_CHARACTERS, "scene3d: too many characters");
			Character3D *character = character3d_create(entity_def.character, entity);
			scene.character[scene.character3d_count++] = character;

			character3dPhysics_createBody(character, &g_physics);

			const Character3DWeaponsDef *weapons = entity_def.character->weapons_def;
			for (int slot = 0; weapons && slot < WEAPON_SLOT_COUNT; slot++)
				if (weapons->weapon[slot])
					character3d_equipWeapon(character, slot, weapons->weapon[slot]);
		}
#endif

		/* Every copy of the matrix, characters included: the palette reads
		   the frame's copy on the first render, before any update wrote it. */
		if (entity->mesh)
			for (int fb = 0; fb < FB_COUNT; fb++)
				entity->mesh->setMatrix(&entity->transform, fb);

		scene.entity[scene.entity_count++] = entity;
	}
}

void scene3d_clear(void)
{
	scene = Scene3D();
}

void scene3d_unload(void)
{
#ifdef ENGINE_32_CHARACTER
	for (int i = 0; i < scene.character3d_count; i++)
		character3d_delete(scene.character[i]);
#endif
	for (int i = 0; i < scene.entity_count; i++)
		entity3d_delete(scene.entity[i]);
#ifdef ENGINE_32_WATER
	water_clear();
#endif
	scene3d_clear();
#ifdef ENGINE_32_PHYSICS
	g_physics.shutdown();
#endif

	Texture::init();
}

/* The characters' half of the frame after physics_update: each one collides
   against the world, hands the outcome to its body, and from there to what
   draws it. Always these four, always in this order, so no game writes them
   out. The list is the scene's, which is why it lives here. */
void scene3d_updateCharacters(uint8_t fb_index)
{
#ifdef ENGINE_32_CHARACTER
	for (int i = 0; i < scene.character3d_count; i++) {
		Character3D *character = scene.character[i];

		character3dPhysics_collide(character, &g_physics);
		character3dPhysics_syncBody(character);
		entity3d_setTransform(character->entity, &character->body);
		entity3d_setMatrix(character->entity, fb_index);
	}
#else
	(void)fb_index;
#endif
}

void scene3d_addEntity(Entity3D *entity)
{
	psyqo::Kernel::assert(scene.entity_count < SCENE_MAX_ENTITIES, "scene3d: too many entities");
	scene.entity[scene.entity_count++] = entity;
}

Character3D *scene3d_getCharacter3D(uint8_t index)
{
	if (index >= scene.character3d_count) return NULL;
	return scene.character[index];
}

void scene3d_setRenderContext(const Scene3D *s, RenderContext *ctx, const Viewport *viewport)
{
	uint8_t fb_index = viewport->fb_index;

	for (int i = 0; i < s->entity_count; i++) {
		Entity3D    *e      = s->entity[i];
		Mesh        *mesh   = e->mesh;
		if (!mesh) continue;
		const Transform *matrix = mesh->matrix_buffer ? &mesh->matrix_buffer[fb_index] : NULL;
		Armature    *skel   = mesh->skeleton;

		/* The mesh culls itself and writes its own flags; here they are
		   only consumed. */
		if (e->cull) {
			mesh->cull(viewport);
			if (mesh->culled) continue;
		}

		/* Whatever drives this mesh has already moved: fold the new positions
		   into this frame's vertex buffer, then point the mesh at that same
		   copy for the draw. */
		mesh->updateDeform(fb_index);
		mesh->bindDeformFrame(fb_index);

		/* A skinned mesh's pose is in its bones by now: write this frame's
		   palette, which is what its parts read. */
		mesh->updatePalette(viewport, fb_index);

		if (mesh->dl_count == 0) {
			psyqo::Kernel::assert(ctx->object_count < RENDER_MAX_3D_ELEMENTS, "render: too many elements");
			ctx->object[ctx->object_count++] = { mesh, 0, matrix, skel, NULL, mesh->draw_conf };
			continue;
		}

		for (int part = 0; part < mesh->dl_count; part++) {
			if (!(mesh->visible & (1u << part))) continue;

			psyqo::Kernel::assert(ctx->object_count < RENDER_MAX_3D_ELEMENTS, "render: too many elements");
			ctx->object[ctx->object_count++] = { mesh, (uint8_t)part, matrix, skel, mesh->partPalette(fb_index), NULL };
		}
	}
}
