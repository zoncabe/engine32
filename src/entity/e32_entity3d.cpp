#include "psyqo/alloc.h"
#include "psyqo/kernel.hh"

#include "animation/e32_model.h"
#include "animation/e32_armature.h"

#include "entity/e32_entity3d.h"
#include "resource/e32_resource.h"
#include "shaders/e32_mesh_deform.h"
#include "viewport/e32_viewport.h"
#include "physics/math/e32_math_common.h"
#ifdef ENGINE_32_SOUND
#include "sound/e32_sound.h"
#endif
#ifdef ENGINE_32_PHYSICS
#include "physics/world/e32_physics_world.h"
#include "physics/body/e32_rigid_body.h"
#endif
#ifdef ENGINE_32_CHARACTER
#include "character3d/e32_character3d_physics.h"
#include "character3d/e32_character3d_animation.h"
#endif


void entity3d_init(Entity3D *entity, const Entity3DDef *def)
{
	*entity = Entity3D();
	entity->transform.init();
	entity->transform.position = def->position;
	entity->transform.rotation = def->rotation;
	entity->transform.scale    = def->scale;
	entity->cull               = def->cull;
}

Entity3D *entity3d_create(const Entity3DDef *def)
{
	Entity3D *entity = (Entity3D *)psyqo_malloc(sizeof(Entity3D));
	psyqo::Kernel::assert(entity != NULL, "entity: out of memory");
	entity3d_init(entity, def);

#ifdef ENGINE_32_SOUND
	/* The sounds open with the entity. A looping one is the object's own
	   noise: it starts here, from where the object stands, and stops when
	   the entity goes. */
	if (def->sound_count) {
		entity->sound = (Sound *)psyqo_malloc(def->sound_count * sizeof(Sound));
		psyqo::Kernel::assert(entity->sound != NULL, "entity: out of memory");
		for (int i = 0; i < def->sound_count; i++) {
			entity->sound[i] = sound_load(def->sound[i]);
			if (def->sound[i]->loop)
				sound_play(&entity->sound[i], &entity->transform.position, 1.0_fp, Fixed());
		}
		entity->sound_count = def->sound_count;
	}
#endif

	/* A sound placed alone has nothing to draw. */
	if (!def->model_path) return entity;

	entity->mesh = (Mesh *)psyqo_malloc(sizeof(Mesh));
	psyqo::Kernel::assert(entity->mesh != NULL, "entity: out of memory");
	*entity->mesh = Mesh();
	entity->mesh->model = Model::load(def->model_path);
	psyqo::Kernel::assert(entity->mesh->model != NULL, "entity: model not found");
	entity->mesh->matrix_buffer = (Transform *)psyqo_malloc(sizeof(Transform) * FB_COUNT);
	psyqo::Kernel::assert(entity->mesh->matrix_buffer != NULL, "entity: out of memory");
	for (int fb = 0; fb < FB_COUNT; fb++)
		entity->mesh->matrix_buffer[fb] = Transform::identity();

	entity->mesh->skeleton   = NULL;
	entity->mesh->subdivide  = def->subdivide;
	/* Part 0 and every part the def does not single out follow the model-wide
	   flag; a named part with its own says what it says. */
	entity->mesh->vertex_lighting = def->vertex_lighting ? 0xFF : 0;
	entity->mesh->deform     = NULL;
	entity->mesh->draw_conf  = NULL;
	entity->mesh->palette    = NULL;
	entity->mesh->dl_buffers = 1;
	entity->mesh->initBounds();

	if (def->character) {
		entity->mesh->object_part = NULL;   /* character3d_create builds the skinned parts */
		entity->mesh->dl_count = 0;
		entity->mesh->visible  = 0;
	} else if (def->cloth) {
		/* One part with the whole model in it. */
		entity->mesh->recordParts(NULL, 0);
	} else if (def->part_count) {
		/* The model is static, so its parts record with no palette, and every
		   one of them starts on screen: a prop shows whole until the game
		   decides to hide something. */
		entity->mesh->recordParts(def->part, def->part_count);
		entity->mesh->visible = (uint8_t)((1u << entity->mesh->dl_count) - 1);

		/* A part declared away from where it was modelled gets its offset
		   here, so it is already in place the first time it is drawn. */
		if (def->part_position)
			entity->mesh->setPartOffsets(def->part_position, def->part_count);

		if (def->part_vertex_lighting) {
			uint8_t mask = def->vertex_lighting ? 1 : 0;   /* part 0, the remainder */
			for (uint8_t i = 0; i < def->part_count; i++)
				if (def->part_vertex_lighting[i]) mask |= (uint8_t)(1u << (i + 1));
			entity->mesh->vertex_lighting = mask;
		}
	} else {
		entity->mesh->recordObjects();
	}

	return entity;
}

/* Shows or hides one of the model's own objects, by the name the prefab
   recorded it under. A name the entity has no part for does nothing. */
void entity3d_setPartVisible(Entity3D *entity, const char *name, bool visible)
{
	if (entity->mesh == NULL) return;

	uint8_t part = entity->mesh->findPart(name);
	if (part == 0) return;

	entity->mesh->setPartVisible(part, visible);
}


void entity3d_delete(Entity3D *entity)
{
	if (entity->mesh) {
		if (entity->mesh->object_part) psyqo_free(entity->mesh->object_part);
		if (entity->mesh->palette)     psyqo_free(entity->mesh->palette);
		if (entity->mesh->deform) {
			entity->mesh->deform->destroy();
			psyqo_free(entity->mesh->deform);
		}
		if (entity->mesh->part_matrix) psyqo_free(entity->mesh->part_matrix);
		psyqo_free(entity->mesh->bound);
		psyqo_free(entity->mesh->matrix_buffer);
		entity->mesh->model->free();
		psyqo_free(entity->mesh);
	}

#ifdef ENGINE_32_SOUND
	for (int i = 0; i < entity->sound_count; i++)
		sound_unload(&entity->sound[i]);
	if (entity->sound) psyqo_free(entity->sound);
#endif

	psyqo_free(entity);
}

void entity3d_playSound(const Entity3D *entity, uint8_t trigger, const Vector3 *position, Fixed volume_scale)
{
#ifdef ENGINE_32_SOUND
	/* The candidates are the entity's sounds tagged with this trigger; the
	   loops are already playing and never fire. */
	uint8_t candidate[SOUND_MAX_EMITTERS];
	uint8_t count = 0;

	for (uint8_t i = 0; i < entity->sound_count && count < SOUND_MAX_EMITTERS; i++) {
		const SoundDef *def = entity->sound[i].def;
		if (def->loop || def->trigger != trigger) continue;
		candidate[count++] = i;
	}
	if (count == 0) return;

	Vector3 here;
	if (position == NULL) {
		if (entity->body) here = entity->body->tx.position;
		else               here = entity->transform.position;
		position = &here;
	}

	sound_play(&entity->sound[candidate[rand() % count]], position, volume_scale, Fixed());
#else
	(void)entity; (void)trigger; (void)position; (void)volume_scale;
#endif
}

/* The body and the render transform are both in meters. */
void entity3d_setTransform(Entity3D *entity, const KinematicBody *body)
{
#ifdef ENGINE_32_PHYSICS
	entity->transform.position = body->position;
	entity->transform.rotation = body->rotation;
#else
	(void)entity; (void)body;
#endif
}

void entity3d_setMatrix(Entity3D *entity, uint8_t fb_index)
{
	entity->mesh->setMatrix(&entity->transform, fb_index);
}

/* For entities the solver moves: their placement lives in the body, not in the
   render transform, and a tumbling body needs its quaternion rather than the
   euler angles the transform carries. No-op for anything else. */
void entity3d_setMatrixFromBody(Entity3D *entity, uint8_t fb_index)
{
#ifdef ENGINE_32_PHYSICS
	if (entity->body == NULL || !(entity->body->flags & BODY_FLAG_DYNAMIC)) return;

	entity->mesh->setMatrixFromBody(&entity->body->tx.position, &entity->body->q,
	                                &entity->transform.scale, fb_index);
#else
	(void)entity; (void)fb_index;
#endif
}


/* World transform of a static collider: entity position in meters plus the
   entity rotation built with the same euler function the renderer uses, so
   collision and visuals always match. */
Transform entity3d_colliderTransform(const Entity3DDef *def)
{
	Transform t;
	t.rotation = Matrix3::fromEuler(def->rotation.x, def->rotation.y, def->rotation.z);
	t.position = def->position;
	return t;
}

/* Copies the body-def override bits on top of the freshly-initialised body
   (position, orientation) and then attaches the entity's shape to it. */
RigidBody *entity3d_attachPhysics(Entity3D *entity, const Entity3DDef *def, PhysicsWorld *world)
{
#ifdef ENGINE_32_PHYSICS
	RigidBodyDef body_def;
	body_def.init();

	if (def->body) {
		/* The user-supplied RigidBodyDef describes only the body properties,
		   not the world position. Position/rotation come from the entity. */
		body_def.body_type       = def->body->body_type;
		body_def.gravity_scale   = def->body->gravity_scale;
		body_def.layers          = def->body->layers ? def->body->layers : 1;
		body_def.linear_damping  = def->body->linear_damping;
		body_def.angular_damping = def->body->angular_damping;
		body_def.allow_sleep     = def->body->allow_sleep;
		body_def.awake           = def->body->awake;
		body_def.active          = def->body->active;
		body_def.lock_axis_x     = def->body->lock_axis_x;
		body_def.lock_axis_y     = def->body->lock_axis_y;
		body_def.lock_axis_z     = def->body->lock_axis_z;
	}

	/* Position in meters and rotation through the renderer's euler convention,
	   both from the collider transform: a rotated entity collides the way it
	   renders. */
	Transform collider = entity3d_colliderTransform(def);
	body_def.position = collider.position;

	Quaternion rotation = Quaternion::fromMatrix3(collider.rotation);
	rotation.toAxisAngle(&body_def.axis, &body_def.angle);

	/* Identity degenerates to a zero axis; any axis stands for no rotation. */
	if (body_def.axis.squaredMagnitude() == 0)
		body_def.axis = { 0.0_fp, 0.0_fp, 1.0_fp };

	RigidBody *body = world->createBody(&body_def);
	body->owner   = entity;
	entity->body  = body;

	if (def->collider) {
		for (uint8_t i = 0; i < def->collider->count; i++)
			body->addShape(&def->collider->shape[i], def->scale);
	}

	return body;
#else
	(void)entity; (void)def; (void)world;
	return NULL;
#endif
}
