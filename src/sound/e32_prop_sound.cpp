#include "sound/e32_prop_sound.h"
#include "entity/e32_entity3d.h"
#include "shaders/e32_water.h"
#include "physics/collision/e32_contact.h"
#include "physics/body/e32_rigid_body.h"
#include "physics/world/e32_physics_world.h"
#include "physics/math/e32_math_common.h"

/* The measure of a hit is the impulse the solver spent stopping it, over the
   body's mass: the speed it killed, in m/s — one scale for every weight.
   Below the floor it is resting jitter and stays silent. */
#define PROP_SOUND_COLLISION_SPEED_MIN  0.4f
#define PROP_SOUND_COLLISION_SPEED_MAX  3.0f
#define PROP_SOUND_COLLISION_VOLUME_MIN 0.1f
#define PROP_SOUND_COLLISION_VOLUME_MAX 1.0f

/* The plunge is the vertical speed on the frame the body meets the water:
   rolling in barely whispers, a fall from the deck slaps. No cutoff floor —
   entering slowly still wets. */
#define PROP_SOUND_PLUNGE_SPEED_MIN  0.5f
#define PROP_SOUND_PLUNGE_SPEED_MAX  6.0f
#define PROP_SOUND_PLUNGE_VOLUME_MIN 0.1f
#define PROP_SOUND_PLUNGE_VOLUME_MAX 0.4f


static void propSound_collision(const RigidBody *body, float impulse)
{
	float speed = impulse * body->inv_mass;
	if (speed < PROP_SOUND_COLLISION_SPEED_MIN) return;

	float t = (speed - PROP_SOUND_COLLISION_SPEED_MIN)
	        / (PROP_SOUND_COLLISION_SPEED_MAX - PROP_SOUND_COLLISION_SPEED_MIN);
	if (t > 1.0f) t = 1.0f;

	float volume = PROP_SOUND_COLLISION_VOLUME_MIN
	             + t * (PROP_SOUND_COLLISION_VOLUME_MAX - PROP_SOUND_COLLISION_VOLUME_MIN);

	entity3d_playSound(body->owner, PROP_SOUND_COLLISION, NULL, volume);
}

/* The sound is the surface's, the place and the volume are the body's that
   entered. */
static void propSound_waterEntry(const RigidBody *body, const Water *water)
{
	float plunge_speed = -body->linear_velocity.z;

	float t = (plunge_speed - PROP_SOUND_PLUNGE_SPEED_MIN)
	        / (PROP_SOUND_PLUNGE_SPEED_MAX - PROP_SOUND_PLUNGE_SPEED_MIN);
	if (t < 0.0f) t = 0.0f;
	if (t > 1.0f) t = 1.0f;

	float volume = PROP_SOUND_PLUNGE_VOLUME_MIN
	             + t * (PROP_SOUND_PLUNGE_VOLUME_MAX - PROP_SOUND_PLUNGE_VOLUME_MIN);

	/* The body lives in metres; the emitters in render units. */
	Vector3 position = vector3_scaled(&body->tx.position, RENDER_SCALE);

	entity3d_playSound(water->volume.body->owner, PROP_SOUND_WATER_ENTRY, &position, volume);
}

void propSound_update(struct PhysicsWorld *world)
{
	/* The constraint's first colliding frame, tracked by the engine itself,
	   so no state lives here. A solid contact is a hit; a sensor one is a
	   body meeting a water volume. */
	for (const ContactConstraint *c = world->contact_manager.contact_list; c; c = c->next) {
		if (!(c->flags & CONSTRAINT_COLLIDING))    continue;
		if (  c->flags & CONSTRAINT_WAS_COLLIDING) continue;

		if (c->manifold.sensor) {
			const Water *water_a = water_getBoundSurface(c->body_a);
			const Water *water_b = water_getBoundSurface(c->body_b);

			if      (water_a) propSound_waterEntry(c->body_b, water_a);
			else if (water_b) propSound_waterEntry(c->body_a, water_b);
			continue;
		}

		float impulse = 0.0f;
		for (int32_t i = 0; i < c->manifold.contact_count; i++)
			impulse += c->manifold.contacts[i].normal_impulse;

		bool a_hits = (c->body_a->flags & BODY_FLAG_DYNAMIC) && c->body_a->owner;
		bool b_hits = (c->body_b->flags & BODY_FLAG_DYNAMIC) && c->body_b->owner;

		/* When both sides would sound, the heavier body owns the hit: one
		   thud per contact, not two stacked. */
		if (a_hits && b_hits) {
			if (c->body_a->mass >= c->body_b->mass) b_hits = false;
			else                                    a_hits = false;
		}

		if (a_hits) propSound_collision(c->body_a, impulse);
		if (b_hits) propSound_collision(c->body_b, impulse);
	}
}
