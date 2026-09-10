#ifndef ENGINE_32_CHARACTER3D_SOUND_H
#define ENGINE_32_CHARACTER3D_SOUND_H

#include <stdbool.h>
#include <stdint.h>

#include "sound/e32_sound.h"

typedef struct Character3D Character3D;

#define CHARACTER3D_SOUND_MAX_VARIATIONS 8
#define CHARACTER3D_SOUND_MAX_FOOTINGS   4


/* Indices into the entity's sound list, the way ANIMATION_CLIPS names clips. */
#define SOUND_CLIPS(...) ((const uint8_t[]){ __VA_ARGS__ })

/* What a character sounds like. Sits in its def next to the animation and
   movement settings: two bodies can walk the same graph and step on different
   samples. The samples themselves are the prefab's list, open in the
   entity; every event below names them by index into it. */
typedef struct Character3DSoundDef {

	/* One is picked at random per step, so the same noise does not repeat. */
	const uint8_t *footstep;
	uint8_t footstep_count;

	/* Points of the locomotion clip where a foot lands, as normalized time.
	   They belong to the clip, not to the sound: a gait that lands its feet
	   elsewhere moves them. */
	const float *footing;
	uint8_t footing_count;

	/* A step at walking pace should not hit as hard as one at a sprint. */
	float footstep_volume_min;
	float footstep_volume_max;
	float footstep_speed_max;

	const uint8_t *roll;
	uint8_t roll_count;
	float roll_volume;

	/* Seconds to wait past roll_ground_time. The movement settings say when
	   the body reaches the floor; this says how far into that the sample
	   wants to come in. */
	float roll_delay;

	/* Feet on either end of the roll: the one that pushes off on entry, and
	   the one planted at roll_grip_time to come out of it. */
	float roll_launch_volume;
	float roll_stand_volume;

	/* Seconds a footstep has to be behind for the roll's launch foot to fire:
	   pushing off right after a step would flam two samples into one. */
	float roll_launch_gap;

	/* Fires when the crouch ends and the body leaves the floor. */
	const uint8_t *jump;
	uint8_t jump_count;
	float jump_volume;

	/* Same guard for the jump takeoff, tuned on its own. */
	float jump_launch_gap;

	/* Fires on touchdown, scaled by how fast the body was falling. */
	const uint8_t *land;
	uint8_t land_count;
	float land_volume_min;
	float land_volume_max;
	float land_speed_max;

	/* Swim strokes fire on the stroke phases of the swim cycle; the pace
	   picks the set, calm strokes against sprint ones. */
	const uint8_t *swim_stroke_light;
	uint8_t swim_stroke_light_count;
	const uint8_t *swim_stroke_heavy;
	uint8_t swim_stroke_heavy_count;

	/* Points of the swim clips where an arm pulls, like the footings. */
	const float *stroke;
	uint8_t stroke_count;
	float stroke_volume;

	/* Fires entering the water, scaled by the plunge speed. The minimum sits
	   near zero: wading in from the ramp barely whispers, a jump slaps. */
	const uint8_t *splash;
	uint8_t splash_count;
	float splash_volume_min;
	float splash_volume_max;
	float splash_speed_max;

} Character3DSoundDef;


/* Edge detection state. Nothing here is worth saving. */
typedef struct Character3DSound {

	const Character3DSoundDef *def;

	/* Negative until the first update: with no frame behind it, every mark
	   of the cycle would read as just crossed. */
	float previous_cycle;

	float previous_roll_timer;

	/* The collision zeroes the fall before this system runs, so the speed of
	   the impact is the one the previous frame carried. */
	float previous_fall_speed;
	bool previous_grounded;

	/* Same scheme for the splash: the water damps the fall on the entry
	   frame, so the plunge speed is the last dry frame's. */
	float previous_plunge_speed;
	bool previous_in_water;

	/* Clock reading of the last footstep, for whoever has to keep its distance
	   from one. */
	float last_footstep;

} Character3DSound;


void character3dSound_update(Character3D *character);

#endif
