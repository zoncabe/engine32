#ifndef ENGINE_32_SOUND_H
#define ENGINE_32_SOUND_H

#include <stdbool.h>
#include <stdint.h>
#include "physics/math/e32_vector3.h"

/* The sample the backend keeps. Opaque here: the module that fills it
   is the console's own. */
struct wae32_t;

#define SOUND_MAX_EMITTERS 12

/* Mixer channels the emitters draw from. A stereo sample takes two of them. */
#define SOUND_MIXER_CHANNELS 16

#define SOUND_PRIORITY_ONESHOT  64
#define SOUND_PRIORITY_AMBIENCE 128

/* Returned by sound_play; SOUND_NO_EMITTER when the bank or the mixer had
   nothing left to give. */
typedef int SoundEmitter;
#define SOUND_NO_EMITTER (-1)


/* What a sound is: its file and how it plays. Const data, declared by the
   game next to whatever uses it. Nothing here changes at runtime.

   Every sound is positional: the two radii below are what decide its volume,
   and its own volume only scales the result. A sound meant to play flat sets
   max_distance to 0. */
typedef struct SoundDef {

	const char *path;

	/* Own volume of the sample, in [0..1]. Multiplies the distance gain. */
	float volume;

	/* Inside min_distance the sound plays at its own volume; past
	   max_distance it is silent and gives its mixer channels back. */
	float min_distance;
	float max_distance;

	bool loop;

	/* What fires this sound. A number the game defines; the engine only
	   matches it. Several sounds of one prefab may share a trigger: one of
	   them is picked at random when it fires. Ignored for loops. */
	uint8_t trigger;

	/* Voice stealing weight: ambience outlives one-shots. */
	uint8_t priority;

	/* Short samples are decoded into RAM once, so firing one costs no DMA.
	   Long ones stream from ROM, which is the only thing that fits. */
	bool preload;

} SoundDef;


/* A def with its file open: what whoever plays it keeps, from load to
   unload, and the only thing the emitters play. The file itself lives in
   the resource table, shared with whoever else loads the same path. */
typedef struct Sound {

	const SoundDef *def;
	wae32_t        *wave;

} Sound;


/* Brings the mixer up. Runs once, before any scene loads; nothing is
   opened here. */
void sound_init(void);
void sound_close(void);

/* Opens a def's file through the resource table. Whoever loads a sound
   unloads it; unloading cuts whatever emitter was still playing it. */
Sound sound_load(const SoundDef *def);
void  sound_unload(Sound *sound);

/* Where the world is heard from. Position decides attenuation, right decides
   panning: the caller is free to take them from different places. */
void sound_setListener(const Vector3 *position, const Vector3 *right);

/* Where sound_update puts the ear each frame. On the player, the sound
   sticks to the driven body while the camera floats on its arm; on the
   camera, what is heard is what is seen from. Panning always follows the
   camera. Default: the player. */
typedef enum {

	SOUND_LISTENER_PLAYER,
	SOUND_LISTENER_CAMERA,

} SoundListenerMode;

void sound_setListenerMode(SoundListenerMode mode);

/* Runs from the game loop, in every state: the mixer has to be polled whether
   or not a scene is loaded. */
void sound_update(void);

/* Feeds the mixer without touching the emitters. A single call per frame runs
   dry whenever a frame stretches, so this goes around the expensive parts of
   the loop as well. */
void sound_poll(void);

/* Starts the sound at a point in the world. Looping sounds hold their emitter
   until sound_stop; one-shots release it when the sample ends. The sound has
   to outlive its emitters: unloading it cuts them.

   volume_scale scales the sound's own volume, for a noise that is the same
   sample at different strengths. duration asks the sample to last that many
   seconds, slowing it down as much as that takes; 0 plays it at its
   own speed. */
SoundEmitter sound_play(const Sound *sound, const Vector3 *position, float volume_scale, float duration);
void sound_stop(SoundEmitter emitter);
void sound_stopAll(void);

/* For emitters that follow something that moves. */
void sound_setEmitterPosition(SoundEmitter emitter, const Vector3 *position);

#endif
