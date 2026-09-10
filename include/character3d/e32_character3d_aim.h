/*
	Camera-pitch aim bend, split evenly along a spine chain.

	The delta is never added in a bone's own frame: the aiming pose keeps the
	torso half-turned at whatever angle the clip authored, so a local axis is
	tilted with it. The bend is built once about the world's horizontal side
	axis and conjugated into each bone's frame through the rotation the chain
	actually carries this frame, which keeps it pure pitch under any twist.
*/
#ifndef ENGINE_32_CHARACTER3D_AIM_H
#define ENGINE_32_CHARACTER3D_AIM_H

#include <stdint.h>
#include "animation/e32_armature.h"

typedef struct Character3D Character3D;
typedef struct Camera    Camera;

#define CHARACTER3D_AIM_MAX_BONES 4

typedef struct Character3DAimingSettings {

	const char *const *bone;   /* spine chain, root to tip */
	uint8_t count;
	float pitch_scale;         /* spine degrees per camera degree, sign included */

} Character3DAimingSettings;

/* Resolved once at create: the names above become indices so the bend never
   searches the skeleton by string, and the scale rides along so the bend has
   everything it needs without reaching back into the def. */
typedef struct Character3DAiming {

	int16_t bone[CHARACTER3D_AIM_MAX_BONES];
	uint8_t count;
	float   pitch_scale;

} Character3DAiming;


void character3dAim_init(Character3D *character, const Character3DAimingSettings *settings);

/* SkeletonModifierFn; context is the Character3D. Weighted by the aim blend,
   so the torso straightens on its own when the mode fades. */
void character3dAim_apply(Armature *skeleton, void *context);

#endif
