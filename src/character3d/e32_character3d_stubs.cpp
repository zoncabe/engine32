/*
	Stand-ins for the character modules the port has not reached yet:
	animation, weapons, aiming, spring bones, sound and stats. The headers
	are the real ones, so Character3D keeps its shape; only the functions
	the core calls are here, doing nothing. A body driven through these
	moves, collides, jumps, swims and climbs without a skeleton, drawn the
	way a prop is.

	This file leaves the build as each real module is ported in its place.
*/
#include <stddef.h>

#include "character3d/e32_character3d.h"


/* --- animation ------------------------------------------------------------- */

void character3dAnimation_initGraph(Character3D *character, const Character3DAnimationDef *def)
{
	(void)character; (void)def;
}

void character3d_setAnimation(Character3D *character)
{
	(void)character;
}


/* --- weapons --------------------------------------------------------------- */

void character3d_equipWeapon(Character3D *character, uint8_t slot, const WeaponDef *weapon)
{
	(void)character; (void)slot; (void)weapon;
}

void character3d_cycleWeapon(Character3D *character, int8_t dir)
{
	(void)character; (void)dir;
}

const WeaponDef *character3d_drawnWeapon(const Character3D *character)
{
	(void)character;
	return NULL;
}

void character3dWeapon_setBones(Character3D *character)
{
	(void)character;
}


/* --- aiming ---------------------------------------------------------------- */

void character3dAim_init(Character3D *character, const Character3DAimingSettings *settings)
{
	(void)character; (void)settings;
}

void character3dAim_apply(Armature *skeleton, void *context)
{
	(void)skeleton; (void)context;
}


/* --- spring bones ---------------------------------------------------------- */

uint8_t springBones_resolveChain(const Armature *skeleton, const SpringBonesDef *def,
                                 int16_t *joints, uint8_t max)
{
	(void)skeleton; (void)def; (void)joints; (void)max;
	return 0;
}

bool springBone_init(SpringBone *spring_bone, const Armature *skeleton, int16_t bone,
                     uint8_t joint_index, const SpringBonesDef *def, const RenderTransform *world)
{
	(void)spring_bone; (void)skeleton; (void)bone; (void)joint_index; (void)def; (void)world;
	return false;
}

void springBone_apply(Armature *skeleton, void *context)
{
	(void)skeleton; (void)context;
}


/* --- skeleton modifiers ---------------------------------------------------- */

void skeletonModifiers_add(SkeletonModifiers *modifiers, SkeletonModifierFn apply, void *context)
{
	(void)modifiers; (void)apply; (void)context;
}

void skeletonModifiers_apply(SkeletonModifiers *modifiers, Armature *skeleton)
{
	(void)modifiers; (void)skeleton;
}


/* --- sound ----------------------------------------------------------------- */

void character3dSound_update(Character3D *character)
{
	(void)character;
}


/* --- stats ----------------------------------------------------------------- */

/* The real one writes the command's speed scale every frame, tired or not:
   left at zero the body would never move. */
void character3dStats_update(Character3D *character, MovementCommand *cmd, Fixed dt)
{
	(void)character; (void)dt;
	cmd->speed_scale = 1.0_fp;
}
