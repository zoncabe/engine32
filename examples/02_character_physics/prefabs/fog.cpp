/*
	Distance haze. The scene only says from where and until where: nothing
	fades before 15 metres and everything is background colour past 45.
*/
#include "scene3d/e32_fog.h"


extern const FogDef fog = {

	.color   = {{70, 80, 100}},
	.near    = 15.0_fp,
	.far     = 45.0_fp,
	.enabled = true,
};
