/*
	The pool that fills the sunken quad. Two halves that have to agree: the
	sensor is the volume buoyancy reads, from the basin floor up to the
	resting surface, and the WaterDef is the surface itself, waves included.

	The mesh the waves run on is the welded collision mesh of the same plane,
	not the model: the model is what gets drawn, deformed from those points.
*/
#include "prefab/e32_prefab3d.h"


static const PhysicsShapeDef water_shapes[] = {
	{ .type = SHAPE_BOX, .box = {
		.tx     = { .position = { 0.0_fp, 0.0_fp, -1.25_fp } },
		.e      = { 5.0_fp, 5.0_fp, 1.25_fp },
		.sensor = SENSOR_VOLUME,
	}},
};

static const Entity3DColliderDef water_collider = { water_shapes, 1 };

/* Three sines with unrelated frequencies: one alone reads as a machine. The
   two texture layers, the material's texture and its emission map, scroll
   against each other, which is what sells the caustics. Density and drag
   left at zero take the defaults. */
static const WaterDef water_surface = {

	.mesh_path = "rom:/collision/water.collision",

	.wave = {
		{ .direction_x =  1.0_fp, .direction_y =  0.3_fp, .amplitude = 0.05_fp, .frequency = 1.6_fp, .speed = 1.6_fp },
		{ .direction_x = -0.4_fp, .direction_y =  1.0_fp, .amplitude = 0.03_fp, .frequency = 2.9_fp, .speed = 2.3_fp },
		{ .direction_x =  0.6_fp, .direction_y = -1.0_fp, .amplitude = 0.02_fp, .frequency = 4.3_fp, .speed = 3.1_fp },
	},
	.wave_count = 3,

	.scroll_a = {  1.5_fp, 2.4_fp },
	.scroll_b = { -2.0_fp, 1.0_fp },
	.wrap_a   = 64.0_fp,
	.wrap_b   = 64.0_fp,

	.color = { 89, 166, 204 },
};

extern const Prefab3D water = {

	.type     = PREFAB3D_WATER,
	.model    = "rom:/models/water.model",
	.collider = &water_collider,
	.water    = &water_surface,
};
