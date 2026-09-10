/*
	Procedural water surface.

	Drives a subdivided plane through meshDeform, the same path the cloth
	uses: a welded collision mesh of the plane seeds the points, a sum of
	sine waves moves them, and analytic normals from the same sum keep the
	shading and any generated UVs alive.

	The two texture layers scroll through the material's tile settings, so
	the mesh must be drawn through the per-frame material path (recorded
	objects), never a fully recorded display list.
*/
#ifndef ENGINE_32_WATER_H
#define ENGINE_32_WATER_H

#include <stdint.h>
#include "animation/e32_model.h"
#include "graphics/e32_mesh.h"

#include "physics/math/e32_vector3.h"
#include "physics/buoyancy/e32_buoyancy.h"

#define WATER_MAX_WAVES    3
#define WATER_MAX_SURFACES 2


struct RigidBody;
struct PhysicsWorld;


/* One directional sine. Direction is in the horizontal plane and gets
   normalized at creation; amplitude is in metres, like the cloth. A single
   sine reads as a machine, so stack two or three with unrelated frequencies. */
struct WaterWave
{
	Fixed direction_x;
	Fixed direction_y;
	Fixed amplitude;    /* metres */
	Fixed frequency;    /* radians per metre along the direction */
	Fixed speed;        /* radians per second */
};

/* Authoring side. The particles are seeded from a welded collision mesh of
   the same plane, exactly like the cloth, so the topology comes from the
   asset. Scroll speeds are in texels per second; wrap is the texture size in
   texels, so the offset can fold instead of growing without bound. The
   first layer is the material's texture, the second its emission map. */
struct WaterDef
{
	const char *mesh_path;

	WaterWave wave[WATER_MAX_WAVES];
	uint8_t   wave_count;

	Fixed scroll_a[2];   /* base texture, u/v texels per second */
	Fixed scroll_b[2];   /* emission map */
	Fixed wrap_a;        /* texture size of the base texture, texels */
	Fixed wrap_b;        /* texture size of the emission map */

	/* Water tint, written as vertex color and scaled by the wave height:
	   crests lighter, troughs darker. The GPU multiplies it into the
	   texture. */
	uint8_t color[3];

	/* Buoyancy, fed to the physics world when the surface is bound to a
	   sensor body. Zero means the default: fresh water and a mild drag. */
	Fixed density;        /* kg/m3 */
	Fixed linear_drag;    /* per-second rate on the linear velocity */
	Fixed angular_drag;   /* per-second rate on the angular velocity */
};


struct Water
{
	Vector3 *position;   /* animated points, metres; the mesh reads these */
	Vector3 *normal;     /* analytic, from the wave derivatives */
	Vector3 *rest;       /* the flat pose the waves displace from */
	uint8_t *rgba;       /* tint shaded by wave height, 4 per point */
	uint16_t count;

	Fixed amplitude_sum; /* of every wave; normalises the height for shading */
	Fixed base_z;        /* resting surface height, metres, mesh space */

	/* Placement of the bound sensor body, cached at bind time: the model is
	   authored in local space, the physics asks in world space. Zero until
	   the water is bound. Translation only: buoyancy models the surface as
	   z = h(x, y) under vertical gravity, so a rotated placement is out of
	   the model regardless. */
	Vector3 placement;

	BuoyancyVolume volume;   /* lent to the physics world by water_bindPhysics */

	/* The waves in the trigonometry's own unit, a half turn per 1.0: the
	   def's radians over pi. The phase advances per wave and wraps every
	   full turn, where a running clock would overflow. */
	Fixed frequency[WATER_MAX_WAVES];   /* per metre */
	Fixed speed[WATER_MAX_WAVES];       /* per second */
	Fixed phase[WATER_MAX_WAVES];

	Fixed offset_a[2];   /* accumulated scroll, texels */
	Fixed offset_b[2];

	const bool *culled;  /* the mesh's flag; waves are skipped out of view */

	WaterDef def;

	/* Handed to the render path, which reads the scroll off it. */
	ModelDrawConf conf;
};


/* Loads the collision mesh, seeds the points at rest and registers the
   surface for update. NULL if the pool is full or the mesh did not load. */
Water *water_create(const WaterDef *def);

/* Advances every registered surface. Time always moves, so a culled pool
   does not freeze mid-wave; only the per-point work is skipped. */
void water_update(Fixed delta);

/* Height of the surface over any world (x, y), metres: the query drops into
   mesh space through the bound body's placement, rides the same wave sum the
   render points use, and comes back out with the placement's height. */
Fixed water_getSurfaceHeight(const Water *water, Fixed x, Fixed y);

/* Binds the surface to the static body carrying the water's sensor shape
   and registers the pair as a buoyancy volume in the world. From then on
   every dynamic body inside the sensor floats against these waves. */
void water_bindPhysics(Water *water, RigidBody *body, PhysicsWorld *world);

/* The surface water_bindPhysics tied to this body. NULL: the body carries no
   water. */
Water *water_getBoundSurface(const RigidBody *body);

/* Deletes every registered surface. Runs with scene3d_unload. */
void water_clear(void);

#endif
