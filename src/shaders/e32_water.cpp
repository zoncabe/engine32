#include <stddef.h>
#include <stdint.h>

#include "psyqo/alloc.h"

#include "animation/e32_model.h"

#include "shaders/e32_water.h"
#include "physics/collision/e32_collision_mesh.h"
#include "physics/world/e32_physics_world.h"
#include "physics/body/e32_rigid_body.h"
#include "physics/math/e32_math_common.h"
#include "physics/math/e32_trig.h"


/* Fresh water, and a drag that settles a bobbing crate in a few swings. */
#define WATER_DEFAULT_DENSITY       1000.0_fp
#define WATER_DEFAULT_LINEAR_DRAG   2.5_fp
#define WATER_DEFAULT_ANGULAR_DRAG  1.5_fp

/* A full turn of the trigonometry's angle: 1.0 is half a turn. */
#define WATER_TURN 2.0_fp


static Water  water_pool[WATER_MAX_SURFACES];
static uint8_t water_count;


Water *water_create(const WaterDef *def)
{
	if (water_count >= WATER_MAX_SURFACES) return NULL;

	CollisionMesh *mesh = CollisionMesh::load(def->mesh_path);
	if (mesh == NULL) return NULL;

	Water *water = &water_pool[water_count];
	*water = {};
	water->def = *def;

	water->count    = mesh->vertex_count;
	water->position = (Vector3 *)psyqo_malloc(sizeof(Vector3) * water->count * 3);
	water->rgba     = (uint8_t *)psyqo_malloc(4 * water->count);
	if (water->position == NULL || water->rgba == NULL) {
		psyqo_free(water->position);
		psyqo_free(water->rgba);
		mesh->free();
		*water = {};
		return NULL;
	}
	water->normal = water->position + water->count;
	water->rest   = water->normal   + water->count;

	/* The mesh is scaffolding, same as the cloth: it seeds the points and
	   nothing keeps a reference to it afterwards. */
	__builtin_memcpy(water->rest, mesh->vertices, sizeof(Vector3) * water->count);
	mesh->free();

	/* An unset tint would paint the water black; white leaves the caustics. */
	if (water->def.color[0] == 0 && water->def.color[1] == 0 && water->def.color[2] == 0)
		water->def.color[0] = water->def.color[1] = water->def.color[2] = 255;

	if (water->def.density      == 0) water->def.density      = WATER_DEFAULT_DENSITY;
	if (water->def.linear_drag  == 0) water->def.linear_drag  = WATER_DEFAULT_LINEAR_DRAG;
	if (water->def.angular_drag == 0) water->def.angular_drag = WATER_DEFAULT_ANGULAR_DRAG;

	/* The plane is authored flat; the average irons out export noise. The
	   sum runs in 64 bits: thousands of heights would overflow the fixed
	   point on their own. */
	int64_t z_sum = 0;
	for (uint16_t i = 0; i < water->count; i++)
		z_sum += water->rest[i].z.raw();
	water->base_z = Fixed((int32_t)(z_sum / water->count), Fixed::RAW);

	for (uint16_t i = 0; i < water->count; i++) {
		water->position[i] = water->rest[i];
		water->normal[i]   = { 0.0_fp, 0.0_fp, 1.0_fp };
		uint8_t *rgba = &water->rgba[i * 4];
		rgba[0] = water->def.color[0];
		rgba[1] = water->def.color[1];
		rgba[2] = water->def.color[2];
		rgba[3] = 0xFF;
	}

	for (uint8_t w = 0; w < water->def.wave_count; w++) {
		WaterWave *wave = &water->def.wave[w];

		Vector3 dir = Vector3{ wave->direction_x, wave->direction_y, 0.0_fp }.normalized();
		wave->direction_x = dir.x;
		wave->direction_y = dir.y;
		water->amplitude_sum += wave->amplitude;

		/* Radians to the table's unit, where pi is 1.0. */
		water->frequency[w] = wave->frequency / PI;
		water->speed[w]     = wave->speed / PI;
		water->phase[w]     = 0;
	}

	water->conf = {};
	water->conf.userData = water;

	water_count++;
	return water;
}

static void water_scroll(Fixed offset[2], const Fixed speed[2], Fixed wrap, Fixed delta)
{
	offset[0] += speed[0] * delta;
	offset[1] += speed[1] * delta;

	/* Folded into [0, wrap): the texture window repeats the texture, so
	   the offset only has to stay a texel count inside it. */
	if (wrap > 0) {
		for (int i = 0; i < 2; i++) {
			while (offset[i] >= wrap) offset[i] -= wrap;
			while (offset[i] < 0)     offset[i] += wrap;
		}
	}
}

static void water_waves(Water *water)
{
	const WaterDef *def = &water->def;

	for (uint16_t i = 0; i < water->count; i++) {
		const Vector3 *rest = &water->rest[i];
		Fixed height  = 0;
		Fixed slope_x = 0;
		Fixed slope_y = 0;

		for (uint8_t w = 0; w < def->wave_count; w++) {
			const WaterWave *wave = &def->wave[w];

			Fixed phase = (wave->direction_x * rest->x + wave->direction_y * rest->y)
			            * water->frequency[w] + water->phase[w];

			Fixed s, c;
			Trig::sincos(Angle(phase), &s, &c);

			height  += wave->amplitude * s;
			slope_x += wave->amplitude * wave->frequency * wave->direction_x * c;
			slope_y += wave->amplitude * wave->frequency * wave->direction_y * c;
		}

		water->position[i].z = rest->z + height;

		/* Normal of z = h(x,y) is (-dh/dx, -dh/dy, 1). */
		Vector3 normal = { -slope_x, -slope_y, 1.0_fp };
		water->normal[i] = normal.normalized();

		/* Crests lighter, troughs darker. */
		Fixed bright = 0.75_fp;
		if (water->amplitude_sum > 0)
			bright += (height / water->amplitude_sum) * 0.25_fp;

		uint8_t *rgba = &water->rgba[i * 4];
		rgba[0] = (uint8_t)(bright * water->def.color[0]).integer();
		rgba[1] = (uint8_t)(bright * water->def.color[1]).integer();
		rgba[2] = (uint8_t)(bright * water->def.color[2]).integer();
	}
}

void water_update(Fixed delta)
{
	for (uint8_t i = 0; i < water_count; i++) {
		Water *water = &water_pool[i];

		for (uint8_t w = 0; w < water->def.wave_count; w++) {
			water->phase[w] += water->speed[w] * delta;
			while (water->phase[w] >= WATER_TURN) water->phase[w] -= WATER_TURN;
		}

		water_scroll(water->offset_a, water->def.scroll_a, water->def.wrap_a, delta);
		water_scroll(water->offset_b, water->def.scroll_b, water->def.wrap_b, delta);

		water->conf.scroll_u      = (uint8_t)water->offset_a[0].integer();
		water->conf.scroll_v      = (uint8_t)water->offset_a[1].integer();
		water->conf.glow_scroll_u = (uint8_t)water->offset_b[0].integer();
		water->conf.glow_scroll_v = (uint8_t)water->offset_b[1].integer();

		if (water->culled && *water->culled) continue;

		water_waves(water);
	}
}

/* Same sum as water_waves, at one arbitrary point instead of the mesh's:
   the buoyancy samples ask here, so a body floats on the exact surface the
   player sees. Waves ride on the rest height, which lives in mesh space;
   the cached placement pulls the world query in and lifts the result out. */
Fixed water_getSurfaceHeight(const Water *water, Fixed x, Fixed y)
{
	const WaterDef *def = &water->def;
	Fixed local_x = x - water->placement.x;
	Fixed local_y = y - water->placement.y;
	Fixed height  = water->placement.z + water->base_z;

	for (uint8_t w = 0; w < def->wave_count; w++) {
		const WaterWave *wave = &def->wave[w];

		Fixed phase = (wave->direction_x * local_x + wave->direction_y * local_y)
		            * water->frequency[w] + water->phase[w];
		height += wave->amplitude * Trig::sin(Angle(phase));
	}

	return height;
}

static Fixed water_volumeSurfaceHeight(const void *surface, Fixed x, Fixed y)
{
	return water_getSurfaceHeight((const Water *)surface, x, y);
}

void water_bindPhysics(Water *water, RigidBody *body, PhysicsWorld *world)
{
	/* The body is static: its placement is settled for good at bind time. */
	water->placement = body->getTransform().position;

	water->volume.body           = body;
	water->volume.density        = water->def.density;
	water->volume.linear_drag    = water->def.linear_drag;
	water->volume.angular_drag   = water->def.angular_drag;
	water->volume.surface_height = water_volumeSurfaceHeight;
	water->volume.surface        = water;

	world->addBuoyancy(&water->volume);
}

Water *water_getBoundSurface(const RigidBody *body)
{
	for (uint8_t i = 0; i < water_count; i++)
		if (water_pool[i].volume.body == body) return &water_pool[i];
	return NULL;
}

void water_clear(void)
{
	for (uint8_t i = 0; i < water_count; i++) {
		psyqo_free(water_pool[i].position);
		psyqo_free(water_pool[i].rgba);
		water_pool[i] = {};
	}
	water_count = 0;
}
