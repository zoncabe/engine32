#include <stdint.h>

#include "psyqo/gte-registers.hh"

#include "scene3d/e32_fog.h"
#include "graphics/e32_model_format.h"


static Fog fog;

Fog *FogState::get() { return &fog; }


void FogState::init(const FogDef *def)
{
	fog.color   = def->color;
	fog.near    = def->near;
	fog.far     = def->far;
	fog.enabled = def->enabled;
}

/* The GTE computes, per projected vertex, the depth cue as
   MAC0 = p * DQA + DQB, with p the perspective factor H * 65536 / SZ, and
   IR0 = MAC0 / 4096 clamped to 0..4096: the share of the far color. */
void FogState::set(const Fog *fog, int projection)
{
	using namespace psyqo::GTE;

	/* Far color: the fog's, in 4.12 where 255 is nearly 1.0 (the far color
	   registers take the value times 16). */
	write<Register::RFC, Unsafe>((uint32_t)fog->color.r << 4);
	write<Register::GFC, Unsafe>((uint32_t)fog->color.g << 4);
	write<Register::BFC, Unsafe>((uint32_t)fog->color.b << 4);

	if (!fog->enabled || fog->far <= fog->near) {
		clear<Register::DQA, Unsafe>();
		clear<Register::DQB, Safe>();
		return;
	}

	/* Depths in the GTE's units. */
	int32_t near = fog->near.raw() >> MODEL_CPU_TO_UNITS;
	int32_t far  = fog->far.raw()  >> MODEL_CPU_TO_UNITS;
	if (near < 1) near = 1;
	if (far <= near) far = near + 1;

	/* p at both planes, 16.16 */
	int64_t p_near = ((int64_t)projection << 16) / near;
	int64_t p_far  = ((int64_t)projection << 16) / far;

	/* MAC0 is 0 at near and 4096 * 4096 at far. */
	int64_t dqa = ((int64_t)1 << 24) / (p_far - p_near);
	int64_t dqb = -p_near * dqa;

	write<Register::DQA, Unsafe>((uint32_t)(int32_t)dqa);
	write<Register::DQB, Safe>((uint32_t)(int32_t)dqb);
}
