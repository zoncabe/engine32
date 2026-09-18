#include <stdint.h>

#include "psyqo/gte-registers.hh"
#include "psyqo/matrix.hh"

#include "physics/math/e32_math.h"
#include "scene3d/e32_lighting.h"


static Light light;

Light *Lighting::get() { return &light; }


void Lighting::init(const LightDef *def)
{
	light = *def;

	for (int i = 0; i < LIGHT_COUNT; i++) {
		if (light.source[i].type == LIGHT_NONE) break;
		if (light.source[i].type == LIGHT_DIRECTIONAL)
			light.source[i].directional.direction.normalize();
	}
}

/* A color byte as the GTE's 4.12 intensity: 255 is very nearly 1.0. */
static Fixed intensity(uint8_t channel)
{
	return Fixed((int32_t)channel << 4, Fixed::RAW);
}

void Lighting::setAmbient(const Light *light)
{
	psyqo::GTE::write<psyqo::GTE::Register::RBK, psyqo::GTE::Unsafe>((uint32_t)intensity(light->ambient_color.r).raw());
	psyqo::GTE::write<psyqo::GTE::Register::GBK, psyqo::GTE::Unsafe>((uint32_t)intensity(light->ambient_color.g).raw());
	psyqo::GTE::write<psyqo::GTE::Register::BBK, psyqo::GTE::Safe>((uint32_t)intensity(light->ambient_color.b).raw());
}

bool Lighting::hasPositional(const Light *light)
{
	for (int i = 0; i < LIGHT_COUNT; i++) {
		if (light->source[i].type == LIGHT_NONE) break;
		if (light->source[i].type == LIGHT_POINT) return true;
	}
	return false;
}

void Lighting::set(const Light *light, const Transform *model)
{
	setAt(light, model, Vector3::zero());
}

void Lighting::setAt(const Light *light, const Transform *model, const Vector3 &local_point)
{
	const Vector3 from = model->mulVector(local_point);

	psyqo::Matrix33 directions = {};
	psyqo::Matrix33 colors     = {};

	int count = 0;
	for (; count < LIGHT_COUNT; count++) {
		const LightSource *source = &light->source[count];
		if (source->type == LIGHT_NONE) break;

		Vector3 direction;
		Fixed   factor = 1.0_fp;

		if (source->type == LIGHT_DIRECTIONAL) {
			direction = source->directional.direction;
		} else {
			/* From the object to the light, fading linearly to nothing at
			   the light's size. */
			direction = source->point.position - from;
			Fixed distance = direction.magnitude();
			if (distance == 0) continue;
			direction *= 1 / distance;
			if (source->point.size > 0)
				factor = clamp(1 - distance / source->point.size, 0.0_fp, 1.0_fp);
		}

		/* Into model space through the transpose of the rotation, then back
		   to unit length: the model's scale rides in that rotation. */
		Vector3 local = model->rotation.transformTransposed(direction);
		local.normalize();

		directions.vs[count] = {local.x, local.y, local.z};

		colors.vs[0][count] = intensity(source->color.r) * factor;
		colors.vs[1][count] = intensity(source->color.g) * factor;
		colors.vs[2][count] = intensity(source->color.b) * factor;
	}

	psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::Light>(directions);
	psyqo::GTE::writeSafe<psyqo::GTE::PseudoRegister::Color>(colors);
}
