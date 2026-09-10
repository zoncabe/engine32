#ifndef ENGINE_32_FRUSTUM_H
#define ENGINE_32_FRUSTUM_H

#include <stdint.h>

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_matrix3.h"


/* A plane as normal and distance: inside is where normal . p + d >= 0. */
struct FrustumPlane
{
	Vector3 normal;
	Fixed   distance;
};

/* View frustum as six planes, in whatever space it was built or moved to. */
struct Frustum
{
	FrustumPlane planes[6];   /* left, right, bottom, top, near, far */

	/* From the view the GTE draws with: the rotation whose rows are right,
	   down and forward, the eye position, the projection distance in pixels
	   and the two depth planes in meters. The result is in world space. */
	static Frustum fromView(const Matrix3 &rotation, const Vector3 &position,
	                        int projection, int half_width, int half_height,
	                        Fixed near, Fixed far);

	/* The same frustum seen from a model placed with this rotation, position
	   and scale: model space boxes test against it as they are. */
	Frustum toModel(const Matrix3 &rotation, const Vector3 &position, const Vector3 &scale) const;

	void scale(Fixed factor);

	bool vsAabb(const Vector3 &min, const Vector3 &max) const;

	/* A box in the vertices' units (MODEL_UNITS_PER_METER). */
	bool vsAabbS16(const int16_t min[3], const int16_t max[3]) const;

	bool vsSphere(const Vector3 &center, Fixed radius) const;
};


#endif
