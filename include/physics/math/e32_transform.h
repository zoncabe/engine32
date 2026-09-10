/*
	Physics transform: position + rotation matrix. (from qu3e q3Transform).
	The render-side transform (pos+euler+scale) lives in render/e32_render.h
	as RenderTransform.
*/
#ifndef ENGINE_32_TRANSFORM_H
#define ENGINE_32_TRANSFORM_H

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_matrix3.h"
#include "physics/geometry/e32_half_space.h"


struct Transform
{
	Vector3 position;
	Matrix3 rotation;

	static constexpr Transform identity() { return { Vector3::zero(), Matrix3::identity() }; }

	constexpr void init() { *this = identity(); }

	Transform inverse() const;

	/* rotation * v + position */
	Vector3 mulVector(const Vector3 &v) const;
	Vector3 mulVectorScaled(const Vector3 &scale, const Vector3 &v) const;

	/* this * u */
	Transform operator*(const Transform &u) const;

	/* transpose(rotation) * (v - position) */
	Vector3   mulVectorTransposed(const Vector3 &v) const;
	Transform productTransposed(const Transform &u) const;

	HalfSpace mulHalfSpace(const HalfSpace &p) const;
	HalfSpace mulHalfSpaceScaled(const Vector3 &scale, const HalfSpace &p) const;
	HalfSpace mulHalfSpaceTransposed(const HalfSpace &p) const;
};


#endif
