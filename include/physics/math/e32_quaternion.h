#ifndef ENGINE_32_QUATERNION_H
#define ENGINE_32_QUATERNION_H

#include <stdint.h>

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_matrix3.h"


struct Quaternion
{
	Fixed x, y, z, w;

	static constexpr Quaternion identity() { return {0.0_fp, 0.0_fp, 0.0_fp, 1.0_fp}; }

	static Quaternion fromAxisAngle(const Vector3 &axis, Angle angle);
	static Quaternion fromMatrix3(const Matrix3 &m);

	/* Decompression of the smallest-three quantized quaternion the model
	   animation files store. */
	static Quaternion unpacked(uint16_t dataHi, uint16_t dataLo);

	void setAxisAngle(const Vector3 &axis, Angle angle);
	void toAxisAngle(Vector3 *axis, Angle *angle) const;

	/* this * q */
	constexpr Quaternion operator*(const Quaternion &q) const
	{
		return {
			w * q.x + x * q.w + y * q.z - z * q.y,
			w * q.y + y * q.w + z * q.x - x * q.z,
			w * q.z + z * q.w + x * q.y - y * q.x,
			w * q.w - x * q.x - y * q.y - z * q.z,
		};
	}

	Quaternion normalized() const;

	/* Shortest-path component lerp, renormalized. */
	Quaternion nlerp(const Quaternion &b, Fixed t) const;

	/* One step of angular velocity over dt, renormalized. */
	void integrate(const Vector3 &omega, Fixed dt);

	/* Rotates a vector by this unit quaternion. */
	Vector3 rotate(const Vector3 &v) const;

	Matrix3 toMatrix3() const;

	/* x, y, z and w are contiguous */
	constexpr Fixed       &operator[](int i)       { return (&x)[i]; }
	constexpr const Fixed &operator[](int i) const { return (&x)[i]; }
};


#endif
