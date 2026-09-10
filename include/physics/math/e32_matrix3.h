#ifndef ENGINE_32_MATRIX3_H
#define ENGINE_32_MATRIX3_H

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_trig.h"


/* Pitch, yaw and roll as read back from a rotation matrix. */
struct EulerAngles
{
	Angle x;
	Angle y;
	Angle z;
};


/* Column-major: ex, ey and ez are the columns. */
struct Matrix3
{
	Vector3 ex;
	Vector3 ey;
	Vector3 ez;

	/* Nine stores each, inline: as a call the 36-byte result would be
	   written once by the callee and copied again by the caller. */

	static constexpr Matrix3 zero() { return {}; }

	static constexpr Matrix3 identity()
	{
		return {
			{1.0_fp, 0.0_fp, 0.0_fp},
			{0.0_fp, 1.0_fp, 0.0_fp},
			{0.0_fp, 0.0_fp, 1.0_fp},
		};
	}

	static constexpr Matrix3 diagonal(Fixed x, Fixed y, Fixed z)
	{
		return {
			{x,      0.0_fp, 0.0_fp},
			{0.0_fp, y,      0.0_fp},
			{0.0_fp, 0.0_fp, z     },
		};
	}

	static constexpr Matrix3 fromColumns(const Vector3 &ex, const Vector3 &ey, const Vector3 &ez)
	{
		return {ex, ey, ez};
	}

	static Matrix3 fromAxisAngle(const Vector3 &axis, Angle angle);
	static Matrix3 fromEuler(Angle pitch, Angle yaw, Angle roll);
	static Matrix3 outerProduct(const Vector3 &u, const Vector3 &v);

	constexpr Matrix3 transposed() const
	{
		return {
			{ex.x, ey.x, ez.x},
			{ex.y, ey.y, ez.y},
			{ex.z, ey.z, ez.z},
		};
	}

	Matrix3 inverse() const;
	EulerAngles toEuler() const;

	constexpr void transpose() { *this = transposed(); }

	constexpr Matrix3 operator+(const Matrix3 &m) const { return {ex + m.ex, ey + m.ey, ez + m.ez}; }
	constexpr Matrix3 operator-(const Matrix3 &m) const { return {ex - m.ex, ey - m.ey, ez - m.ez}; }
	constexpr Matrix3 operator*(Fixed s) const          { return {ex * s, ey * s, ez * s}; }

	constexpr void operator+=(const Matrix3 &m) { ex += m.ex; ey += m.ey; ez += m.ez; }
	constexpr void operator-=(const Matrix3 &m) { ex -= m.ex; ey -= m.ey; ez -= m.ez; }
	constexpr void operator*=(Fixed s)          { ex *= s;    ey *= s;    ez *= s;    }

	/* this * v */
	constexpr Vector3 transform(const Vector3 &v) const
	{
		return {
			ex.x * v.x + ey.x * v.y + ez.x * v.z,
			ex.y * v.x + ey.y * v.y + ez.y * v.z,
			ex.z * v.x + ey.z * v.y + ez.z * v.z,
		};
	}

	/* transpose(this) * v */
	constexpr Vector3 transformTransposed(const Vector3 &v) const
	{
		return {ex.dot(v), ey.dot(v), ez.dot(v)};
	}

	/* this * m */
	constexpr Matrix3 operator*(const Matrix3 &m) const
	{
		return {transform(m.ex), transform(m.ey), transform(m.ez)};
	}

	/* Element access is inline: a couple of loads, smaller than a call. With
	   literal indices, as the box-box solver uses them, get folds to a
	   single load. */

	constexpr Vector3 column0() const { return {ex.x, ey.x, ez.x}; }
	constexpr Vector3 column1() const { return {ex.y, ey.y, ez.y}; }
	constexpr Vector3 column2() const { return {ex.z, ey.z, ez.z}; }

	constexpr Fixed get(int i, int j) const
	{
		const Vector3 &col = (i == 0) ? ex : (i == 1) ? ey : ez;
		return col[j];
	}
};


#endif
