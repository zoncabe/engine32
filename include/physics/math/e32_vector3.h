#ifndef ENGINE_32_VECTOR3_H
#define ENGINE_32_VECTOR3_H

#include "physics/math/e32_fixed.h"


/* The arithmetic is inline on purpose: each body is a handful of integer
   ops, smaller than the call sequence it replaces. The three that take a
   square root stay real functions in the .cpp: their body is a call plus a
   branch, and copying that per call site only grows the code. */

struct Vector3
{
	Fixed x;
	Fixed y;
	Fixed z;

	static constexpr Vector3 zero() { return {}; }

	constexpr Vector3 operator+(const Vector3 &v) const { return {x + v.x, y + v.y, z + v.z}; }
	constexpr Vector3 operator-(const Vector3 &v) const { return {x - v.x, y - v.y, z - v.z}; }
	constexpr Vector3 operator-() const                 { return {-x, -y, -z}; }
	constexpr Vector3 operator*(Fixed s) const          { return {x * s, y * s, z * s}; }

	constexpr void operator+=(const Vector3 &v) { x += v.x; y += v.y; z += v.z; }
	constexpr void operator-=(const Vector3 &v) { x -= v.x; y -= v.y; z -= v.z; }
	constexpr void operator*=(Fixed s)          { x *= s;   y *= s;   z *= s;   }

	/* this += v * s, without the temporary */
	constexpr void addScaledVector(const Vector3 &v, Fixed s) { x += v.x * s; y += v.y * s; z += v.z * s; }

	constexpr Fixed dot(const Vector3 &v) const { return x * v.x + y * v.y + z * v.z; }

	constexpr Vector3 cross(const Vector3 &v) const
	{
		return {
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x,
		};
	}

	constexpr Vector3 abs() const { return {x.abs(), y.abs(), z.abs()}; }

	constexpr void invert() { x = -x; y = -y; z = -z; }

	constexpr Fixed squaredMagnitude() const { return x * x + y * y + z * z; }

	constexpr Vector3 reflected(const Vector3 &normal) const
	{
		return *this - normal * (dot(normal) * 2);
	}

	constexpr Vector3 lerp(const Vector3 &b, Fixed t) const
	{
		return *this + (b - *this) * t;
	}

	/* x, y and z are contiguous */
	constexpr Fixed       &operator[](int i)       { return (&x)[i]; }
	constexpr const Fixed &operator[](int i) const { return (&x)[i]; }

	Fixed   magnitude() const;
	void    normalize();
	Vector3 normalized() const;
};

constexpr Vector3 operator*(Fixed s, const Vector3 &v) { return v * s; }


#endif
