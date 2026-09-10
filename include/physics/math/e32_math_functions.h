#ifndef ENGINE_32_MATH_FUNCTIONS_H
#define ENGINE_32_MATH_FUNCTIONS_H

#include "physics/math/e32_vector3.h"


/* Closest points on segments and triangles. The triangle ones multiply
   two dot products together: in 20.12 that holds up to edges of 27 m
   with the point about as far. */

Vector3 segment_closestToPoint(const Vector3 &a, const Vector3 &b, const Vector3 &point);

void segment_closestToSegment(
	const Vector3 &a1, const Vector3 &b1,
	const Vector3 &a2, const Vector3 &b2,
	Vector3 *closest1, Vector3 *closest2);

void triangle_barycentric(
	const Vector3 &a, const Vector3 &b, const Vector3 &c,
	const Vector3 &point,
	Fixed *u, Fixed *v, Fixed *w);

Vector3 triangle_closestToPoint(
	const Vector3 &a, const Vector3 &b, const Vector3 &c,
	const Vector3 &point);


#endif
