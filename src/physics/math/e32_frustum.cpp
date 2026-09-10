/* Ported from tiny3d's t3dmath frustum functions (Max Bebök, MIT, see
 * LICENSE), rewired to the engine's math types. */
#include "physics/math/e32_frustum.h"
#include "physics/math/e32_math_common.h"
#include "graphics/e32_model_format.h"


/* A side plane through the eye: its normal in camera space, then turned
   into the world with the transpose of the view rotation. */
static FrustumPlane sidePlane(const Matrix3 &rotation, const Vector3 &position, Vector3 normal)
{
	normal.normalize();
	Vector3 world = rotation.transformTransposed(normal);
	return { world, -world.dot(position) };
}

Frustum Frustum::fromView(const Matrix3 &rotation, const Vector3 &position,
                          int projection, int half_width, int half_height,
                          Fixed near, Fixed far)
{
	Frustum f;

	/* Camera space is x right, y down, z forward. A point is on screen while
	   |x| <= z * half_width / projection, and the same for y. The plane
	   normals below point inside. */
	Fixed h = Fixed(projection, 0);
	Fixed w = Fixed(half_width, 0);
	Fixed v = Fixed(half_height, 0);

	f.planes[0] = sidePlane(rotation, position, { h, 0.0_fp, w });    /* left   */
	f.planes[1] = sidePlane(rotation, position, {-h, 0.0_fp, w });    /* right  */
	f.planes[2] = sidePlane(rotation, position, { 0.0_fp, -h, v });   /* bottom */
	f.planes[3] = sidePlane(rotation, position, { 0.0_fp,  h, v });   /* top    */

	Vector3 forward = rotation.transformTransposed({ 0.0_fp, 0.0_fp, 1.0_fp });
	f.planes[4] = {  forward, -forward.dot(position) - near };        /* near   */
	f.planes[5] = { -forward,  forward.dot(position) + far  };        /* far    */

	return f;
}

Frustum Frustum::toModel(const Matrix3 &rotation, const Vector3 &position, const Vector3 &scale) const
{
	Frustum f;
	for (int i = 0; i < 6; ++i) {
		const FrustumPlane &p = planes[i];
		/* n . (R S m + t) + d >= 0  ->  (S R^T n) . m + (d + n . t) >= 0 */
		Vector3 n = rotation.transformTransposed(p.normal);
		n.x *= scale.x;
		n.y *= scale.y;
		n.z *= scale.z;
		f.planes[i] = { n, p.distance + p.normal.dot(position) };
	}
	return f;
}

void Frustum::scale(Fixed factor)
{
	for (int i = 0; i < 6; ++i)
		planes[i].normal *= factor;
}

static bool aabbInside(const FrustumPlane *planes, const Vector3 &min, const Vector3 &max)
{
	for (int i = 0; i < 6; ++i) {
		const Vector3 &n = planes[i].normal;

		Fixed p0Min = n.x * min.x;
		Fixed p0Max = n.x * max.x;
		Fixed p1Min = n.y * min.y;
		Fixed p1Max = n.y * max.y;

		Fixed p2MinAndW = -planes[i].distance - n.z * min.z;
		if (p0Min + p1Min > p2MinAndW) continue;
		if (p0Max + p1Min > p2MinAndW) continue;
		if (p0Min + p1Max > p2MinAndW) continue;
		if (p0Max + p1Max > p2MinAndW) continue;

		Fixed p2MaxAndW = -planes[i].distance - n.z * max.z;
		if (p0Min + p1Min > p2MaxAndW) continue;
		if (p0Max + p1Min > p2MaxAndW) continue;
		if (p0Min + p1Max > p2MaxAndW) continue;
		if (p0Max + p1Max > p2MaxAndW) continue;
		return false;
	}
	return true;
}

bool Frustum::vsAabb(const Vector3 &min, const Vector3 &max) const
{
	return aabbInside(planes, min, max);
}

/* The vertices' units to 20.12. */
static Fixed fromS16(int16_t v)
{
	return Fixed((int32_t)v << MODEL_CPU_TO_UNITS, Fixed::RAW);
}

bool Frustum::vsAabbS16(const int16_t min[3], const int16_t max[3]) const
{
	Vector3 lo = { fromS16(min[0]), fromS16(min[1]), fromS16(min[2]) };
	Vector3 hi = { fromS16(max[0]), fromS16(max[1]), fromS16(max[2]) };
	return aabbInside(planes, lo, hi);
}

bool Frustum::vsSphere(const Vector3 &center, Fixed radius) const
{
	for (int i = 0; i < 6; ++i) {
		Fixed dist = planes[i].normal.dot(center) + planes[i].distance;
		if (dist < -radius) return false;
	}
	return true;
}
