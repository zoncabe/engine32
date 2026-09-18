#include "physics/math/e32_quaternion.h"

#include "physics/math/e32_math_common.h"


void Quaternion::setAxisAngle(const Vector3 &axis, Angle angle)
{
	Fixed s, c;
	Trig::sincos(angle / 2, &s, &c);
	x = s * axis.x;
	y = s * axis.y;
	z = s * axis.z;
	w = c;
}

Quaternion Quaternion::fromAxisAngle(const Vector3 &axis, Angle angle)
{
	Quaternion q;
	q.setAxisAngle(axis, angle);
	return q;
}

/* The half angle comes from the vector part against w, never from acos(w).
   On a unit quaternion the two agree, but acos is vertical at 1: a w one
   step below it, which is all a normalise in 20.12 can promise, reads as
   several degrees, and sqrt(1 - w*w) turns that same step into an axis
   length out of nothing. The identity then leaves here as a few degrees
   around whatever axis the caller substitutes for the degenerate one, and a
   static body built that way sits rotated against the mesh it was authored
   with. Measuring the vector part instead keeps zero at zero. */
void Quaternion::toAxisAngle(Vector3 *axis, Angle *angle) const
{
	Vector3 v = {x, y, z};
	Fixed   l = v.magnitude();

	*angle = Trig::atan2(l, w) * 2;

	if (l == 0) {
		*axis = Vector3::zero();
	} else {
		Fixed inv = 1 / l;
		*axis = {x * inv, y * inv, z * inv};
	}
}


Quaternion Quaternion::normalized() const
{
	Quaternion q = *this;
	Fixed d = w*w + x*x + y*y + z*z;
	if (d == 0) { q.w = 1.0_fp; d = 1.0_fp; }
	Fixed inv = inverseSqrt(d);
	q.x *= inv; q.y *= inv; q.z *= inv; q.w *= inv;
	return q;
}


void Quaternion::integrate(const Vector3 &omega, Fixed dt)
{
	Quaternion dq = { omega.x * dt, omega.y * dt, omega.z * dt, 0.0_fp };
	Quaternion m  = dq * *this;
	x += m.x / 2;
	y += m.y / 2;
	z += m.z / 2;
	w += m.w / 2;
	*this = normalized();
}


Matrix3 Quaternion::toMatrix3() const
{
	Fixed qx2 = x + x;
	Fixed qy2 = y + y;
	Fixed qz2 = z + z;
	Fixed qxqx2 = x * qx2;
	Fixed qxqy2 = x * qy2;
	Fixed qxqz2 = x * qz2;
	Fixed qxqw2 = w * qx2;
	Fixed qyqy2 = y * qy2;
	Fixed qyqz2 = y * qz2;
	Fixed qyqw2 = w * qy2;
	Fixed qzqz2 = z * qz2;
	Fixed qzqw2 = w * qz2;
	return {
		{ 1 - qyqy2 - qzqz2, qxqy2 + qzqw2,     qxqz2 - qyqw2     },
		{ qxqy2 - qzqw2,     1 - qxqx2 - qzqz2, qyqz2 + qxqw2     },
		{ qxqz2 + qyqw2,     qyqz2 - qxqw2,     1 - qxqx2 - qyqy2 },
	};
}


/* Shepperd's method: branch on the largest diagonal term to stay away from
   the singular traces. Follows the column convention of toMatrix3 above. */
Quaternion Quaternion::fromMatrix3(const Matrix3 &m)
{
	Fixed m00 = m.ex.x, m01 = m.ey.x, m02 = m.ez.x;
	Fixed m10 = m.ex.y, m11 = m.ey.y, m12 = m.ez.y;
	Fixed m20 = m.ex.z, m21 = m.ey.z, m22 = m.ez.z;
	Fixed trace = m00 + m11 + m22;
	Quaternion q;

	if (trace > 0) {
		Fixed s = sqrt(trace + 1) * 2;
		q.w = s / 4;
		q.x = (m21 - m12) / s;
		q.y = (m02 - m20) / s;
		q.z = (m10 - m01) / s;
	}
	else if (m00 > m11 && m00 > m22) {
		Fixed s = sqrt(1 + m00 - m11 - m22) * 2;
		q.w = (m21 - m12) / s;
		q.x = s / 4;
		q.y = (m01 + m10) / s;
		q.z = (m02 + m20) / s;
	}
	else if (m11 > m22) {
		Fixed s = sqrt(1 + m11 - m00 - m22) * 2;
		q.w = (m02 - m20) / s;
		q.x = (m01 + m10) / s;
		q.y = s / 4;
		q.z = (m12 + m21) / s;
	}
	else {
		Fixed s = sqrt(1 + m22 - m00 - m11) * 2;
		q.w = (m10 - m01) / s;
		q.x = (m02 + m20) / s;
		q.y = (m12 + m21) / s;
		q.z = s / 4;
	}

	return q.normalized();
}


Quaternion Quaternion::nlerp(const Quaternion &b, Fixed t) const
{
	Fixed dot  = x * b.x + y * b.y + z * b.z + w * b.w;
	int   sign = (dot < 0) ? -1 : 1;

	Quaternion q = {
		x + (b.x * sign - x) * t,
		y + (b.y * sign - y) * t,
		z + (b.z * sign - z) * t,
		w + (b.w * sign - w) * t,
	};
	return q.normalized();
}


Vector3 Quaternion::rotate(const Vector3 &v) const
{
	Fixed tx = (y * v.z - z * v.y) * 2;
	Fixed ty = (z * v.x - x * v.z) * 2;
	Fixed tz = (x * v.y - y * v.x) * 2;

	return {
		v.x + w * tx + (y * tz - z * ty),
		v.y + w * ty + (z * tx - x * tz),
		v.z + w * tz + (x * ty - y * tx),
	};
}


/*
 * 2 bits pick the largest component, the other three are 10 bits each in
 * [-1/sqrt(2), 1/sqrt(2)], and the largest is rebuilt from the unit norm.
 * Ported from tiny3d's t3danim (Max Bebök, MIT, see LICENSE).
 */

/* 0..1023 to [-1/sqrt(2), 1/sqrt(2)] in Q12: the span is sqrt(2), 5793
   raw, and the offset 1/sqrt(2), 2896 raw. */
static Fixed s10ToFixed(uint32_t value)
{
	int32_t raw = (int32_t)(((int64_t)value * 5793) / 1023) - 2896;
	return Fixed(raw, Fixed::RAW);
}

Quaternion Quaternion::unpacked(uint16_t dataHi, uint16_t dataLo)
{
	int largestIdx = dataHi >> 14;
	int idx0 = (largestIdx + 1) & 0b11;
	int idx1 = (largestIdx + 2) & 0b11;
	int idx2 = (largestIdx + 3) & 0b11;

	uint16_t dataMid = (dataHi << 6) | (dataLo >> 10);
	Fixed q0 = s10ToFixed((dataHi >> 4) & 0x3FF);
	Fixed q1 = s10ToFixed((dataMid    ) & 0x3FF);
	Fixed q2 = s10ToFixed((dataLo     ) & 0x3FF);

	Quaternion out;
	out[idx0] = q0;
	out[idx1] = q1;
	out[idx2] = q2;
	out[largestIdx] = sqrt(1 - q0*q0 - q1*q1 - q2*q2);
	return out;
}
