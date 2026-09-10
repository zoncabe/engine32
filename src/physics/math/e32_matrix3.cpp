#include "physics/math/e32_matrix3.h"

#include "physics/math/e32_math_common.h"


Matrix3 Matrix3::fromAxisAngle(const Vector3 &axis, Angle angle)
{
	Fixed s, c;
	Trig::sincos(angle, &s, &c);
	Fixed x = axis.x, y = axis.y, z = axis.z;
	Fixed xy = x * y, yz = y * z, zx = z * x;
	Fixed t = 1 - c;
	return {
		{ x*x*t + c,   xy*t + z*s, zx*t - y*s },
		{ xy*t - z*s,  y*y*t + c,  yz*t + x*s },
		{ zx*t + y*s,  yz*t - x*s, z*z*t + c  },
	};
}


Matrix3 Matrix3::fromEuler(Angle pitch, Angle yaw, Angle roll)
{
	Fixed sp, cp, sy, cy, sr, cr;
	Trig::sincos(pitch, &sp, &cp);
	Trig::sincos(yaw,   &sy, &cy);
	Trig::sincos(roll,  &sr, &cr);

	/* Same layout as libdragon's fm_mat4_from_srt_euler, which is what the
	   N64 build rotated with: the scenes were authored against it, so an
	   angle has to turn the same way here. */
	return {
		{
			cr * cy,
			cr * sy * sp - sr * cp,
			cr * sy * cp + sr * sp
		},
		{
			sr * cy,
			sr * sy * sp + cr * cp,
			sr * sy * cp - cr * sp
		},
		{
			-sy,
			cy * sp,
			cy * cp
		},
	};
}


Matrix3 Matrix3::outerProduct(const Vector3 &u, const Vector3 &v)
{
	return {
		{ v.x * u.x, v.y * u.x, v.z * u.x },
		{ v.x * u.y, v.y * u.y, v.z * u.y },
		{ v.x * u.z, v.y * u.z, v.z * u.z },
	};
}


Matrix3 Matrix3::inverse() const
{
	Vector3 t0 = ey.cross(ez);
	Vector3 t1 = ez.cross(ex);
	Vector3 t2 = ex.cross(ey);
	Fixed detinv = 1 / ez.dot(t2);
	return {
		{ t0.x * detinv, t1.x * detinv, t2.x * detinv },
		{ t0.y * detinv, t1.y * detinv, t2.y * detinv },
		{ t0.z * detinv, t1.z * detinv, t2.z * detinv },
	};
}


EulerAngles Matrix3::toEuler() const
{
	/* Inverse of fromEuler above: ez holds the yaw and pitch, the first row
	   the roll. */
	EulerAngles euler;
	Fixed sy_p = -ez.x;
	Fixed cy_p = sqrt(ez.y * ez.y + ez.z * ez.z);

	euler.y = Trig::atan2(sy_p, cy_p);
	if (cy_p.raw() > 0) {
		euler.x = Trig::atan2(ez.y, ez.z);
		euler.z = Trig::atan2(ey.x, ex.x);
	} else {
		euler.x = Angle();
		euler.z = Trig::atan2(-ex.y, ey.y);
	}
	return euler;
}
