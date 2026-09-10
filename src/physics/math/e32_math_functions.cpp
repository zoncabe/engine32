#include "physics/math/e32_math_common.h"
#include "physics/math/e32_math_functions.h"


Vector3 segment_closestToPoint(const Vector3 &a, const Vector3 &b, const Vector3 &point)
{
	Vector3 ab = b - a;
	Vector3 ap = point - a;

	Fixed ab_len_sq = ab.squaredMagnitude();
	if (ab_len_sq < TOLERANCE) return a;

	Fixed t = clamp(ap.dot(ab) / ab_len_sq, 0.0_fp, 1.0_fp);

	return a + ab * t;
}


void segment_closestToSegment(
	const Vector3 &a1, const Vector3 &b1,
	const Vector3 &a2, const Vector3 &b2,
	Vector3 *closest1, Vector3 *closest2)
{
	Vector3 d1 = b1 - a1;
	Vector3 d2 = b2 - a2;
	Vector3 r  = a1 - a2;

	Fixed a = d1.squaredMagnitude();
	Fixed e = d2.squaredMagnitude();
	Fixed f = d2.dot(r);

	Fixed s, t;

	if (a <= TOLERANCE && e <= TOLERANCE) {
		*closest1 = a1;
		*closest2 = a2;
		return;
	}

	if (a <= TOLERANCE) {
		s = 0;
		t = clamp(f / e, 0.0_fp, 1.0_fp);
	}
	else {
		Fixed c = d1.dot(r);
		if (e <= TOLERANCE) {
			t = 0;
			s = clamp(-c / a, 0.0_fp, 1.0_fp);
		}
		else {
			Fixed b     = d1.dot(d2);
			Fixed denom = a * e - b * b;
			if (denom.raw() != 0) s = clamp((b*f - c*e) / denom, 0.0_fp, 1.0_fp);
			else                  s = 0;
			t = (b*s + f) / e;
			if (t < 0) {
				t = 0;
				s = clamp(-c / a, 0.0_fp, 1.0_fp);
			}
			else if (t > 1.0_fp) {
				t = 1.0_fp;
				s = clamp((b - c) / a, 0.0_fp, 1.0_fp);
			}
		}
	}

	*closest1 = a1 + d1 * s;
	*closest2 = a2 + d2 * t;
}


void triangle_barycentric(
	const Vector3 &a, const Vector3 &b, const Vector3 &c,
	const Vector3 &point,
	Fixed *u, Fixed *v, Fixed *w)
{
	Vector3 v0 = b - a;
	Vector3 v1 = c - a;
	Vector3 v2 = point - a;

	Fixed d00 = v0.dot(v0);
	Fixed d01 = v0.dot(v1);
	Fixed d11 = v1.dot(v1);
	Fixed d20 = v2.dot(v0);
	Fixed d21 = v2.dot(v1);

	Fixed denom = d00 * d11 - d01 * d01;
	if (denom < TOLERANCE) {
		*u = 1.0_fp; *v = 0; *w = 0;
		return;
	}

	*v = (d11 * d20 - d01 * d21) / denom;
	*w = (d00 * d21 - d01 * d20) / denom;
	*u = 1.0_fp - *v - *w;
}


Vector3 triangle_closestToPoint(
	const Vector3 &a, const Vector3 &b, const Vector3 &c,
	const Vector3 &point)
{
	Vector3 ab = b - a;
	Vector3 ac = c - a;
	Vector3 ap = point - a;

	Fixed d1 = ab.dot(ap);
	Fixed d2 = ac.dot(ap);
	if (d1 <= 0 && d2 <= 0) return a;

	Vector3 bp = point - b;
	Fixed d3 = ab.dot(bp);
	Fixed d4 = ac.dot(bp);
	if (d3 >= 0 && d4 <= d3) return b;

	Fixed vc = d1*d4 - d3*d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0) {
		Fixed v = d1 / (d1 - d3);
		return a + ab * v;
	}

	Vector3 cp = point - c;
	Fixed d5 = ab.dot(cp);
	Fixed d6 = ac.dot(cp);
	if (d6 >= 0 && d5 <= d6) return c;

	Fixed vb = d5*d2 - d1*d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0) {
		Fixed w = d2 / (d2 - d6);
		return a + ac * w;
	}

	Fixed va = d3*d6 - d5*d4;
	if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
		Fixed w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + (c - b) * w;
	}

	/* Each weight divides on its own: the inverse of the sum would keep
	   only 12 bits. The sum is a product of two squared lengths: on a
	   triangle a few centimetres across it rounds to nothing. */
	Fixed sum = va + vb + vc;
	if (sum.raw() == 0) return a;
	Fixed v = vb / sum;
	Fixed w = vc / sum;
	return a + ab * v + ac * w;
}
