/*
	Ported from qu3e q3Collide.cpp — altered source, not the original software.

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/

/*
	Narrowphase. The OBB-vs-OBB SAT and Sutherland-Hodgman clip come from
	qu3e; the type-based dispatcher and the sphere / capsule / triangle pairs
	are added on top.
*/
#include <stddef.h>
#include <stdint.h>

#include "physics/collision/e32_collision.h"
#include "physics/collision/e32_collision_mesh.h"
#include "physics/body/e32_rigid_body.h"
#include "physics/math/e32_math_common.h"
#include "physics/math/e32_math_functions.h"    /* segment_closestToPoint */


/* The largest and smallest 20.12 values: what FLT_MAX stood for. */
#define FIXED_MAX Fixed( 0x7FFFFFFF, Fixed::RAW)
#define FIXED_MIN Fixed(-0x7FFFFFFF, Fixed::RAW)


static inline bool trackFaceAxis(int32_t *axis, int32_t n, Fixed s, Fixed *s_max,
                                 const Vector3 &normal, Vector3 *axis_normal)
{
	if (s > 0) return true;
	if (s > *s_max) {
		*s_max       = s;
		*axis        = n;
		*axis_normal = normal;
	}
	return false;
}


static inline bool trackEdgeAxis(int32_t *axis, int32_t n, Fixed s, Fixed *s_max,
                                 const Vector3 &normal, Vector3 *axis_normal)
{
	if (s > 0) return true;
	Fixed l = inverseSqrt(normal.squaredMagnitude());
	s *= l;
	if (s > *s_max) {
		*s_max       = s;
		*axis        = n;
		*axis_normal = normal * l;
	}
	return false;
}


struct ClipVertex
{
	Vector3     v;
	FeaturePair f;
};


static void computeReferenceEdgesAndBasis(const Vector3 &e_r, const Transform &rtx, Vector3 n, int32_t axis,
                                           uint8_t *out, Matrix3 *basis, Vector3 *e)
{
	n = rtx.rotation.transformTransposed(n);

	if (axis >= 3) axis -= 3;

	Vector3 neg_ex = -rtx.rotation.ex;
	Vector3 neg_ey = -rtx.rotation.ey;
	Vector3 neg_ez = -rtx.rotation.ez;

	switch (axis) {
	case 0:
		if (n.x > 0) {
			out[0] = 1;  out[1] = 8;  out[2] = 7;  out[3] = 9;
			*e = { e_r.y, e_r.z, e_r.x };
			*basis = Matrix3::fromColumns(rtx.rotation.ey, rtx.rotation.ez, rtx.rotation.ex);
		} else {
			out[0] = 11; out[1] = 3;  out[2] = 10; out[3] = 5;
			*e = { e_r.z, e_r.y, e_r.x };
			*basis = Matrix3::fromColumns(rtx.rotation.ez, rtx.rotation.ey, neg_ex);
		}
		break;

	case 1:
		if (n.y > 0) {
			out[0] = 0;  out[1] = 1;  out[2] = 2;  out[3] = 3;
			*e = { e_r.z, e_r.x, e_r.y };
			*basis = Matrix3::fromColumns(rtx.rotation.ez, rtx.rotation.ex, rtx.rotation.ey);
		} else {
			out[0] = 4;  out[1] = 5;  out[2] = 6;  out[3] = 7;
			*e = { e_r.z, e_r.x, e_r.y };
			*basis = Matrix3::fromColumns(rtx.rotation.ez, neg_ex, neg_ey);
		}
		break;

	case 2:
		if (n.z > 0) {
			out[0] = 11; out[1] = 4;  out[2] = 8;  out[3] = 0;
			*e = { e_r.y, e_r.x, e_r.z };
			*basis = Matrix3::fromColumns(neg_ey, rtx.rotation.ex, rtx.rotation.ez);
		} else {
			out[0] = 6;  out[1] = 10; out[2] = 2;  out[3] = 9;
			*e = { e_r.y, e_r.x, e_r.z };
			*basis = Matrix3::fromColumns(neg_ey, neg_ex, neg_ez);
		}
		break;
	}
}


/* Corners of the incident face, on the box opposing the reference face. */
static void computeIncidentFace(const Transform &itx, const Vector3 &e, const Vector3 &n_world, ClipVertex *out)
{
	Vector3 n     = -itx.rotation.transformTransposed(n_world);
	Vector3 abs_n = n.abs();

	/* Only in_i/out_i are set per face below; the reference halves of the key
	   are filled by the clipper. Start from ~0 as qu3e's ClipVertex does, so a
	   key is the same bytes step after step and warm starting can match it. */
	for (int32_t i = 0; i < 4; ++i) out[i].f.key = ~0;

	if (abs_n.x > abs_n.y && abs_n.x > abs_n.z) {
		if (n.x > 0) {
			out[0].v = { e.x,  e.y, -e.z};
			out[1].v = { e.x,  e.y,  e.z};
			out[2].v = { e.x, -e.y,  e.z};
			out[3].v = { e.x, -e.y, -e.z};

			out[0].f.in_i = 9;  out[0].f.out_i = 1;
			out[1].f.in_i = 1;  out[1].f.out_i = 8;
			out[2].f.in_i = 8;  out[2].f.out_i = 7;
			out[3].f.in_i = 7;  out[3].f.out_i = 9;
		} else {
			out[0].v = {-e.x, -e.y,  e.z};
			out[1].v = {-e.x,  e.y,  e.z};
			out[2].v = {-e.x,  e.y, -e.z};
			out[3].v = {-e.x, -e.y, -e.z};

			out[0].f.in_i = 5;  out[0].f.out_i = 11;
			out[1].f.in_i = 11; out[1].f.out_i = 3;
			out[2].f.in_i = 3;  out[2].f.out_i = 10;
			out[3].f.in_i = 10; out[3].f.out_i = 5;
		}
	}
	else if (abs_n.y > abs_n.x && abs_n.y > abs_n.z) {
		if (n.y > 0) {
			out[0].v = {-e.x,  e.y,  e.z};
			out[1].v = { e.x,  e.y,  e.z};
			out[2].v = { e.x,  e.y, -e.z};
			out[3].v = {-e.x,  e.y, -e.z};

			out[0].f.in_i = 3;  out[0].f.out_i = 0;
			out[1].f.in_i = 0;  out[1].f.out_i = 1;
			out[2].f.in_i = 1;  out[2].f.out_i = 2;
			out[3].f.in_i = 2;  out[3].f.out_i = 3;
		} else {
			out[0].v = { e.x, -e.y,  e.z};
			out[1].v = {-e.x, -e.y,  e.z};
			out[2].v = {-e.x, -e.y, -e.z};
			out[3].v = { e.x, -e.y, -e.z};

			out[0].f.in_i = 7;  out[0].f.out_i = 4;
			out[1].f.in_i = 4;  out[1].f.out_i = 5;
			out[2].f.in_i = 5;  out[2].f.out_i = 6;
			out[3].f.in_i = 5;  out[3].f.out_i = 6;
		}
	}
	else {
		if (n.z > 0) {
			out[0].v = {-e.x,  e.y,  e.z};
			out[1].v = {-e.x, -e.y,  e.z};
			out[2].v = { e.x, -e.y,  e.z};
			out[3].v = { e.x,  e.y,  e.z};

			out[0].f.in_i = 0;  out[0].f.out_i = 11;
			out[1].f.in_i = 11; out[1].f.out_i = 4;
			out[2].f.in_i = 4;  out[2].f.out_i = 8;
			out[3].f.in_i = 8;  out[3].f.out_i = 0;
		} else {
			out[0].v = { e.x, -e.y, -e.z};
			out[1].v = {-e.x, -e.y, -e.z};
			out[2].v = {-e.x,  e.y, -e.z};
			out[3].v = { e.x,  e.y, -e.z};

			out[0].f.in_i = 9;  out[0].f.out_i = 6;
			out[1].f.in_i = 6;  out[1].f.out_i = 10;
			out[2].f.in_i = 10; out[2].f.out_i = 2;
			out[3].f.in_i = 2;  out[3].f.out_i = 9;
		}
	}

	for (int32_t i = 0; i < 4; ++i) {
		out[i].v = itx.mulVector(out[i].v);
	}
}


/* Sutherland-Hodgman one-plane clip. */
#define IN_FRONT(a) ((a) < 0)
#define BEHIND(a)   ((a) >= 0)
#define ON_PLANE(a) ((a) < 0.005_fp && (a) > -0.005_fp)

static int32_t orthographic(int sign, Fixed e, int32_t axis, int32_t clip_edge,
                             ClipVertex *in, int32_t in_count, ClipVertex *out)
{
	int32_t out_count = 0;
	ClipVertex a = in[in_count - 1];

	for (int32_t i = 0; i < in_count; ++i) {
		ClipVertex b = in[i];

		Fixed da = a.v[axis] * sign - e;
		Fixed db = b.v[axis] * sign - e;

		ClipVertex cv;

		if (((IN_FRONT(da) && IN_FRONT(db)) || ON_PLANE(da) || ON_PLANE(db))) {
			out[out_count++] = b;
		}
		else if (IN_FRONT(da) && BEHIND(db)) {
			cv.v           = a.v + (b.v - a.v) * (da / (da - db));
			cv.f           = b.f;
			cv.f.out_r     = (uint8_t)clip_edge;
			cv.f.out_i     = 0;
			out[out_count++] = cv;
		}
		else if (BEHIND(da) && IN_FRONT(db)) {
			cv.v           = a.v + (b.v - a.v) * (da / (da - db));
			cv.f           = a.f;
			cv.f.in_r      = (uint8_t)clip_edge;
			cv.f.in_i      = 0;
			out[out_count++] = cv;
			out[out_count++] = b;
		}

		a = b;
	}

	return out_count;
}


/* Clip the incident face against the reference face. */
static int32_t clipFace(const Vector3 &r_pos, const Vector3 &e, const uint8_t *clip_edges, const Matrix3 &basis,
                         const ClipVertex *incident, ClipVertex *out_verts, Fixed *out_depths)
{
	int32_t in_count = 4;
	int32_t out_count;
	/* Ping-pong buffers, always written up to their count before being
	   read: no fill needed. */
	ClipVertex in[8]  __attribute__((uninitialized));
	ClipVertex out[8] __attribute__((uninitialized));

	for (int32_t i = 0; i < 4; ++i) {
		in[i].v = basis.transformTransposed(incident[i].v - r_pos);
		in[i].f = incident[i].f;
	}

	out_count = orthographic( 1, e.x, 0, clip_edges[0], in,  in_count,  out);
	if (!out_count) return 0;

	in_count  = orthographic( 1, e.y, 1, clip_edges[1], out, out_count, in);
	if (!in_count) return 0;

	out_count = orthographic(-1, e.x, 0, clip_edges[2], in,  in_count,  out);
	if (!out_count) return 0;

	in_count  = orthographic(-1, e.y, 1, clip_edges[3], out, out_count, in);

	/* Keep only vertices behind the reference face. */
	out_count = 0;
	for (int32_t i = 0; i < in_count; ++i) {
		Fixed d = in[i].v.z - e.z;
		if (d <= 0) {
			out_verts[out_count].v = basis.transform(in[i].v) + r_pos;
			out_verts[out_count].f = in[i].f;
			out_depths[out_count++] = d;
		}
	}

	return out_count;
}


/* Closest points between two line segments (edge-edge contact). */
static void edgesContact(Vector3 *ca, Vector3 *cb,
                          const Vector3 &pa, const Vector3 &qa, const Vector3 &pb, const Vector3 &qb)
{
	Vector3 da = qa - pa;
	Vector3 db = qb - pb;
	Vector3 r  = pa - pb;
	Fixed a = da.dot(da);
	Fixed e = db.dot(db);
	Fixed f = db.dot(r);
	Fixed c = da.dot(r);
	Fixed b = da.dot(db);
	Fixed denom = a * e - b * b;

	Fixed ta = (b * f - c * e) / denom;
	Fixed tb = (b * ta + f) / e;

	*ca = pa + da * ta;
	*cb = pb + db * tb;
}


/* Supporting edge of a box for the given direction n (world space). */
static void supportEdge(const Transform &tx, const Vector3 &e, const Vector3 &n, Vector3 *a_out, Vector3 *b_out)
{
	Vector3 n_local = tx.rotation.transformTransposed(n);
	Vector3 abs_n   = n_local.abs();
	Vector3 a, b;

	if (abs_n.x > abs_n.y) {
		if (abs_n.y > abs_n.z) {
			a = { e.x,  e.y,  e.z};
			b = { e.x,  e.y, -e.z};
		} else {
			a = { e.x,  e.y,  e.z};
			b = { e.x, -e.y,  e.z};
		}
	} else {
		if (abs_n.x > abs_n.z) {
			a = { e.x,  e.y,  e.z};
			b = { e.x,  e.y, -e.z};
		} else {
			a = { e.x,  e.y,  e.z};
			b = {-e.x,  e.y,  e.z};
		}
	}

	if (n_local.x < 0) { a.x = -a.x; b.x = -b.x; }
	if (n_local.y < 0) { a.y = -a.y; b.y = -b.y; }
	if (n_local.z < 0) { a.z = -a.z; b.z = -b.z; }

	*a_out = tx.mulVector(a);
	*b_out = tx.mulVector(b);
}


/* OBB-vs-OBB SAT plus face and edge clipping. */
void boxToBox(ContactManifold *m, PhysicsShape *a, PhysicsShape *b)
{
	Transform atx = a->body->tx * a->local;
	Transform btx = b->body->tx * b->local;
	Vector3 e_a = a->box.e;
	Vector3 e_b = b->box.e;

	/* B's frame in A's space. */
	Matrix3 C = atx.rotation.transposed() * btx.rotation;

	/* Two axes within one raw unit of parallel: the edge cross products
	   between them would be too short to trust. */
	Matrix3 abs_C = { C.ex.abs(), C.ey.abs(), C.ez.abs() };
	bool parallel = false;
	const Fixed k_cos_tol = 1.0_fp - TOLERANCE;
	for (int32_t i = 0; i < 3; ++i) {
		for (int32_t j = 0; j < 3; ++j) {
			if (abs_C.get(i, j) >= k_cos_tol) parallel = true;
		}
	}

	Vector3 t = atx.rotation.transformTransposed(btx.position - atx.position);

	Fixed s;
	Fixed a_max = FIXED_MIN;
	Fixed b_max = FIXED_MIN;
	Fixed e_max = FIXED_MIN;
	int32_t a_axis = ~0;
	int32_t b_axis = ~0;
	int32_t e_axis = ~0;
	Vector3 n_a = Vector3::zero();
	Vector3 n_b = Vector3::zero();
	Vector3 n_e = Vector3::zero();

	Vector3 col0 = abs_C.column0();
	Vector3 col1 = abs_C.column1();
	Vector3 col2 = abs_C.column2();

	/* Face axes of A. */
	s = t.x.abs() - (e_a.x + col0.dot(e_b));
	if (trackFaceAxis(&a_axis, 0, s, &a_max, atx.rotation.ex, &n_a)) return;

	s = t.y.abs() - (e_a.y + col1.dot(e_b));
	if (trackFaceAxis(&a_axis, 1, s, &a_max, atx.rotation.ey, &n_a)) return;

	s = t.z.abs() - (e_a.z + col2.dot(e_b));
	if (trackFaceAxis(&a_axis, 2, s, &a_max, atx.rotation.ez, &n_a)) return;

	/* Face axes of B. */
	s = t.dot(C.ex).abs() - (e_b.x + abs_C.ex.dot(e_a));
	if (trackFaceAxis(&b_axis, 3, s, &b_max, btx.rotation.ex, &n_b)) return;

	s = t.dot(C.ey).abs() - (e_b.y + abs_C.ey.dot(e_a));
	if (trackFaceAxis(&b_axis, 4, s, &b_max, btx.rotation.ey, &n_b)) return;

	s = t.dot(C.ez).abs() - (e_b.z + abs_C.ez.dot(e_a));
	if (trackFaceAxis(&b_axis, 5, s, &b_max, btx.rotation.ez, &n_b)) return;

	if (!parallel) {
		Fixed r_a, r_b;
		Vector3 n;

		/* Cross(a.x, b.x) */
		r_a = e_a.y * abs_C.get(0, 2) + e_a.z * abs_C.get(0, 1);
		r_b = e_b.y * abs_C.get(2, 0) + e_b.z * abs_C.get(1, 0);
		s   = (t.z * C.get(0, 1) - t.y * C.get(0, 2)).abs() - (r_a + r_b);
		n = { 0.0_fp, -C.get(0, 2), C.get(0, 1) };
		if (trackEdgeAxis(&e_axis, 6, s, &e_max, n, &n_e)) return;

		/* Cross(a.x, b.y) */
		r_a = e_a.y * abs_C.get(1, 2) + e_a.z * abs_C.get(1, 1);
		r_b = e_b.x * abs_C.get(2, 0) + e_b.z * abs_C.get(0, 0);
		s   = (t.z * C.get(1, 1) - t.y * C.get(1, 2)).abs() - (r_a + r_b);
		n = { 0.0_fp, -C.get(1, 2), C.get(1, 1) };
		if (trackEdgeAxis(&e_axis, 7, s, &e_max, n, &n_e)) return;

		/* Cross(a.x, b.z) */
		r_a = e_a.y * abs_C.get(2, 2) + e_a.z * abs_C.get(2, 1);
		r_b = e_b.x * abs_C.get(1, 0) + e_b.y * abs_C.get(0, 0);
		s   = (t.z * C.get(2, 1) - t.y * C.get(2, 2)).abs() - (r_a + r_b);
		n = { 0.0_fp, -C.get(2, 2), C.get(2, 1) };
		if (trackEdgeAxis(&e_axis, 8, s, &e_max, n, &n_e)) return;

		/* Cross(a.y, b.x) */
		r_a = e_a.x * abs_C.get(0, 2) + e_a.z * abs_C.get(0, 0);
		r_b = e_b.y * abs_C.get(2, 1) + e_b.z * abs_C.get(1, 1);
		s   = (t.x * C.get(0, 2) - t.z * C.get(0, 0)).abs() - (r_a + r_b);
		n = { C.get(0, 2), 0.0_fp, -C.get(0, 0) };
		if (trackEdgeAxis(&e_axis, 9, s, &e_max, n, &n_e)) return;

		/* Cross(a.y, b.y) */
		r_a = e_a.x * abs_C.get(1, 2) + e_a.z * abs_C.get(1, 0);
		r_b = e_b.x * abs_C.get(2, 1) + e_b.z * abs_C.get(0, 1);
		s   = (t.x * C.get(1, 2) - t.z * C.get(1, 0)).abs() - (r_a + r_b);
		n = { C.get(1, 2), 0.0_fp, -C.get(1, 0) };
		if (trackEdgeAxis(&e_axis, 10, s, &e_max, n, &n_e)) return;

		/* Cross(a.y, b.z) */
		r_a = e_a.x * abs_C.get(2, 2) + e_a.z * abs_C.get(2, 0);
		r_b = e_b.x * abs_C.get(1, 1) + e_b.y * abs_C.get(0, 1);
		s   = (t.x * C.get(2, 2) - t.z * C.get(2, 0)).abs() - (r_a + r_b);
		n = { C.get(2, 2), 0.0_fp, -C.get(2, 0) };
		if (trackEdgeAxis(&e_axis, 11, s, &e_max, n, &n_e)) return;

		/* Cross(a.z, b.x) */
		r_a = e_a.x * abs_C.get(0, 1) + e_a.y * abs_C.get(0, 0);
		r_b = e_b.y * abs_C.get(2, 2) + e_b.z * abs_C.get(1, 2);
		s   = (t.y * C.get(0, 0) - t.x * C.get(0, 1)).abs() - (r_a + r_b);
		n = { -C.get(0, 1), C.get(0, 0), 0.0_fp };
		if (trackEdgeAxis(&e_axis, 12, s, &e_max, n, &n_e)) return;

		/* Cross(a.z, b.y) */
		r_a = e_a.x * abs_C.get(1, 1) + e_a.y * abs_C.get(1, 0);
		r_b = e_b.x * abs_C.get(2, 2) + e_b.z * abs_C.get(0, 2);
		s   = (t.y * C.get(1, 0) - t.x * C.get(1, 1)).abs() - (r_a + r_b);
		n = { -C.get(1, 1), C.get(1, 0), 0.0_fp };
		if (trackEdgeAxis(&e_axis, 13, s, &e_max, n, &n_e)) return;

		/* Cross(a.z, b.z) */
		r_a = e_a.x * abs_C.get(2, 1) + e_a.y * abs_C.get(2, 0);
		r_b = e_b.x * abs_C.get(1, 2) + e_b.y * abs_C.get(0, 2);
		s   = (t.y * C.get(2, 0) - t.x * C.get(2, 1)).abs() - (r_a + r_b);
		n = { -C.get(2, 1), C.get(2, 0), 0.0_fp };
		if (trackEdgeAxis(&e_axis, 14, s, &e_max, n, &n_e)) return;
	}

	/* Pick the SAT axis, biased to avoid flipping between frames. */
	const Fixed k_rel_tol = 0.95_fp;
	const Fixed k_abs_tol = 0.01_fp;
	int32_t axis;
	Fixed   s_max;
	Vector3 n;
	Fixed face_max = (a_max > b_max) ? a_max : b_max;
	if (k_rel_tol * e_max > face_max + k_abs_tol) {
		axis = e_axis; s_max = e_max; n = n_e;
	} else {
		if (k_rel_tol * b_max > a_max + k_abs_tol) {
			axis = b_axis; s_max = b_max; n = n_b;
		} else {
			axis = a_axis; s_max = a_max; n = n_a;
		}
	}

	if (n.dot(btx.position - atx.position) < 0) {
		n = -n;
	}

	if (axis == ~0) return;

	if (axis < 6) {
		Transform rtx;
		Transform itx;
		Vector3   e_r;
		Vector3   e_i;
		bool      flip;

		if (axis < 3) {
			rtx = atx; itx = btx; e_r = e_a; e_i = e_b; flip = false;
		} else {
			rtx = btx; itx = atx; e_r = e_b; e_i = e_a; flip = true;
			n = -n;
		}

		ClipVertex incident[4] __attribute__((uninitialized));
		computeIncidentFace(itx, e_i, n, incident);
		uint8_t clip_edges[4] = { 0, 0, 0, 0 };
		Matrix3 basis         = Matrix3::identity();
		Vector3 e             = Vector3::zero();
		computeReferenceEdgesAndBasis(e_r, rtx, n, axis, clip_edges, &basis, &e);

		ClipVertex out[8]    __attribute__((uninitialized));
		Fixed      depths[8] __attribute__((uninitialized));
		int32_t out_num = clipFace(rtx.position, e, clip_edges, basis, incident, out, depths);

		if (out_num) {
			m->contact_count = out_num;
			m->normal        = flip ? -n : n;

			for (int32_t i = 0; i < out_num; ++i) {
				ContactPoint *c = m->contacts + i;

				FeaturePair pair = out[i].f;
				if (flip) {
					uint8_t tmp;
					tmp = pair.in_i;  pair.in_i  = pair.in_r;  pair.in_r  = tmp;
					tmp = pair.out_i; pair.out_i = pair.out_r; pair.out_r = tmp;
				}

				c->fp          = pair;
				c->position    = out[i].v;
				c->penetration = depths[i];
			}
		}
		(void)s_max;
	}
	else {
		n = atx.rotation.transform(n);

		if (n.dot(btx.position - atx.position) < 0) {
			n = -n;
		}

		Vector3 pa, qa, pb, qb;
		supportEdge(atx, e_a, n,  &pa, &qa);
		supportEdge(btx, e_b, -n, &pb, &qb);

		Vector3 ca, cb;
		edgesContact(&ca, &cb, pa, qa, pb, qb);

		m->normal        = n;
		m->contact_count = 1;

		ContactPoint *c = m->contacts;
		FeaturePair pair;
		pair.key       = axis;
		c->fp          = pair;
		c->penetration = s_max;
		c->position    = (ca + cb) * 0.5_fp;
	}
}


/* Tier 2: single-contact pairs (sphere/capsule). Normal points A→B. Ported
   from the old engine, which handled every non-box shape by transforming into
   box-local space and leveraging the AABB primitives. */


/* The unit vector along d, or the fallback when d is too short to have
   one. dist is |d|, already computed by the caller. */
static inline Vector3 collision_direction(const Vector3 &d, Fixed dist, const Vector3 &fallback)
{
	return (dist > 0) ? d * (1.0_fp / dist) : fallback;
}


void sphereToSphere(ContactManifold *m, PhysicsShape *a, PhysicsShape *b)
{
	Transform atx = a->body->tx * a->local;
	Transform btx = b->body->tx * b->local;

	Vector3 d     = btx.position - atx.position;
	Fixed   rsum  = a->sphere.radius + b->sphere.radius;
	Fixed   dist2 = d.dot(d);
	if (dist2 > rsum * rsum) return;

	Fixed dist = sqrt(dist2);
	Vector3 n = collision_direction(d, dist, { 0.0_fp, 0.0_fp, 1.0_fp });

	m->normal        = n;
	m->contact_count = 1;
	ContactPoint *c  = m->contacts;
	c->position      = atx.position + n * a->sphere.radius;
	c->penetration   = dist - rsum;
	c->fp.key        = 0;
}


/* Sphere is A, box is B. */
void sphereToBox(ContactManifold *m, PhysicsShape *a, PhysicsShape *b)
{
	Transform stx = a->body->tx * a->local;
	Transform btx = b->body->tx * b->local;

	Fixed   r = a->sphere.radius;
	Vector3 e = b->box.e;
	AABB    box_local = { -e, e };

	/* Sphere center in box-local space. Closest point on the local AABB. */
	Vector3 s_local = btx.mulVectorTransposed(stx.position);
	Vector3 c_local = box_local.closestToPoint(s_local);
	Vector3 d_local = s_local - c_local;
	Fixed   dist2   = d_local.dot(d_local);
	if (dist2 > r * r) return;

	/* Bring the contact point back to world, build normal sphere→box. */
	Vector3 c_world = btx.mulVector(c_local);
	Vector3 to_box  = c_world - stx.position;
	Fixed   len     = to_box.magnitude();
	Vector3 normal  = collision_direction(to_box, len, { 0.0_fp, 0.0_fp, 1.0_fp });

	m->normal        = normal;
	m->contact_count = 1;
	ContactPoint *c  = m->contacts;
	c->position      = c_world;
	c->penetration   = len - r;   /* negative = overlap */
	c->fp.key        = 0;
}


/* Capsule is A, box is B. */
void capsuleToBox(ContactManifold *m, PhysicsShape *a, PhysicsShape *b)
{
	Transform atx = a->body->tx * a->local;
	Transform btx = b->body->tx * b->local;

	capsuleToStaticBox(m, &a->capsule, &atx, &b->box, &btx);
}


/* Sphere is A, capsule is B. Treat the capsule as its segment plus radius:
   closest-point-on-segment reduces the pair to sphere-vs-sphere. */
void sphereToCapsule(ContactManifold *m, PhysicsShape *a, PhysicsShape *b)
{
	Transform stx = a->body->tx * a->local;
	Transform ctx = b->body->tx * b->local;

	Fixed sr = a->sphere.radius;
	Fixed cr = b->capsule.radius;

	Vector3 bot, top;
	b->capsule.getSegment(ctx, &bot, &top);

	Vector3 on_seg = segment_closestToPoint(bot, top, stx.position);
	Vector3 d      = on_seg - stx.position;
	Fixed   rsum   = sr + cr;
	Fixed   dist2  = d.dot(d);
	if (dist2 > rsum * rsum) return;

	Fixed dist = sqrt(dist2);
	Vector3 n = collision_direction(d, dist, { 0.0_fp, 0.0_fp, 1.0_fp });

	m->normal        = n;
	m->contact_count = 1;
	ContactPoint *c  = m->contacts;
	c->position      = stx.position + n * sr;
	c->penetration   = dist - rsum;
	c->fp.key        = 0;
}


/* Closest point between the two inner segments. */
void capsuleToCapsule(ContactManifold *m, PhysicsShape *a, PhysicsShape *b)
{
	Transform atx = a->body->tx * a->local;
	Transform btx = b->body->tx * b->local;

	Fixed ra = a->capsule.radius;
	Fixed rb = b->capsule.radius;

	Vector3 pa1, pa2, pb1, pb2;
	a->capsule.getSegment(atx, &pa1, &pa2);
	b->capsule.getSegment(btx, &pb1, &pb2);

	/* Iterate closest-point both ways a couple of times: good enough here. */
	Vector3 ca = pa1;
	Vector3 cb = segment_closestToPoint(pb1, pb2, ca);
	ca = segment_closestToPoint(pa1, pa2, cb);
	cb = segment_closestToPoint(pb1, pb2, ca);

	Vector3 d    = cb - ca;
	Fixed   rsum = ra + rb;
	Fixed   d2   = d.dot(d);
	if (d2 > rsum * rsum) return;

	Fixed dist = sqrt(d2);
	Vector3 n = collision_direction(d, dist, { 0.0_fp, 0.0_fp, 1.0_fp });

	m->normal        = n;
	m->contact_count = 1;
	ContactPoint *c  = m->contacts;
	c->position      = ca + n * ra;
	c->penetration   = dist - rsum;
	c->fp.key        = 0;
}


/* Without a RigidBody: static geometry placed by a world transform. Capsule
   is A, box is B. */
void capsuleToStaticBox(ContactManifold *m, const Capsule *capsule, const Transform *capsule_world,
                        const Box *box, const Transform *box_world)
{
	Fixed r = capsule->radius;

	/* Capsule segment endpoints in box-local space. */
	Vector3 bot_world, top_world;
	capsule->getSegment(*capsule_world, &bot_world, &top_world);
	Vector3 top_local = box_world->mulVectorTransposed(top_world);
	Vector3 bot_local = box_world->mulVectorTransposed(bot_world);

	Vector3 e = box->e;
	AABB    box_local = { -e, e };

	Vector3 c_on_box = box_local.closestToSegment(bot_local, top_local);
	Vector3 c_on_seg = segment_closestToPoint(bot_local, top_local, c_on_box);
	Vector3 d_local  = c_on_seg - c_on_box;
	Fixed   dist2    = d_local.dot(d_local);
	if (dist2 > r * r) return;

	/* Contact lives on the box surface; normal goes capsule→box. */
	Vector3 box_point = box_world->mulVector(c_on_box);
	Vector3 seg_point = box_world->mulVector(c_on_seg);
	Vector3 to_box    = box_point - seg_point;
	Fixed   len       = to_box.magnitude();
	Vector3 normal    = collision_direction(to_box, len, { 0.0_fp, 0.0_fp, 1.0_fp });

	m->normal        = normal;
	m->contact_count = 1;
	ContactPoint *c  = m->contacts;
	c->position      = box_point;
	c->penetration   = len - r;
	c->fp.key        = 0;
}


/* Without a RigidBody. Capsule is A, sphere is B. */
void capsuleToStaticSphere(ContactManifold *m, const Capsule *capsule, const Transform *capsule_world,
                           const Sphere *sphere, const Transform *sphere_world)
{
	Vector3 top, bot;
	capsule->getSegment(*capsule_world, &bot, &top);

	Vector3 on_seg = segment_closestToPoint(bot, top, sphere_world->position);
	Vector3 d      = sphere_world->position - on_seg;
	Fixed   rsum   = capsule->radius + sphere->radius;
	Fixed   dist2  = d.dot(d);
	if (dist2 > rsum * rsum) return;

	Fixed dist = sqrt(dist2);
	Vector3 n = collision_direction(d, dist, { 0.0_fp, 0.0_fp, 1.0_fp });

	m->normal        = n;
	m->contact_count = 1;
	ContactPoint *c  = m->contacts;
	c->position      = sphere_world->position - n * sphere->radius;
	c->penetration   = dist - rsum;
	c->fp.key        = 0;
}


/* Without a RigidBody. A is the moving capsule. */
void capsuleToStaticCapsule(ContactManifold *m, const Capsule *capsule, const Transform *capsule_world,
                            const Capsule *other, const Transform *other_world)
{
	Vector3 pa1, pa2, pb1, pb2;
	capsule->getSegment(*capsule_world, &pa1, &pa2);
	other->getSegment(*other_world, &pb1, &pb2);

	Vector3 ca, cb;
	segment_closestToSegment(pa1, pa2, pb1, pb2, &ca, &cb);

	Vector3 d    = cb - ca;
	Fixed   rsum = capsule->radius + other->radius;
	Fixed   d2   = d.dot(d);
	if (d2 > rsum * rsum) return;

	Fixed dist = sqrt(d2);
	Vector3 n = collision_direction(d, dist, { 0.0_fp, 0.0_fp, 1.0_fp });

	m->normal        = n;
	m->contact_count = 1;
	ContactPoint *c  = m->contacts;
	c->position      = cb - n * other->radius;
	c->penetration   = dist - rsum;
	c->fp.key        = 0;
}


/* The capsule is A, the triangle is B. Reference point on the segment via
   plane intersection, then one closest-point refinement. */
void capsuleToTriangle(ContactManifold *m, const Capsule *capsule, const Transform *world,
                       const Triangle *triangle)
{
	const Vector3 *vertices        = triangle->vertices;
	const Vector3 &triangle_normal = triangle->normal;

	Fixed r = capsule->radius;

	Vector3 bot, top;
	capsule->getSegment(*world, &bot, &top);

	/* Reference point: where the segment crosses the triangle plane. */
	Vector3 seg   = top - bot;
	Vector3 to_v0 = vertices[0] - bot;
	Fixed   denom = triangle_normal.dot(seg);
	Fixed   t     = 0;
	if (denom.raw() != 0)
		t = clamp(triangle_normal.dot(to_v0) / denom, 0.0_fp, 1.0_fp);
	Vector3 ref = bot;
	ref.addScaledVector(seg, t);

	Vector3 tri_pt = triangle_closestToPoint(vertices[0], vertices[1], vertices[2], ref);
	Vector3 center = segment_closestToPoint(bot, top, tri_pt);
	tri_pt = triangle_closestToPoint(vertices[0], vertices[1], vertices[2], center);

	Vector3 d     = tri_pt - center;
	Fixed   dist2 = d.dot(d);
	if (dist2 > r * r) return;

	Fixed dist = sqrt(dist2);
	Vector3 n = collision_direction(d, dist, -triangle_normal);

	m->normal        = n;
	m->contact_count = 1;
	ContactPoint *c  = m->contacts;
	c->position      = center + n * r;
	c->penetration   = dist - r;
	c->fp.key        = 0;
}


/*
	Contact normal correction on inactive triangle edges, ported from Jolt's
	ActiveEdges::FixNormal and ClosestPoint::GetBaryCentricCoordinates
	(JoltPhysics, MIT licensed). A contact landing on an internal seam of the
	mesh carries a normal pointing from the edge to the capsule axis instead
	of the surface normal; here it is replaced by the face normal, so sliding
	over a triangulated floor or ramp does not read as hitting a wall.
*/

/* Barycentric coordinates of the origin inside triangle (a, b, c), built on
   the two shortest edges for accuracy. False on a degenerate triangle — the
   importer drops those, so it cannot trigger on baked meshes. */
static bool collision_baryCentric(const Vector3 &a, const Vector3 &b, const Vector3 &c,
                                  Fixed *u, Fixed *v, Fixed *w)
{
	Vector3 v0 = b - a;
	Vector3 v1 = c - a;
	Vector3 v2 = c - b;

	Fixed d00 = v0.dot(v0);
	Fixed d11 = v1.dot(v1);
	Fixed d22 = v2.dot(v2);

	if (d00 <= d22) {
		Fixed d01 = v0.dot(v1);
		Fixed denominator = d00 * d11 - d01 * d01;
		if (denominator <= 0) return false;

		Fixed a0 = a.dot(v0);
		Fixed a1 = a.dot(v1);
		*v = (d01 * a1 - d11 * a0) / denominator;
		*w = (d01 * a0 - d00 * a1) / denominator;
		*u = 1.0_fp - *v - *w;
	}
	else {
		Fixed d12 = v1.dot(v2);
		Fixed denominator = d11 * d22 - d12 * d12;
		if (denominator <= 0) return false;

		Fixed c1 = c.dot(v1);
		Fixed c2 = c.dot(v2);
		*u = (d22 * c1 - d12 * c2) / denominator;
		*v = (d11 * c2 - d12 * c1) / denominator;
		*w = 1.0_fp - *u - *v;
	}
	return true;
}

/* normal and the returned normal follow the manifold convention: unit, from
   the capsule toward the surface. point is the contact point on the triangle,
   in the same space as the triangle. movement_direction is the capsule's
   motion and may be zero; it separates sliding over a seam (use the face
   normal) from grazing a real wall through an inactive edge (keep the
   contact normal, the face normal would bounce the capsule back). */
Vector3 collision_fixTriangleNormal(const Triangle *triangle, const Vector3 &point,
                                    const Vector3 &normal, const Vector3 &movement_direction)
{
	/* All edges active: the normal is already correct. */
	if ((triangle->active_edges & 0x7) == 0x7) return normal;

	/* Face normal in the manifold's convention: toward the surface. */
	Vector3 face_normal = -triangle->normal;

	/* If normal would affect movement less than the face normal, keep it. */
	if (movement_direction.dot(normal) < movement_direction.dot(face_normal))
		return normal;

	/* None of the edges are active: the face normal is the only real one. */
	if (triangle->active_edges == 0) return face_normal;

	/* Some edges are active. Parallel to the face: no need to check them. */
	if (face_normal.dot(normal) > 0.999848_fp)   /* cos(1 degree) */
		return normal;

	const Fixed epsilon = TOLERANCE;
	const Fixed one_minus_epsilon = 1.0_fp - epsilon;

	/* Where the contact point sits in the triangle: vertex, edge or interior.
	   The coordinates are of the origin relative to the shifted vertices. */
	Vector3 a = triangle->vertices[0] - point;
	Vector3 b = triangle->vertices[1] - point;
	Vector3 c = triangle->vertices[2] - point;

	Fixed u, v, w;
	if (!collision_baryCentric(a, b, c, &u, &v, &w)) return face_normal;

	uint8_t colliding_edge;
	if      (u > one_minus_epsilon) colliding_edge = 0x5;   /* vertex v0: edge 0 or 2 */
	else if (v > one_minus_epsilon) colliding_edge = 0x3;   /* vertex v1: edge 0 or 1 */
	else if (w > one_minus_epsilon) colliding_edge = 0x6;   /* vertex v2: edge 1 or 2 */
	else if (u < epsilon)           colliding_edge = 0x2;   /* edge v1v2 */
	else if (v < epsilon)           colliding_edge = 0x4;   /* edge v2v0 */
	else if (w < epsilon)           colliding_edge = 0x1;   /* edge v0v1 */
	else return face_normal;                                /* interior hit */

	return (triangle->active_edges & colliding_edge) ? normal : face_normal;
}


/* Convex shape against a static triangle mesh.

   A mesh is not a convex piece, so it cannot be fed to the SAT routines above.
   Instead the shape's AABB queries the mesh tree and every triangle it touches
   is resolved on its own, with the results merged into the one manifold the
   contact holds. That manifold carries a single normal, so the deepest contact
   sets it: on a floor or a ramp all the triangles agree anyway, and where they
   do not, the deepest one is the constraint that matters. */

#define MESH_QUERY_MAX 24

struct MeshQuery
{
	const CollisionMesh *mesh;
	int32_t              triangle[MESH_QUERY_MAX];
	int32_t              count;
};


static int collision_collectTriangle(void *cb, int32_t id)
{
	MeshQuery *query = (MeshQuery *)cb;
	if (query->count >= MESH_QUERY_MAX) return 0;

	query->triangle[query->count++] = (int32_t)(intptr_t)query->mesh->tree.userData(id);
	return 1;
}


/* Closest point on the triangle to the sphere centre: exact, and the only
   test a sphere needs. */
static void sphereToTriangle(ContactManifold *m, const Sphere *sphere, const Vector3 &center,
                             const Triangle *triangle)
{
	Vector3 closest = triangle_closestToPoint(triangle->vertices[0], triangle->vertices[1],
	                                          triangle->vertices[2], center);
	Vector3 d     = closest - center;
	Fixed   dist2 = d.dot(d);
	Fixed   r     = sphere->radius;

	if (dist2 > r * r) return;

	Fixed dist = sqrt(dist2);
	Vector3 n = collision_direction(d, dist, -triangle->normal);

	m->normal        = n;
	m->contact_count = 1;
	m->contacts[0].position    = closest;
	m->contacts[0].penetration = dist - r;
	m->contacts[0].fp.key      = 0;
}


/* Box corners against the triangle's plane. A resting box meets a floor
   triangle with its whole face, so this yields the several points a stack
   needs to stay up; a box caught on a bare edge gets a coarser answer than
   full SAT would give, which is the trade for keeping this cheap. */
static void boxToTriangle(ContactManifold *m, const Box *box, const Transform *world,
                          const Triangle *triangle)
{
	m->contact_count = 0;
	m->normal        = -triangle->normal;

	const Vector3 &v0 = triangle->vertices[0];

	for (int i = 0; i < 8; i++) {
		Vector3 local = {
			(i & 1) ? box->e.x : -box->e.x,
			(i & 2) ? box->e.y : -box->e.y,
			(i & 4) ? box->e.z : -box->e.z,
		};
		Vector3 corner = world->mulVector(local);

		/* Signed distance to the plane; only corners behind it touch. */
		Fixed depth = triangle->normal.dot(corner - v0);
		if (depth >= 0) continue;

		/* Behind the plane is not enough: it has to be behind the face. */
		Vector3 on_plane = corner - triangle->normal * depth;

		Vector3 closest = triangle_closestToPoint(triangle->vertices[0], triangle->vertices[1],
		                                          triangle->vertices[2], on_plane);
		Vector3 slip = closest - on_plane;
		if (slip.dot(slip) > TOLERANCE) continue;

		if (m->contact_count >= 8) break;

		ContactPoint *c = &m->contacts[m->contact_count++];
		c->position    = corner;
		c->penetration = depth;
		c->fp.key      = (uint32_t)i;
	}
}


/* Runs the shape against every triangle its AABB reaches and merges the hits.
   The mesh tree is in mesh-local space, so the shape goes in through the
   inverse of the mesh's world transform, and the contact points and the
   normal come back out through it. */
static void shapeToMesh(ContactManifold *m, PhysicsShape *shape, PhysicsShape *mesh_shape)
{
	/* The contact manager runs computeBasis on this normal without checking
	   the contact count, so it must always hold a unit vector: uninitialised
	   memory blows up on the normalise, and so does a zero vector. */
	m->contact_count = 0;
	m->normal        = { 0.0_fp, 0.0_fp, 1.0_fp };

	const CollisionMesh *mesh = mesh_shape->mesh;
	if (mesh == NULL) return;

	Transform mesh_world = mesh_shape->local;
	if (mesh_shape->body) mesh_world = mesh_shape->body->tx * mesh_shape->local;

	Transform shape_world = shape->local;
	if (shape->body) shape_world = shape->body->tx * shape->local;

	Transform local_world = mesh_world.productTransposed(shape_world);

	/* The shape's bound in mesh-local space, straight from its primitive:
	   going through PhysicsShape::computeAABB would read the cached world
	   transform instead of this one. */
	AABB aabb __attribute__((uninitialized));
	switch (shape->type) {
		case SHAPE_SPHERE:  aabb = shape->sphere.computeAABB (local_world); break;
		case SHAPE_BOX:     aabb = shape->box.computeAABB    (local_world); break;
		case SHAPE_CAPSULE: aabb = shape->capsule.computeAABB(local_world); break;
		default:            return;
	}

	/* count gates the triangle array; the designated initializer would zero
	   all of it. */
	MeshQuery query __attribute__((uninitialized));
	query.mesh  = mesh;
	query.count = 0;
	mesh->queryAABB(&query, collision_collectTriangle, aabb);

	/* Starts at infinity, not at zero: a contact that merely touches still has
	   to set the normal, or the deepest-wins test below never fires. */
	Fixed deepest = FIXED_MAX;

	for (int32_t t = 0; t < query.count; t++) {
		Triangle triangle = mesh->triangle(query.triangle[t]);

		/* contact_count gates the hit: normal, position, penetration and key
		   are written by each routine whenever it sets a contact, and only
		   those four are read below. Zeroing the whole manifold was a
		   476-byte memset per candidate triangle. */
		ContactManifold hit __attribute__((uninitialized));
		hit.contact_count = 0;

		switch (shape->type) {
			case SHAPE_SPHERE:
				sphereToTriangle(&hit, &shape->sphere, local_world.position, &triangle);
				break;
			case SHAPE_BOX:
				boxToTriangle(&hit, &shape->box, &local_world, &triangle);
				break;
			case SHAPE_CAPSULE:
				capsuleToTriangle(&hit, &shape->capsule, &local_world, &triangle);
				break;
			default:
				break;
		}

		for (int32_t i = 0; i < hit.contact_count; i++) {
			if (m->contact_count >= 8) break;

			ContactPoint *c = &m->contacts[m->contact_count];
			c->position    = mesh_world.mulVector(hit.contacts[i].position);
			c->penetration = hit.contacts[i].penetration;
			/* Key by triangle so warm starting can match points across steps. */
			c->fp.key      = ((uint32_t)query.triangle[t] << 4) | (hit.contacts[i].fp.key & 0xF);
			m->contact_count++;

			if (c->penetration < deepest) {
				deepest   = c->penetration;
				m->normal = mesh_world.rotation.transform(hit.normal);
			}
		}
	}
}


/* Type-based dispatcher. */
void collision(ContactManifold *m, PhysicsShape *a, PhysicsShape *b)
{
	ShapeType ta = a->type;
	ShapeType tb = b->type;

	if (ta == SHAPE_BOX && tb == SHAPE_BOX) {
		boxToBox(m, a, b);
	}
	else if (ta == SHAPE_SPHERE && tb == SHAPE_SPHERE) {
		sphereToSphere(m, a, b);
	}
	else if (ta == SHAPE_CAPSULE && tb == SHAPE_CAPSULE) {
		capsuleToCapsule(m, a, b);
	}
	else if (ta == SHAPE_SPHERE && tb == SHAPE_BOX) {
		sphereToBox(m, a, b);
	}
	else if (ta == SHAPE_BOX && tb == SHAPE_SPHERE) {
		sphereToBox(m, b, a);
		m->normal = -m->normal;
	}
	else if (ta == SHAPE_SPHERE && tb == SHAPE_CAPSULE) {
		sphereToCapsule(m, a, b);
	}
	else if (ta == SHAPE_CAPSULE && tb == SHAPE_SPHERE) {
		sphereToCapsule(m, b, a);
		m->normal = -m->normal;
	}
	else if (ta == SHAPE_CAPSULE && tb == SHAPE_BOX) {
		capsuleToBox(m, a, b);
	}
	else if (ta == SHAPE_BOX && tb == SHAPE_CAPSULE) {
		capsuleToBox(m, b, a);
		m->normal = -m->normal;
	}
	/* Mesh is static-only, so the pair always has one convex side. */
	else if (tb == SHAPE_MESH) {
		shapeToMesh(m, a, b);
	}
	else if (ta == SHAPE_MESH) {
		shapeToMesh(m, b, a);
		/* No contact means no normal was written: inverting it would be
		   reading whatever the manifold happened to hold. */
		if (m->contact_count) m->normal = -m->normal;
	}
}
