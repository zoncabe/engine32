#include "physics/shapes/e32_capsule.h"
#include "physics/shapes/e32_physics_shape.h"   /* MassData */
#include "physics/math/e32_math_common.h"        /* PI, sqrt */
#include "physics/math/e32_math_functions.h"     /* segment_closestToPoint */


void Capsule::getSegment(const Transform &world, Vector3 *a, Vector3 *b) const
{
	Vector3 local_top    = { 0.0_fp, 0.0_fp,  half_height };
	Vector3 local_bottom = { 0.0_fp, 0.0_fp, -half_height };
	*a = world.mulVector(local_bottom);
	*b = world.mulVector(local_top);
}


bool Capsule::testPoint(const Transform &world, const Vector3 &p) const
{
	Vector3 a, b;
	getSegment(world, &a, &b);
	Vector3 closest = segment_closestToPoint(a, b, p);
	Vector3 d       = p - closest;
	return d.squaredMagnitude() <= radius * radius;
}


bool Capsule::raycast(const Transform &world, RaycastData *ray) const
{
	/* Cheap: enclose the capsule in a sphere at its center with radius = radius + half_height.
	   Good enough for broad raycast; exact capsule-ray iterates cylinder+caps. */
	Fixed   R  = radius + half_height;
	Vector3 m  = ray->start - world.position;
	Fixed   b  = m.dot(ray->dir);
	Fixed   cc = m.dot(m) - R * R;
	if (cc > 0 && b > 0) return false;
	Fixed disc = b * b - cc;
	if (disc < 0) return false;
	Fixed t = -b - sqrt(disc);
	if (t < 0) t = 0;
	if (t > ray->t) return false;

	ray->toi    = t;
	Vector3 hit = ray->start + ray->dir * t;
	ray->normal = (hit - world.position).normalized();
	return true;
}


AABB Capsule::computeAABB(const Transform &world) const
{
	Vector3 a, b;
	getSegment(world, &a, &b);
	Vector3 r  = { radius, radius, radius };
	Vector3 mn = { a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z };
	Vector3 mx = { a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z };
	return { mn - r, mx + r };
}


void Capsule::computeMass(const Transform &local, Fixed density, MassData *md) const
{
	Fixed r  = radius;
	Fixed r2 = r * r;
	Fixed h  = half_height * 2;

	Fixed cyl_vol = PI * r2 * h;
	Fixed cap_vol = PI * r2 * r * 4 / 3;
	Fixed mass    = (cyl_vol + cap_vol) * density;

	Fixed cyl_mass = cyl_vol * density;
	Fixed cap_mass = cap_vol * density;

	/* Solid cylinder around its own axis = (1/2) m r²; perpendicular = (1/12) m (3r² + h²). */
	Fixed Izz = cyl_mass * r2 * 0.5_fp + cap_mass * r2 * 0.4_fp;
	Fixed Ixx = cyl_mass * ( h * h / 12 + r2 * 0.25_fp )
	          + cap_mass * ( r2 * 0.4_fp + h * h * 0.5_fp + r * h * 0.375_fp );
	Fixed Iyy = Ixx;

	Matrix3 I = Matrix3::diagonal(Ixx, Iyy, Izz);

	/* Rotate to local frame. */
	I = local.rotation * I * local.rotation.transposed();

	/* Parallel axis: I += m · (|pos|² · identity - pos⊗pos). */
	Fixed dot = local.position.dot(local.position);
	I += (Matrix3::identity() * dot - Matrix3::outerProduct(local.position, local.position)) * mass;

	md->center  = local.position;
	md->inertia = I;
	md->mass    = mass;
}


void CapsuleDef::init()
{
	tx.init();
	radius      = 0.5_fp;
	half_height = 0.5_fp;
	friction    = 0.4_fp;
	restitution = 0.0_fp;
	density     = 1.0_fp;
	sensor      = 0;
}

void CapsuleDef::set(const Transform &t, Fixed r, Fixed hh)
{
	tx          = t;
	radius      = r;
	half_height = hh;
}
