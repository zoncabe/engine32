#include "physics/shapes/e32_sphere.h"
#include "physics/shapes/e32_physics_shape.h"   /* MassData */
#include "physics/math/e32_math_common.h"        /* PI, sqrt */


bool Sphere::testPoint(const Transform &world, const Vector3 &p) const
{
	Vector3 d = p - world.position;
	return d.squaredMagnitude() <= radius * radius;
}


bool Sphere::raycast(const Transform &world, RaycastData *ray) const
{
	Vector3 m = ray->start - world.position;
	Fixed   b = m.dot(ray->dir);
	Fixed   c = m.dot(m) - radius * radius;

	if (c > 0 && b > 0) return false;
	Fixed disc = b * b - c;
	if (disc < 0) return false;

	Fixed t = -b - sqrt(disc);
	if (t < 0) t = 0;
	if (t > ray->t) return false;

	ray->toi    = t;
	Vector3 hit = ray->start + ray->dir * t;
	ray->normal = (hit - world.position).normalized();
	return true;
}


AABB Sphere::computeAABB(const Transform &world) const
{
	Vector3 r = { radius, radius, radius };
	return { world.position - r, world.position + r };
}


void Sphere::computeMass(const Transform &local, Fixed density, MassData *md) const
{
	Fixed r    = radius;
	Fixed r2   = r * r;
	Fixed vol  = PI * r2 * r * 4 / 3;
	Fixed mass = vol * density;

	/* Solid sphere: I = (2/5) · m · r² on each axis. */
	Fixed i = mass * r2 * 0.4_fp;
	Matrix3 I = Matrix3::diagonal(i, i, i);

	/* Parallel axis: I += m · (|c|² · I - c⊗c). */
	Fixed dot = local.position.dot(local.position);
	I += (Matrix3::identity() * dot - Matrix3::outerProduct(local.position, local.position)) * mass;

	md->center  = local.position;
	md->inertia = I;
	md->mass    = mass;
}


void SphereDef::init()
{
	tx.init();
	radius      = 0.5_fp;
	friction    = 0.4_fp;
	restitution = 0.2_fp;
	density     = 1.0_fp;
	sensor      = 0;
}

void SphereDef::set(const Transform &t, Fixed r)
{
	tx     = t;
	radius = r;
}
