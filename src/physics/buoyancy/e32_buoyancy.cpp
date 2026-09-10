/*
	Archimedes buoyancy over the bodies inside a water sensor.

	The bodies in the water come from the sensor's own contact list: the
	broadphase keeps those pairs alive and the narrowphase marks them
	COLLIDING, the island solver just never resolves them. No body scan.

	Each shape is sampled as one or more vertical columns. A column's
	submerged fraction is measured against the surface height at its own
	(x, y), so a wave lifting one end of a box and not the other produces
	the torque that rocks it; the same offset is what rights a tilted
	floating body, since the deeper samples push harder. The sphere keeps
	its closed-form spherical cap instead of a column.

	Forces are accumulated per body and applied before the islands are
	built, so they integrate in the same step. applyLinearForce takes an
	acceleration (it scales by mass), applyTorque takes a real torque.
*/
#include <stddef.h>

#include "physics/buoyancy/e32_buoyancy.h"
#include "physics/world/e32_physics_world.h"
#include "physics/body/e32_rigid_body.h"
#include "physics/collision/e32_contact.h"
#include "physics/math/e32_math_common.h"


/* Sample columns per box shape: the minimum that tilts on both axes. */
#define BUOYANCY_BOX_COLUMNS 4


/* Fraction of a vertical column [bottom, top] sitting under the surface. */
static Fixed buoyancy_columnFraction(Fixed bottom, Fixed top, Fixed surface)
{
	if (surface <= bottom) return 0;
	if (surface >= top)    return 1.0_fp;
	return (surface - bottom) / (top - bottom);
}

/* Submerged fraction of a sphere: spherical cap of height h over the full
   volume, h measured from the bottom of the sphere up to the surface. */
static Fixed buoyancy_sphereFraction(Fixed center_z, Fixed radius, Fixed surface)
{
	Fixed h = surface - (center_z - radius);
	if (h <= 0)          return 0;
	if (h >= radius * 2) return 1.0_fp;
	return h * h * (radius * 3 - h) / (radius * radius * radius * 4);
}


struct BuoyancyAccum
{
	Vector3 force;       /* real force, world space */
	Vector3 torque;      /* real torque about the center of mass */
	Fixed   volume;      /* displaceable volume of the whole body */
	Fixed   submerged;   /* submerged part of that volume */
};

/* One sample: a share of the shape's volume tested at a world point. Its
   buoyant force pushes against gravity and its arm from the center of mass
   is what turns uneven submersion into torque. */
static void buoyancy_addSample(BuoyancyAccum *accum, const BuoyancyVolume *volume,
                               const Vector3 &gravity, const Vector3 &center_of_mass,
                               const Vector3 &point, Fixed share_volume, Fixed fraction)
{
	accum->volume    += share_volume;
	accum->submerged += share_volume * fraction;
	if (fraction <= 0) return;

	Vector3 force = gravity * (-volume->density * share_volume * fraction);
	accum->force += force;

	accum->torque += (point - center_of_mass).cross(force);
}

static void buoyancy_sampleShape(BuoyancyAccum *accum, const BuoyancyVolume *volume,
                                 const Vector3 &gravity, const RigidBody *body,
                                 const PhysicsShape *shape)
{
	Transform tx = body->tx * shape->local;

	switch (shape->type) {
		case SHAPE_SPHERE: {
			Fixed r = shape->sphere.radius;
			Fixed v = PI * r * r * r * 4 / 3;

			Fixed surface  = volume->surface_height(volume->surface, tx.position.x, tx.position.y);
			Fixed fraction = buoyancy_sphereFraction(tx.position.z, r, surface);
			buoyancy_addSample(accum, volume, gravity, body->world_center, tx.position, v, fraction);
			break;
		}

		case SHAPE_BOX: {
			const Vector3 &e = shape->box.e;
			Fixed v = e.x * e.y * e.z * 8;

			/* World-z half-extent of the OBB: its slab along the vertical. */
			Fixed hz = tx.rotation.ex.z.abs() * e.x
			         + tx.rotation.ey.z.abs() * e.y
			         + tx.rotation.ez.z.abs() * e.z;

			/* Four columns halfway to the corners, a quarter volume each. */
			static const Fixed offset[BUOYANCY_BOX_COLUMNS][2] = {
				{ -0.5_fp, -0.5_fp }, { 0.5_fp, -0.5_fp }, { -0.5_fp, 0.5_fp }, { 0.5_fp, 0.5_fp },
			};
			for (int i = 0; i < BUOYANCY_BOX_COLUMNS; i++) {
				Vector3 local = { e.x * offset[i][0], e.y * offset[i][1], 0.0_fp };
				Vector3 point = tx.mulVector(local);

				Fixed surface  = volume->surface_height(volume->surface, point.x, point.y);
				Fixed fraction = buoyancy_columnFraction(point.z - hz, point.z + hz, surface);
				buoyancy_addSample(accum, volume, gravity, body->world_center, point,
				                   v / BUOYANCY_BOX_COLUMNS, fraction);
			}
			break;
		}

		case SHAPE_CAPSULE: {
			Fixed r  = shape->capsule.radius;
			Fixed hh = shape->capsule.half_height;
			Fixed v  = PI * r * r * (hh * 2) + PI * r * r * r * 4 / 3;

			/* One column of radius r at each end of the segment: exact when
			   the capsule floats on its side, approximate upright. */
			Vector3 end[2];
			shape->capsule.getSegment(tx, &end[0], &end[1]);

			for (int i = 0; i < 2; i++) {
				Fixed surface  = volume->surface_height(volume->surface, end[i].x, end[i].y);
				Fixed fraction = buoyancy_columnFraction(end[i].z - r, end[i].z + r, surface);
				buoyancy_addSample(accum, volume, gravity, body->world_center, end[i], v * 0.5_fp, fraction);
			}
			break;
		}

		case SHAPE_MESH:   /* static-only, never on a dynamic body */
			break;
	}
}

static void buoyancy_applyToBody(const PhysicsWorld *world, const BuoyancyVolume *volume, RigidBody *body)
{
	BuoyancyAccum accum = {};

	for (const PhysicsShape *shape = body->shapes; shape; shape = shape->next)
		buoyancy_sampleShape(&accum, volume, world->gravity, body, shape);

	if (accum.volume <= 0 || accum.submerged <= 0) return;

	Fixed fraction = accum.submerged / accum.volume;

	/* applyLinearForce multiplies by mass, so it is handed F/m. It also
	   wakes the body: floating under waves never settles, on purpose. */
	body->applyLinearForce(accum.force * body->inv_mass);
	body->applyTorque(accum.torque);

	/* Water drag, gated by how submerged the body is. The angular term uses
	   the mass as a stand-in for the inertia, so both coefficients stay
	   per-second rates independent of the body's size. */
	body->applyLinearForce(body->linear_velocity * (-volume->linear_drag * fraction));
	body->applyTorque(body->angular_velocity * (-volume->angular_drag * fraction * body->mass));
}

void BuoyancyVolume::apply(const PhysicsWorld *world) const
{
	if (body == NULL || surface_height == NULL) return;

	for (const ContactEdge *edge = body->contact_list; edge; edge = edge->next) {
		if (!(edge->constraint->flags & CONSTRAINT_COLLIDING)) continue;
		if (!(edge->other->flags & BODY_FLAG_DYNAMIC)) continue;

		buoyancy_applyToBody(world, this, edge->other);
	}
}
