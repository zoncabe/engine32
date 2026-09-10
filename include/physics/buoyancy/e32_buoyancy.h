/*
	Archimedes buoyancy over the bodies inside a water sensor.

	The water is a sensor shape on a static body: the broadphase keeps its
	overlap pairs alive and the island solver already skips sensor contacts,
	so the sensor's contact list is exactly the set of bodies in the water.
	Each step, every dynamic body on that list gets the buoyant force of its
	submerged volume, plus linear and angular drag.

	The surface height is a callback so the wave animation stays the single
	source of truth: the water module owns the wave sum, this module only
	asks how high the surface sits over a given (x, y).
*/
#ifndef ENGINE_32_BUOYANCY_H
#define ENGINE_32_BUOYANCY_H

#include "physics/math/e32_vector3.h"


struct PhysicsWorld;
struct RigidBody;


typedef Fixed (*BuoyancySurfaceHeightFn)(const void *surface, Fixed x, Fixed y);


struct BuoyancyVolume
{
	RigidBody *body;             /* static body carrying the water sensor shape */

	Fixed density;               /* water, kg/m3 */
	Fixed linear_drag;           /* per-second rate on the linear velocity */
	Fixed angular_drag;          /* per-second rate on the angular velocity */

	BuoyancySurfaceHeightFn surface_height;
	const void             *surface;

	/* Pushes every dynamic body inside the sensor, once per step. */
	void apply(const PhysicsWorld *world) const;
};


#endif
