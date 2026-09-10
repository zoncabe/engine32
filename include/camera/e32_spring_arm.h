#ifndef ENGINE_32_SPRING_ARM_H
#define ENGINE_32_SPRING_ARM_H

#include "physics/math/e32_vector2.h"
#include "physics/math/e32_vector3.h"
#include "physics/math/e32_trig.h"


struct Camera;

/* Hard zoom-out ceiling, in meters. */
constexpr Fixed SPRING_ARM_MAX_LENGTH = 80.0_fp;

struct CameraSpringArmSettings
{
	/* Per second. The velocities are angles per second; x is the yaw, y
	   the pitch. */
	Vector2 response_rate;
	Vector2 max_velocity;
	Vector2 direction;

	Fixed zoom_response_rate;

	Fixed distance_speed;   /* meters per second */
	Fixed fov_speed;        /* angle per second */

	Angle max_pitch;
	Angle min_pitch;

	/* What the aim adds to whatever the player has the arm set to, so the free
	   distance and lens are never overwritten: negative pulls the arm in and
	   narrows the view. The swing is the one that scales instead of adding. */
	Fixed aim_arm_length;
	Fixed aim_side_offset;
	Angle aim_field_of_view;
	Fixed aim_velocity_scale;
};


/* Where the arm is right now: seeded from the def, moved by the engine. */
struct CameraSpringArmData
{
	/* What the arm is asked for and where it actually is. The stick writes the
	   target, the aim adds its offset on top, and the arm chases the sum: the
	   free distance survives an aim because nothing ever writes over it. */
	Fixed target_arm_length;
	Fixed arm_length;

	Fixed target_side_offset;
	Fixed side_offset;

	Angle yaw;
	Angle pitch;

	Fixed pivot_height;

	Vector2 velocity;
	Vector2 target_velocity;
};


struct CameraSpringArmDef
{
	Fixed arm_length;
	Fixed side_offset;

	Angle yaw;
	Angle pitch;
	Fixed pivot_height;

	CameraSpringArmSettings settings;
};


class CameraSpringArm
{
public:

	static void init(Camera *camera, const CameraSpringArmDef *def);
	static void update(Camera *camera, Vector3 *center, Fixed dt);

	/* The arm's two control-driven angles. All three answer zero on a
	   camera that is not an arm, which is what a caller that needs them
	   reads as nothing to do. */
	static Angle getPitch(const Camera *camera);
	static Angle getYaw(const Camera *camera);
	static Fixed getLength(const Camera *camera);
};


#endif
