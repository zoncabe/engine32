#ifndef ENGINE_32_CAMERA_H
#define ENGINE_32_CAMERA_H

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_trig.h"
#include "e32_spring_arm.h"


typedef enum {

	CAMERA_TYPE_SPRING_ARM,
	CAMERA_TYPE_COUNT,
	CAMERA_TYPE_NONE,

} CameraType;


struct CameraDef
{
	CameraType type;

	/* Vertical. */
	Angle field_of_view;
	Fixed near_clipping;
	Fixed far_clipping;

	/* Refits near/far every frame to the arm: nearest in, farthest out.
	   Off keeps the fixed planes above. */
	bool auto_clipping;

	CameraSpringArmDef spring_arm;
};

/* Sane lens bounds. Below the minimum the projection distance grows past
   what the GTE's 16 bit register holds; above the maximum the image is
   unusable anyway. */
constexpr Angle CAMERA_FOV_MIN =  10.0_deg;
constexpr Angle CAMERA_FOV_MAX = 120.0_deg;

struct Scene3D;

struct Camera
{
	Vector3 position;
	Vector3 target;

	/* Same split as the arm: the stick writes the target, the aim adds its
	   offset, and the lens chases the sum. */
	Angle target_field_of_view;
	Angle field_of_view;
	Fixed near_clipping;
	Fixed far_clipping;

	/* The def's planes, kept apart: the clipping method moves the live
	   ones starting from these. */
	Fixed base_near_clipping;
	Fixed base_far_clipping;

	bool  auto_clipping;   /* the clipping method refits the planes each frame */

	/* view target transition: the outgoing center is frozen at switch time, so
	   the old target moving afterwards cannot disturb the blend */
	Vector3 blend_from;
	Fixed   blend_elapsed;
	Fixed   blend_duration;

	CameraType type;

	/* The buttons that move this camera, or NULL when none name it. Wired
	   when the state is loaded, from the controls that state declared. */
	const struct CameraControlBinding *binding;

	struct {
		CameraSpringArmSettings settings;
		CameraSpringArmData     data;
	} spring_arm;


	void init();
	void reset();
	void update(Vector3 *center, const Scene3D *scene, Fixed dt);
	void setViewTarget(const Vector3 *from, Fixed duration);
	Angle getAngleAround(const Vector3 *point) const;
	Angle getPitch() const;
	Vector3 getRight() const;
	void fitClipping(const Scene3D *scene);
};


#endif
