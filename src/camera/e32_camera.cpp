#include "physics/math/e32_math_common.h"
#include "camera/e32_camera.h"
#include "camera/e32_spring_arm.h"


static void (*camera_handler[CAMERA_TYPE_COUNT])(Camera *, Vector3 *, Fixed) = {
	CameraSpringArm::update,   /* CAMERA_TYPE_SPRING_ARM */
};

void Camera::init()
{
	/* Lens and placement come from the scene's CameraDef; position and target
	   are one unit apart so the view matrix is not degenerate before the first
	   update places them. */
	*this = Camera();
	position = { 0.0_fp, 1.0_fp, 0.0_fp };
	target   = { 0.0_fp, 0.0_fp, 0.0_fp };
	type     = CAMERA_TYPE_NONE;
}

void Camera::reset()
{
	init();
}

/* horizontal yaw of the view direction, in the yaw convention of the engine;
   measured from the view (position -> target) so lateral offsets like the
   shoulder cancel out instead of skewing the angle */
Angle Camera::getAngleAround(const Vector3 *point) const
{
	(void)point;

	Fixed dx = target.x - position.x;
	Fixed dy = target.y - position.y;

	if (dx == 0 && dy == 0) return Angle();

	return Trig::atan2(dx, dy);
}

Angle Camera::getPitch() const
{
	return CameraSpringArm::getPitch(this);
}

/* The camera's right hand side on the ground plane. */
Vector3 Camera::getRight() const
{
	Vector3 flat = { target.x - position.x, target.y - position.y, 0.0_fp };

	if (flat.squaredMagnitude() < TOLERANCE) return { 0.0_fp, -1.0_fp, 0.0_fp };

	flat.normalize();

	/* Z is up, so right is forward crossed with it. */
	return { flat.y, -flat.x, 0.0_fp };
}


/* Starts a transition to whatever center the camera is fed next, gliding out of
   the given point. Passing a duration of zero cuts straight to the new target. */
void Camera::setViewTarget(const Vector3 *from, Fixed duration)
{
	blend_from     = *from;
	blend_elapsed  = Fixed();
	blend_duration = duration;
}

/* The near runs at a fifth of the arm, floored at the def's: the cut zone
   ends at 20% of the way to the pivot, so it never reaches the pieces in
   view. The far rides the arm whole, so the back plane keeps its authored
   distance past the pivot at any zoom. */
void Camera::fitClipping(const Scene3D *scene)
{
	(void)scene;

	if (!auto_clipping) return;
	if (type != CAMERA_TYPE_SPRING_ARM) return;

	Fixed arm  = spring_arm.data.arm_length;
	Fixed near = arm / 5;

	near_clipping = near > base_near_clipping ? near : base_near_clipping;
	far_clipping  = arm + base_far_clipping;
}

void Camera::update(Vector3 *center, const Scene3D *scene, Fixed dt)
{
	if (type == CAMERA_TYPE_NONE) return;

	Vector3 blended;

	if (blend_elapsed < blend_duration) {
		blend_elapsed += dt;

		Fixed t = blend_elapsed / blend_duration;
		if (t > 1) t = 1.0_fp;

		Fixed alpha = ease_cubic_in_out(t);

		blended = blend_from.lerp(*center, alpha);

		center = &blended;
	}

	camera_handler[type](this, center, dt);

	/* The flags read here are the previous frame's cull: the planes trail
	   the visibility by one frame. */
	fitClipping(scene);
}
