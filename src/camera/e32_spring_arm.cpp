#include "physics/math/e32_math_common.h"
#include "camera/e32_camera.h"
#include "camera/e32_spring_arm.h"


Angle CameraSpringArm::getPitch(const Camera *camera)
{
	if (camera->type != CAMERA_TYPE_SPRING_ARM) return Angle();
	return camera->spring_arm.data.pitch;
}

Angle CameraSpringArm::getYaw(const Camera *camera)
{
	if (camera->type != CAMERA_TYPE_SPRING_ARM) return Angle();
	return camera->spring_arm.data.yaw;
}

Fixed CameraSpringArm::getLength(const Camera *camera)
{
	if (camera->type != CAMERA_TYPE_SPRING_ARM) return Fixed();
	return camera->spring_arm.data.arm_length;
}


static void setVelocity(Camera *camera, Fixed dt)
{
	CameraSpringArmData *data = &camera->spring_arm.data;
	const CameraSpringArmSettings *settings = &camera->spring_arm.settings;

	Fixed factor_x = expNeg(settings->response_rate.x * dt);
	Fixed factor_y = expNeg(settings->response_rate.y * dt);
	data->velocity.x = data->velocity.x * factor_x + data->target_velocity.x * (1 - factor_x);
	data->velocity.y = data->velocity.y * factor_y + data->target_velocity.y * (1 - factor_y);
}


static void setPosition(Camera *camera, Vector3 *center, Fixed dt)
{
	CameraSpringArmData *data = &camera->spring_arm.data;
	const CameraSpringArmSettings *settings = &camera->spring_arm.settings;

	/* The velocities are angles per second, so the step is an angle once
	   the seconds are folded in. */
	data->pitch += Angle(data->velocity.y * dt);
	data->yaw   += Angle(data->velocity.x * dt);

	data->yaw = Trig::wrap(data->yaw);

	if (data->pitch > settings->max_pitch) data->pitch = settings->max_pitch;
	if (data->pitch < settings->min_pitch) data->pitch = settings->min_pitch;

	Fixed sin_yaw, cos_yaw, sin_pitch, cos_pitch;
	Trig::sincos(data->yaw,   &sin_yaw,   &cos_yaw);
	Trig::sincos(data->pitch, &sin_pitch, &cos_pitch);

	/* forward points from the camera toward the pivot; right is its horizontal perpendicular */
	Vector3 forward = { cos_pitch * sin_yaw, cos_pitch * cos_yaw, -sin_pitch };
	Vector3 right   = { cos_yaw, -sin_yaw, 0.0_fp };

	Vector3 pivot = { center->x, center->y, center->z + data->pivot_height };

	camera->position.x = pivot.x - forward.x * data->arm_length + right.x * data->side_offset;
	camera->position.y = pivot.y - forward.y * data->arm_length + right.y * data->side_offset;
	camera->position.z = pivot.z - forward.z * data->arm_length;

	camera->target.x = pivot.x + right.x * data->side_offset;
	camera->target.y = pivot.y + right.y * data->side_offset;
	camera->target.z = pivot.z;
}


void CameraSpringArm::init(Camera *camera, const CameraSpringArmDef *def)
{
	camera->type = CAMERA_TYPE_SPRING_ARM;
	camera->spring_arm.settings = def->settings;
	camera->spring_arm.data     = {
		.target_arm_length  = def->arm_length,
		.arm_length         = def->arm_length,
		.target_side_offset = def->side_offset,
		.side_offset        = def->side_offset,
		.yaw                = def->yaw,
		.pitch              = def->pitch,
		.pivot_height       = def->pivot_height,
	};
}


void CameraSpringArm::update(Camera *camera, Vector3 *center, Fixed dt)
{
	setVelocity(camera, dt);
	setPosition(camera, center, dt);
}
