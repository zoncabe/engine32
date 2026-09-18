#include "physics/math/e32_math_common.h"
#include "camera/e32_camera.h"
#include "camera/e32_spring_arm.h"
#include "control/e32_camera_control.h"
#ifdef ENGINE_32_PLAYER
#include "player/e32_player.h"
#endif


/* x and y arrive normalized: how hard the camera is being pushed, whatever
   the game read to get there. */
static void cameraControl_setSpringArmInput(Camera *camera, Fixed x, Fixed y)
{
	CameraSpringArmData *data = &camera->spring_arm.data;
	const CameraSpringArmSettings *settings = &camera->spring_arm.settings;

	data->target_velocity.x = x * settings->max_velocity.x * settings->direction.x;
	data->target_velocity.y = y * settings->max_velocity.y * settings->direction.y;
}


static void (*cameraControl_handler[CAMERA_TYPE_COUNT])(Camera *, Fixed, Fixed) = {
	cameraControl_setSpringArmInput,   /* CAMERA_TYPE_SPRING_ARM */
};


static void cameraControl_setInput(Camera *camera, Fixed x, Fixed y)
{
	if (camera->type == CAMERA_TYPE_NONE) return;
	cameraControl_handler[camera->type](camera, x, y);
}

/* The aiming pose: in over the shoulder, narrower view, slower swing. It only
   ever adds its offsets to the targets, so what the player set is still there
   when the aim lets go. Runs at the tail of the update, after the control
   wrote the velocity this scales. */
static void cameraControl_setAiming(Camera *camera, bool aiming, Fixed dt)
{
	if (camera->type != CAMERA_TYPE_SPRING_ARM) return;

	const CameraSpringArmSettings *settings = &camera->spring_arm.settings;
	CameraSpringArmData *data = &camera->spring_arm.data;

	Fixed arm  = data->target_arm_length;
	Angle fov  = camera->target_field_of_view;
	Fixed side = data->target_side_offset;

	if (aiming) {
		arm  += settings->aim_arm_length;
		fov  += settings->aim_field_of_view;
		side += settings->aim_side_offset;
	}

	CameraControl::setDistance(camera, arm, dt);
	CameraControl::setFieldOfView(camera, fov, dt);
	CameraControl::setSideOffset(camera, side, dt);

	/* The swing was already written this frame: aiming asks for a share of
	   what the stick pushed, not for a different push. */
	if (aiming) {
		data->velocity.x *= settings->aim_velocity_scale;
		data->velocity.y *= settings->aim_velocity_scale;
	}
}


void CameraControl::update(Camera *camera, const CameraControlBinding *binding,
                           const Scene3D *scene, Fixed dt)
{
	/* No binding is the normal state of a camera no controls name: it holds
	   whatever the scene placed it with and the sticks leave it alone. */
	if (binding == NULL) return;

	const Controller *controller = &controller_get()[binding->player];

	Fixed x =  axis_get(controller, binding->pan);
	Fixed y = -axis_get(controller, binding->tilt);

	cameraControl_setInput(camera, x, y);

	if (camera->type == CAMERA_TYPE_SPRING_ARM) {
		const CameraSpringArmSettings *settings = &camera->spring_arm.settings;

		Fixed distance = button_getPressed(controller, controller->held, binding->distance_out)
		               - button_getPressed(controller, controller->held, binding->distance_in);

		Fixed fov = button_getPressed(controller, controller->held, binding->fov_out)
		          - button_getPressed(controller, controller->held, binding->fov_in);

		/* The stick moves what the arm is asked for, never where it is: the
		   aim rides on top of this and the two never fight over one field. */
		camera->spring_arm.data.target_arm_length += distance * settings->distance_speed * dt;
		camera->target_field_of_view              += Angle(fov * settings->fov_speed * dt);

		if (camera->target_field_of_view < CAMERA_FOV_MIN) camera->target_field_of_view = CAMERA_FOV_MIN;
		if (camera->target_field_of_view > CAMERA_FOV_MAX) camera->target_field_of_view = CAMERA_FOV_MAX;

		camera->spring_arm.data.target_arm_length = clamp(
			camera->spring_arm.data.target_arm_length, Fixed(), SPRING_ARM_MAX_LENGTH);
	}

#ifdef ENGINE_32_PLAYER
	/* The binding names the player, so the camera knows what to follow on its
	   own. A seat with nobody in it leaves the camera where it was: a game
	   that frames something else calls Viewport::updateCamera instead. */
	const Player *player = &player_get()[binding->player];

	/* The body it follows is also the body that aims, so the pose needs no
	   call of its own. */
	cameraControl_setAiming(camera,
		player->character && player->character->movement.data.aiming, dt);

	if (player->entity)
		camera->update(&player->entity->transform.position, scene, dt);
#else
	(void)scene;
	cameraControl_setAiming(camera, false, dt);
#endif
}


void CameraControl::setDistance(Camera *camera, Fixed distance, Fixed dt)
{
	if (camera->type != CAMERA_TYPE_SPRING_ARM) return;

	CameraSpringArmData *data = &camera->spring_arm.data;
	Fixed rate = camera->spring_arm.settings.zoom_response_rate;

	data->arm_length = lerp(data->arm_length, distance, 1 - expNeg(rate * dt));
}


void CameraControl::setFieldOfView(Camera *camera, Angle field_of_view, Fixed dt)
{
	if (camera->type != CAMERA_TYPE_SPRING_ARM) return;

	/* The aim's offset rides on top of the target, so the bound goes here,
	   on the final ask, not only on the stick's side. */
	if (field_of_view < CAMERA_FOV_MIN) field_of_view = CAMERA_FOV_MIN;
	if (field_of_view > CAMERA_FOV_MAX) field_of_view = CAMERA_FOV_MAX;

	Fixed rate = camera->spring_arm.settings.zoom_response_rate;

	camera->field_of_view = Angle(lerp(Fixed(camera->field_of_view), Fixed(field_of_view),
	                                   1 - expNeg(rate * dt)));
}


void CameraControl::setSideOffset(Camera *camera, Fixed side_offset, Fixed dt)
{
	if (camera->type != CAMERA_TYPE_SPRING_ARM) return;

	CameraSpringArmData *data = &camera->spring_arm.data;
	Fixed rate = camera->spring_arm.settings.zoom_response_rate;

	data->side_offset = lerp(data->side_offset, side_offset, 1 - expNeg(rate * dt));
}
