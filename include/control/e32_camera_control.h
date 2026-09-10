#ifndef ENGINE_32_CAMERA_CONTROL_H
#define ENGINE_32_CAMERA_CONTROL_H

#include "e32_controller.h"
#include "camera/e32_camera.h"

struct CameraControlBinding
{
	PlayerID player;

	/* The swing: an axis each, the stick's own magnitude. Up on a stick is
	   negative, so tilt reads the axis inverted. */
	AxisID pan;
	AxisID tilt;

	ButtonID distance_in;
	ButtonID distance_out;
	ButtonID fov_in;
	ButtonID fov_out;
};


class CameraControl
{
public:

	/* Reads the controller of the player the binding names. The scene rides
	   through to the camera update, for the clipping fit. */
	static void update(Camera *camera, const CameraControlBinding *binding,
	                   const Scene3D *scene, Fixed dt);

	static void setDistance(Camera *camera, Fixed distance, Fixed dt);
	static void setFieldOfView(Camera *camera, Angle field_of_view, Fixed dt);
	static void setSideOffset(Camera *camera, Fixed side_offset, Fixed dt);
};


#endif
