/*
	A spring arm behind the body: the arm trails it, the right stick swings
	it around, and the shoulder buttons pull it in and out. What it follows
	is the player's body, handed to the viewport every frame by the state.
*/
#include "camera/e32_camera.h"


extern const CameraDef camera = {

	.type = CAMERA_TYPE_SPRING_ARM,

	.field_of_view = 60.0_deg,
	.near_clipping = 1.0_fp,
	.far_clipping  = 50.0_fp,
	.auto_clipping = true,

	.spring_arm = {
		.arm_length   = 5.0_fp,
		.side_offset  = 0.0_fp,
		.yaw          = -45.0_deg,
		.pitch        = 12.0_deg,
		.pivot_height = 1.2_fp,

		.settings = {
			.response_rate = { 10.0_fp, 10.0_fp },
			.max_velocity  = { Fixed(100.0_deg), Fixed(90.0_deg) },
			.direction     = {  1.0_fp, 1.0_fp },
			.zoom_response_rate = 6.0_fp,
			.distance_speed = 4.0_fp,
			.fov_speed      = Fixed(30.0_deg),
			.max_pitch     =  80.0_deg,
			.min_pitch     = -50.0_deg,
		},
	},
};
