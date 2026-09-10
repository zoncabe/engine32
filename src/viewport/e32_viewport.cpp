#include "viewport/e32_viewport.h"

#include "system/e32_system.h"
#include "time/e32_time.h"
#include "physics/math/e32_math_common.h"


Viewport &Viewport::get()
{
	static Viewport viewport;
	return viewport;
}


void Viewport::init()
{
	width  = SCREEN_WIDTH;
	height = SCREEN_HEIGHT;

	fb_index = 0;

	clear_color = {{0, 0, 0}};

	camera.init();

	view_rotation = Matrix3::identity();
	view_position = Vector3::zero();
	view          = Transform::identity();
	projection    = SCREEN_HEIGHT / 2;

	frustum = Frustum::fromView(view_rotation, view_position, projection,
	                            SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2,
	                            camera.near_clipping, camera.far_clipping);
}


void Viewport::setClearColor(uint8_t r, uint8_t g, uint8_t b)
{
	clear_color = {{r, g, b}};
}


void Viewport::clear()
{
	System::get().getGpu().clear(clear_color);
}


void Viewport::updateCamera(Vector3 *center, const Scene3D *scene)
{
	camera.update(center, scene, Time::get().delta);
}


void Viewport::setPerspectiveCamera()
{
	/* Z is up in the world. */
	const Vector3 up_world = { 0.0_fp, 0.0_fp, 1.0_fp };

	Vector3 forward = camera.target - camera.position;
	if (forward.squaredMagnitude() < TOLERANCE) forward = { 0.0_fp, 1.0_fp, 0.0_fp };
	forward.normalize();

	Vector3 right = forward.cross(up_world);
	if (right.squaredMagnitude() < TOLERANCE) right = { 1.0_fp, 0.0_fp, 0.0_fp };
	right.normalize();

	Vector3 up = right.cross(forward);

	/* Rows right, down, forward: the GTE's y grows toward the bottom of
	   the screen. Columns first, then the transpose makes them rows. */
	view_rotation = Matrix3::fromColumns(right, -up, forward).transposed();
	view_position = camera.position;

	/* v_camera = R (p - eye) = R p + (-R eye) */
	view.rotation = view_rotation;
	view.position = -view_rotation.transform(view_position);

	/* Half the screen height over the tangent of half the vertical field
	   of view: cos over sin, the one divide of the frame. */
	Fixed s, c;
	Trig::sincos(camera.field_of_view / 2, &s, &c);
	if (s <= 0) s = TOLERANCE;

	projection = ((c * (SCREEN_HEIGHT / 2)) / s).integer();
	if (projection < 1)     projection = 1;
	if (projection > 65535) projection = 65535;

	frustum = Frustum::fromView(view_rotation, view_position, projection,
	                            SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2,
	                            camera.near_clipping, camera.far_clipping);
}
