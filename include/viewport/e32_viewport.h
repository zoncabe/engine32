/*
	What the frame is drawn into, and from where: the screen the console
	shows, the color it starts as, and the camera whose view the GTE gets.

	The view is a rotation and a position: the GTE takes both, folded with
	each object's own placement, and does the projection itself with the
	distance to the screen this computes from the lens. The frustum is the
	same view as six planes, for the culling.
*/
#ifndef ENGINE_32_VIEWPORT_H
#define ENGINE_32_VIEWPORT_H

#include <stdint.h>

#include "psyqo/primitives/common.hh"

#include "physics/math/e32_vector3.h"
#include "physics/math/e32_matrix3.h"
#include "physics/math/e32_transform.h"
#include "physics/math/e32_frustum.h"
#include "camera/e32_camera.h"

/* Fixed by the video mode the system comes up with (W320, progressive). */
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

/* Two framebuffers: the GPU shows one while the other is drawn. Anything
   the render reads while a frame is in flight is kept once per buffer. */
#define FB_COUNT 2


struct Scene3D;

class Viewport
{
public:

	static Viewport &get();

	void init();

	void setClearColor(uint8_t r, uint8_t g, uint8_t b);

	/* Paints the frame the clear color right away. The render clears
	   through its chain instead; this is for a frame with nothing else. */
	void clear();

	/* Moves the camera for this frame, following the given point. */
	void updateCamera(Vector3 *center, const Scene3D *scene);

	/* Reads the camera into the view the render uses: the rotation that
	   takes world space to the GTE's (x right, y down, z forward), the
	   camera's position, the projection distance for the lens, and the
	   frustum. */
	void setPerspectiveCamera();

	int width;
	int height;

	/* The buffer this frame draws into; the render advances it. */
	uint8_t fb_index;

	Camera camera;

	/* World to camera space: rotation rows right, down, forward, and the
	   eye position. 'view' is both as one transform. */
	Matrix3   view_rotation;
	Vector3   view_position;
	Transform view;

	Frustum frustum;

	/* Distance from the eye to the screen, in pixels: what the GTE
	   divides by depth. */
	int projection;

	psyqo::Color clear_color;
};


#endif
