/*
	The engine's number: 20.12 fixed point in an int32, PSYQo's FixedPoint
	with its defaults. One meter is 4096 raw. The literal 2.5_fp converts
	at compile time; there is no runtime conversion from float anywhere.
*/
#ifndef ENGINE_32_FIXED_H
#define ENGINE_32_FIXED_H

#include "psyqo/fixed-point.hh"


using Fixed = psyqo::FixedPoint<>;

using namespace psyqo::fixed_point_literals;


#endif
