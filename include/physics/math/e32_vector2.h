#ifndef ENGINE_32_VECTOR2_H
#define ENGINE_32_VECTOR2_H

#include "physics/math/e32_fixed.h"


struct Vector2
{
	Fixed x;
	Fixed y;

	Fixed magnitude() const;
};


#endif
