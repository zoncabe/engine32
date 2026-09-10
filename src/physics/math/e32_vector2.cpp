#include "physics/math/e32_vector2.h"

#include "physics/math/e32_math_common.h"


Fixed Vector2::magnitude() const
{
	return sqrt(x * x + y * y);
}
