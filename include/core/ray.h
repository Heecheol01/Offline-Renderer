// include/core/ray.h

#pragma once

#include "vector.h"

namespace COR {
	struct Ray {
		Vec3 o, d;
		Vec3 at(float t) const { return o + d * t; }
	};
}
