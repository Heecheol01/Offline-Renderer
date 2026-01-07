// include/core/ray.h

#pragma once

#include "vector.h"
#include "scene/medium.h"

namespace COR {
	struct Ray {
		Vec3 o, d;
		Vec3 at(float t) const { return o + d * t; }
		const HomogeneousMedium* medium = nullptr;
	};
}
