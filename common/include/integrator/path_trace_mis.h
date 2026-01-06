// inclue/integrator/path_trace_mis.h

#pragma once

#include "core/ray.h"
#include "core/rng.h"
#include "core/vector.h"
#include "scene/world.h"

namespace COR {
	Vec3 trace_path_mis(const Ray& r0, const World& world, RNG& rng, int maxDepth);
}