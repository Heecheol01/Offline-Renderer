// inclue/film/film.h

#pragma once

#include "core/vector.h"
#include <vector>

namespace COR {
	struct Film {
		int w, h;
		std::vector<Vec3> accum;
		std::vector<uint32_t> spp;

		Film(int width, int height);

		void addSample(int x, int y, const Vec3& rgb);
		Vec3 getPixelAverage(int x, int y) const;
	};
}