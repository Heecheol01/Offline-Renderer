#pragma once

#include <string>

#include "film/film.h"
#include "core/vector.h"

namespace COR {
	bool writePPM(const std::string& path, const Film& film, bool flipY = true, float gamma = 2.2f);
}