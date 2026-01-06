#include "io/write_image.h"

#include <fstream>
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace COR{
	static inline bool isFinite3(const Vec3& v) {
		return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
	}

	static inline float applyGamma(float x, float gamma) {
		if (gamma <= 0.0f) return x;
		return std::pow(x, 1.0f / gamma);
	}

	static inline uint8_t toByte(float x, float gamma) {
		x = clamp01(x);
		x = applyGamma(x, gamma);

		int v = static_cast<int>(x * 255.0f + 0.5f);
		if (v < 0) v = 0;
		if (v > 255) v = 255;
		return static_cast<uint8_t>(v);
	}

    bool writePPM(const std::string& path, const Film& film, bool flipY, float gamma) {
        std::ofstream out(path, std::ios::binary);
        if (!out) return false;

        const int W = film.w;
        const int H = film.h;

        // PPM P6 header
        out << "P6\n" << W << " " << H << "\n255\n";

        for (int y = 0; y < H; ++y) {
            const int yy = flipY ? (H - 1 - y) : y;
            for (int x = 0; x < W; ++x) {
                Vec3 c = film.getPixelAverage(x, yy);
                if (!isFinite3(c)) c = Vec3{ 0.0f, 0.0f, 0.0f };

                const uint8_t r = toByte(c.x, gamma);
                const uint8_t g = toByte(c.y, gamma);
                const uint8_t b = toByte(c.z, gamma);

                out.write(reinterpret_cast<const char*>(&r), 1);
                out.write(reinterpret_cast<const char*>(&g), 1);
                out.write(reinterpret_cast<const char*>(&b), 1);
            }
        }

        return static_cast<bool>(out);
    }
}