// include/core/rng.h

#pragma once

#include <cstdint>

namespace COR {
	// 64-bit Mixer(Mixing Seed)
	static inline uint64_t splitmix64(uint64_t x) {
		x += 0x9E3779B97F4A7C15ull;
		x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
		x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
		return x ^ (x >> 31);
	}

	// PCG32 (O'Neil)
	// - nextU32() : 32-bit Random Number
	// - nextFloat01() : [0, 1) float Random Number
	class RNG {
	public:
		RNG(uint64_t seed = 1, uint64_t stream = 1) { seedRNG(seed, stream); }

		void seedRNG(uint64_t seed, uint64_t stream = 1) {
			// stream(inc) must be odd number
			state_ = 0u;
			inc_ = (stream << 1u) | 1u;
			nextU32();                 // progress internal state once
			state_ += seed;
			nextU32();                 // progressing again (PCG's promote seeding way)
		}

		uint32_t nextU32() {
			// LCG step
			uint64_t oldstate = state_;
			state_ = oldstate * 6364136223846793005ull + inc_;

			// XSH RR output transform
			uint32_t xorshifted = static_cast<uint32_t>(((oldstate >> 18u) ^ oldstate) >> 27u);
			uint32_t rot = static_cast<uint32_t>(oldstate >> 59u);
			return (xorshifted >> rot) | (xorshifted << ((-static_cast<int32_t>(rot)) & 31));
		}

		float nextFloat01() {
			// Create [0,1) using only float valid bits (24 bits)
			// Distribution is more stable with the top 24 bits
			uint32_t u = nextU32();
			u >>= 8; // 24bit
			return static_cast<float>(u) * (1.0f / 16777216.0f); // 2^24
		}

		float uniform(float a, float b) { return a + (b - a) * nextFloat01(); }

	private:
		uint64_t state_ = 0;
		uint64_t inc_ = 0;
	};

	static inline uint64_t makeSeed(uint32_t x, uint32_t y, uint32_t sample,
		uint32_t frame = 0, uint64_t userSeed = 0) {
		uint64_t v = (static_cast<uint64_t>(x) << 32) ^ static_cast<uint64_t>(y);
		v = splitmix64(v ^ (static_cast<uint64_t>(sample) * 0x9E3779B97F4A7C15ull));
		v = splitmix64(v ^ (static_cast<uint64_t>(frame) * 0xBF58476D1CE4E5B9ull));
		v = splitmix64(v ^ userSeed);
		return v;
	}

	static inline uint64_t makeSeed2(uint32_t x, uint32_t y, uint32_t s) {
		uint64_t v = (uint64_t)x | ((uint64_t)y << 32);
		v ^= (uint64_t)s * 0x9e3779b97f4a7c15ull;
		return splitmix64(v);
	}
}