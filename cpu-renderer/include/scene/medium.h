// cpu-renderer/include/scene/medium.h

#pragma once

#include "core/vector.h"
#include "core/rng.h"
#include "core/random.h"
#include "core/constants.h"

#include <cmath>

namespace COR {
	struct PhaseSample {
		Vec3 wi;
		float pdf = 0.0f;
		float p = 0.0f;
	};

	struct PhaseFunction {
		virtual ~PhaseFunction() = default;
		virtual float eval(const Vec3& wo, const Vec3& wi) const = 0;
		virtual float pdf(const Vec3& wo, const Vec3& wi) const = 0;
		virtual bool sample(const Vec3& wo, RNG& rng, PhaseSample& ps) const = 0;
	};

	// isotropic phase
	struct IsotropicPhase final : PhaseFunction {
		float eval(const Vec3& wo, const Vec3& wi) const override {
			(void)wo; (void)wi;
			return 1.0f / (4.0f * PI);
		}
		float pdf(const Vec3& wo, const Vec3& wi) const override {
			(void)wo; (void)wi;
			return 1.0f / (4.0f * PI);
		}
		bool sample(const Vec3& wo, RNG& rng, PhaseSample& ps) const override {
			(void)wo;
			ps.wi = randomUnitVector(rng);
			ps.pdf = 1.0f / (4.0f * PI);
			ps.p = ps.pdf;
			return true;
		}
	};

	struct HomogeneousMedium {
		Vec3 sigma_a{ 0.0f };
		Vec3 sigma_s{ 0.0f };
		Vec3 sigma_t{ 0.0f };

		IsotropicPhase phase;

		HomogeneousMedium() = default;
		HomogeneousMedium(const Vec3& a, const Vec3& s) : sigma_a(a), sigma_s(s), sigma_t(a + s) {};

		// Beer-Lambert
		Vec3 Tr(float dist) const {
			return Vec3{
				std::exp(-sigma_t.x * dist),
				std::exp(-sigma_t.y * dist),
				std::exp(-sigma_t.z * dist)
			};
		}
	};
}