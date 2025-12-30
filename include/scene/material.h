// include/scene/material.h

#pragma once

#include "core/vector.h"
#include "core/ray.h"
#include "core/rng.h"
#include "core/random.h"
#include "scene/hit.h"
#include "core/sampling.h"
#include <cmath>

namespace COR {
	enum class MaterialType {
		Lambert,
		Metal,
		Dielectric,
		Emissive,
	};

	struct Material {
		MaterialType type = MaterialType::Lambert;

		// Common parameters
		Vec3 albedo{ 0.8f };
		float fuzz = 0.0f;
		float ir = 1.5f;
		Vec3 emission{ 0.0f };

		Vec3 emitted() const {
			return (type == MaterialType::Emissive) ? emission : Vec3{ 0.f };
		}

		bool scatter(const Ray& rIn, const HitRecord& rec, RNG& rng, Vec3& attenuation, Ray& scattered) const {
			switch (type) {
			case MaterialType::Lambert: {
				// Lambertian diffuse scattering
				Vec3 scatterDir = cosineSampleHemisphere(rec.n, rng);

				scattered = Ray{ rec.p, scatterDir };
				attenuation = albedo;
				return true;
			}

			case MaterialType::Metal: {
				// Perfect reflection + fuzz perturbation
				Vec3 unitDir = normalize(rIn.d);
				Vec3 reflected = reflect(unitDir, rec.n);

				float f = clamp(fuzz, 0.0f, 1.0f);
				Vec3 dir = reflected + randomInUnitSphere(rng) * f;

				scattered = Ray{ rec.p, dir };
				attenuation = albedo;
				return dot(scattered.d, rec.n);
			}

			case MaterialType::Dielectric: {
				// Glass : reflect or refract
				attenuation = Vec3{ 1.0f };

				float refractionRatio = rec.frontFace ? (1.0f / ir) : ir;

				Vec3 unitDir = normalize(rIn.d);
				float cosTheta = std::fmin(dot(unitDir * -1.0f, rec.n), 1.0f);
				float sinTheta = std::sqrt(std::fmax(0.0f, 1.0f - cosTheta * cosTheta));

				bool cannotRefract = refractionRatio * sinTheta > 1.0f;

				Vec3 direction;
				if (cannotRefract || reflectance(cosTheta, refractionRatio) > rng.nextFloat01())
					direction = reflect(unitDir, rec.n);
				else
					direction = refract(unitDir, rec.n, refractionRatio);

				scattered = Ray{ rec.p, direction };
				return true;
			}

			case MaterialType::Emissive:
				return false;
			}

			return false;
		}
	};
}