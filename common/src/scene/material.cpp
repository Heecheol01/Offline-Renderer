// src/scene/material.cpp

#include "scene/material.h"

#include "core/onb.h"
#include "core/sampling.h"
#include "core/random.h"

#include <cmath>

namespace COR {
	Vec3 Material::evalBSDF(const Vec3& wo, const Vec3& wi, const Vec3& n) const {
		(void)wo; (void)wi;
		if (type == MaterialType::Lambert) {
			return albedo * (1.0f / PI);
		}

		// delta materials: treat as 0 for continuous eval in NEE stage
		return Vec3{ 0 };
	}

	float Material::pdfBSDF(const Vec3& wo, const Vec3& wi, const Vec3& n) const {
		(void)wo;
		if (type == MaterialType::Lambert) {
			return COR::lambertPdf(n, wi);
		}

		return 0.0f;
	}

	bool Material::sampleBSDF(const Vec3& wo, const Vec3& n, RNG& rng, BSDFSample& bs) const {
		if (type == MaterialType::Emissive) return false;

		if (type == MaterialType::Lambert) {
			Vec3 wi = COR::cosineSampleHemisphere(n, rng);

			float pdf = COR::lambertPdf(n, wi);
			if (pdf <= 0.0f) return false;

			bs.wi = wi;
			bs.f = albedo * (1.0f / PI);
			bs.pdf = pdf;
			bs.isDelta = false;
			return true;
		}

		if (type == MaterialType::Metal) {
			Vec3 wi = reflect(-wo, n) + randomInUnitSphere(rng) * fuzz;
			wi = normalize(wi);
			if (dot(wi, n) <= 0.0f) return false;

			bs.wi = wi;
			bs.f = albedo;
			bs.pdf = 1.0f;
			bs.isDelta = true;
			return true;
		}

		if (type == MaterialType::Dielectric) {
			bs.isDelta = true;
			bs.pdf = 1.0f;

			float refractionRatio = (dot(wo, n) > 0.0f) ? (1.0f / ir) : ir;

			Vec3 unitDir = normalize(-wo);
			float cosTheta = std::fmin(dot(-unitDir, n), 1.0f);
			float sinTheta = std::sqrt(std::fmax(0.0f, 1.0f - cosTheta * cosTheta));

			bool cannotRefract = refractionRatio * sinTheta > 1.0f;

			Vec3 dir;
			if (cannotRefract || reflectance(cosTheta, ir) > rng.nextFloat01()) {
				dir = reflect(unitDir, n);
			}
			else {
				if (!refract(unitDir, n, refractionRatio, dir)) {
					dir = reflect(unitDir, n);
				}
			}

			bs.wi = normalize(dir);
			bs.f = Vec3{ 1.f };
			return true;
		}

		return false;
	}

	bool Material::scatter(const Ray& rIn, const HitRecord& rec, RNG& rng, Vec3& attenuation, Ray& scattered) const {
		// Wrapper to keep your old integrator code working if needed.
		Vec3 wo = normalize(-rIn.d);

		BSDFSample bs;
		if (!sampleBSDF(wo, rec.n, rng, bs)) return false;

		float cosTheta = std::fabs(dot(rec.n, bs.wi));
		if (bs.pdf <= 0.0f) return false;

		// attenuation = f * cos / pdf
		attenuation = bs.f * (cosTheta / bs.pdf);
		scattered = Ray{ rec.p, bs.wi };
		return true;
	}
}