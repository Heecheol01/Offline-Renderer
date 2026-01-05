// include/scene/material.h

#pragma once

#include "core/vector.h"
#include "core/rng.h"
#include "core/sampling.h"

#include <cmath>

namespace COR {
	enum class MaterialType {
		Lambert,
		Metal,
		Dielectric,
		Emissive,
	};

	struct BSDFSample {
		Vec3 wi;		// sampled incoming direction
		Vec3 f;			// BSDF value f(wo, wi)
		float pdf = 0;	// pdf for sampling wi
		bool isDelta = false;
	};

	struct Material {
		MaterialType type = MaterialType::Lambert;

		// Common parameters
		Vec3 albedo{ 1.0f };
		Vec3 emission{ 0.0f };

		float fuzz = 0.0f;
		float ir = 1.5f;

		bool isEmissive() const { return type == MaterialType::Emissive; }
		bool isDelta() const { return (type == MaterialType::Metal) || (type == MaterialType::Dielectric); }

		Vec3 emitted() const { return emission; }

		// BSDF evaluation and pdf (only meaningful for non-delta types)
		Vec3 evalBSDF(const Vec3& wo, const Vec3& wi, const Vec3& n) const;
		float pdfBSDF(const Vec3& wo, const Vec3& wi, const Vec3& n) const;

		// sample bsdf: returns false if the material absorbs/terminates
		bool sampleBSDF(const Vec3& wo, const Vec3& n, RNG& rng, BSDFSample& bs) const;

		bool scatter(const Ray& rIn, const HitRecord& rec, RNG& rng, Vec3& attenuation, Ray& scattered) const;
	};
}