// include/scene/sphere.h

#pragma once

#include "core/vector.h"
#include "core/ray.h"
#include "scene/hit.h"
#include <cmath>

namespace COR {
    struct Sphere {
        Vec3 center;
        float radius = 1.0f;
        Vec3 albedo{ 0.8f, 0.8f, 0.8f };

        Sphere(const Vec3& c, float r, const Vec3& a) : center(c), radius(r), albedo(a) { }

        bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const {
            Vec3 oc = r.o - center;
            float a = dot(r.d, r.d);
            float b = dot(oc, r.d);              // half_b
            float c = dot(oc, oc) - radius * radius;
            float disc = b * b - a * c;
            if (disc < 0.0f) return false;

            float s = std::sqrt(disc);
            float t = (-b - s) / a;
            if (t < tMin || t > tMax) {
                t = (-b + s) / a;
                if (t < tMin || t > tMax) return false;
            }

            rec.t = t;
            rec.p = r.at(t);
            Vec3 outward = (rec.p - center) / radius;
            rec.setFaceNormal(r, outward);
            rec.albedo = albedo;
            return true;
        }
    };
}