// include/scene/geometry.h

#pragma once

#include "core/vector.h"
#include "core/ray.h"
#include "scene/hit.h"
#include "core/rng.h"
#include "core/random.h"
#include "scene/aabb.h"
#include <cmath>

namespace COR {
    static constexpr float PI = 3.1415926535f;

    struct Shape {
        virtual ~Shape() = default;

        // ray-geometry intersect
        virtual bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const = 0;

        virtual bool bounds(AABB& out) const = 0;
        // surface area
        virtual float area() const { return 0.0f; }

        // sample a point uniformly on the surface
        virtual bool sampleSurface(RNG& rng, Vec3& pos, Vec3& normal, float& pdf_area) const {
            (void)rng; (void)pos; (void)normal; (void)pdf_area;
            return false;
        }
    };

    struct Sphere final : public Shape {
        Vec3 center;
        float radius = 1.0f;

        Sphere(const Vec3& c, float r) : center(c), radius(r) {}

        bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const override {
            // Quadratic: |o + t * d - c|^2 = R^2
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
            return true;
        }

        bool bounds(AABB& out) const override {
            out.mn = center - Vec3(radius);
            out.mx = center + Vec3(radius);
            return true;
        }

        float area() const override {
            return 4.0f * PI * radius * radius;
        }

        bool sampleSurface(RNG& rng, Vec3& pos, Vec3& normal, float& pdf_area) const override {
            // uniform sampling on sphere surface
            Vec3 n = randomUnitVector(rng);
            pos = center + n * radius;
            normal = n;
            float A = area();
            pdf_area = (A > 0.0f) ? (1.0f / A) : 0.0f;
            return (pdf_area > 0.0f);
        }
    };

    struct Quad final : Shape {
        Vec3 q;
        Vec3 u, v;
        Vec3 n;

        float uu = 0.0f, uv = 0.0f, vv = 0.0f;
        float det = 0.0f;

        Quad(const Vec3& corner, const Vec3& edgeU, const Vec3& edgeV) : q(corner), u(edgeU), v(edgeV) 
        {
            n = normalize(cross(u, v));

            uu = dot(u, u);
            uv = dot(u, v);
            vv = dot(v, v);
            det = uu * vv - uv * uv;
        }

        bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const override {
            // intersect ray with supporting plane
            float denom = dot(r.d, n);
            if (std::fabs(denom) < 1e-6f) return false;

            float t = dot(q - r.o, n) / denom;
            if (t < tMin || t > tMax) return false;

            Vec3 p = r.at(t);

            // p = q + a*u + b*v for (a,b)
            Vec3 w = p - q;

            float wu = dot(w, u);
            float wv = dot(w, v);

            if (std::fabs(det) < 1e-12f) return false; // degenerate quad

            float a = (wu * vv - wv * uv) / det;
            float b = (wv * uu - wu * uv) / det;

            // check if inside [0,1]x[0,1]
            if (a < 0.0f || a > 1.0f || b < 0.0f || b > 1.0f) return false;

            rec.t = t;
            rec.p = p;
            rec.setFaceNormal(r, n);
            return true;
        }

        bool bounds(AABB& out) const override {
            out = AABB{};
            Vec3 p0 = q;
            Vec3 p1 = q + u;
            Vec3 p2 = q + v;
            Vec3 p3 = q + u + v;
            out.expand(p0); out.expand(p1); out.expand(p2); out.expand(p3);

            const float eps = 1e-6f;
            out.mn = out.mn - Vec3(eps);
            out.mx = out.mx + Vec3(eps);
            return true;
        }

        float area() const override {
            return length(cross(u, v));
        }

        bool sampleSurface(RNG& rng, Vec3& pos, Vec3& normal, float& pdf_area) const override {
            float a = rng.nextFloat01();
            float b = rng.nextFloat01();
            pos = q + u * a + v * b;
            normal = n;
            float A = area();
            pdf_area = (A > 0.0f) ? (1.0f / A) : 0.0f;
            return (pdf_area > 0.0f);
        }
    };
}