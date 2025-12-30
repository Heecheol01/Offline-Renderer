// include/core/vector.h

#pragma once

#include <cmath>

namespace COR {
	struct Vec3 {
		float x = 0.0f, y = 0.0f, z = 0.0f;

		Vec3() : x(0), y(0), z(0) {};
		Vec3(float s) : x(s), y(s), z(s) {};
		Vec3(float a, float b, float c) : x(a), y(b), z(c) {};

		// ctor
		Vec3 operator+(const Vec3& r) const { return Vec3{ x + r.x, y + r.y, z + r.z }; }
		Vec3 operator-(const Vec3& r) const { return Vec3{ x - r.x, y - r.y, z - r.z }; }
		Vec3 operator*(float s) const { return Vec3{ x * s, y * s, z * s }; }
		Vec3 operator/(float s) const { return Vec3{ x / s, y / s, z / s }; }

		Vec3& operator+=(const Vec3& r) { x += r.x; y += r.y; z += r.z; return *this; }
		Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

		Vec3& norm() { return *this = *this * (1 / std::sqrt(x * x + y * y + z * z)); }
	};

	inline Vec3 operator*(float s, const Vec3& v) { return v * s; }
	inline Vec3 operator*(const Vec3& a, const Vec3& b) { return Vec3{ a.x * b.x, a.y * b.y, a.z * b.z }; }

	inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	inline Vec3 cross(const Vec3& a, const Vec3& b) { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
	inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
	inline Vec3 normalize(const Vec3& v) { float len = length(v); return len <= 0.0f ? Vec3{ 0.0f, 0.0f, 0.0f } : v / len; }

	inline float clamp01(float x) { return (x < 0.0f) ? 0.0f : (x > 1.0f) ? 1.0f : x; }
	inline float clamp(float x, float a, float b) { return (x < a) ? a : (x > b) ? b : x; }
}