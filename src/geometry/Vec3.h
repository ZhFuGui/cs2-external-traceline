#pragma once
#include "cs2-external-traceline/types.h"
#include <algorithm>
#include <cmath>

namespace cs2::traceline {
struct Vec3 {
    double x{}, y{}, z{};
    Vec3() = default;
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    explicit Vec3(cs2tl_vec3 value) : x(value.x), y(value.y), z(value.z) {}
    double operator[](unsigned i) const { return i == 0 ? x : (i == 1 ? y : z); }
    double &operator[](unsigned i) { return i == 0 ? x : (i == 1 ? y : z); }
    Vec3 operator+(Vec3 b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator-(Vec3 b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return *this * (1.0 / s); }
    double dot(Vec3 b) const { return x * b.x + y * b.y + z * b.z; }
    Vec3 cross(Vec3 b) const { return {y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x}; }
    double length() const { return std::hypot(x, y, z); }
    bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
    cs2tl_vec3 external() const {
        return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }
};
inline Vec3 min(Vec3 a, Vec3 b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
inline Vec3 max(Vec3 a, Vec3 b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}
} // namespace cs2::traceline
