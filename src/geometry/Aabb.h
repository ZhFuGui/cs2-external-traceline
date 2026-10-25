#pragma once
#include "Vec3.h"
#include <limits>

namespace cs2::traceline {
struct Aabb {
    Vec3 lo{INFINITY, INFINITY, INFINITY};
    Vec3 hi{-INFINITY, -INFINITY, -INFINITY};
    void extend(Vec3 p) {
        lo = min(lo, p);
        hi = max(hi, p);
    }
    void extend(const Aabb &b) {
        extend(b.lo);
        extend(b.hi);
    }
    Vec3 center() const { return lo * 0.5 + hi * 0.5; }
    bool contains(Vec3 p, double epsilon = 0) const {
        return p.x >= lo.x - epsilon && p.x <= hi.x + epsilon && p.y >= lo.y - epsilon &&
               p.y <= hi.y + epsilon && p.z >= lo.z - epsilon && p.z <= hi.z + epsilon;
    }
    bool contains(const Aabb &b) const { return contains(b.lo) && contains(b.hi); }
    double area() const {
        const auto e = hi - lo;
        return 2.0 * (e.x * e.y + e.y * e.z + e.z * e.x);
    }
};
struct Ray {
    Vec3 origin, direction;
    double length{}, low{}, high{}, merge_epsilon{};
    Vec3 at(double distance) const { return origin + direction * distance; }
};
inline bool intersects(const Aabb &box, const Ray &ray, double low, double high, double &entry) {
    for (unsigned axis = 0; axis < 3; ++axis) {
        const double direction = ray.direction[axis];
        if (direction == 0) {
            if (ray.origin[axis] < box.lo[axis] || ray.origin[axis] > box.hi[axis]) {
                return false;
            }
            continue;
        }
        double a = (box.lo[axis] - ray.origin[axis]) / direction;
        double b = (box.hi[axis] - ray.origin[axis]) / direction;
        if (a > b) {
            std::swap(a, b);
        }
        low = std::max(low, a);
        high = std::min(high, b);
        if (low > high) {
            return false;
        }
    }
    entry = low;
    return true;
}
} // namespace cs2::traceline
