#include "Intersection.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include <array>
#include <limits>

namespace cs2::traceline {
Ray make_ray(const cs2tl_segment &segment) {
    require(segment.flags == 0, CS2TL_INVALID_ARGUMENT, "Unknown segment flags.");
    Vec3 a(segment.start), b(segment.end);
    require(a.finite() && b.finite(), CS2TL_INVALID_ARGUMENT, "Segment coordinates must be finite.");
    const double length = (b - a).length();
    require(length > 0 && std::isfinite(length) && length <= std::numeric_limits<float>::max(),
            CS2TL_INVALID_ARGUMENT, "Segment length must be nonzero and fit a finite float32.");
    require(std::isfinite(segment.endpoint_epsilon) && segment.endpoint_epsilon >= 0 &&
                std::isfinite(segment.merge_epsilon) && segment.merge_epsilon >= 0,
            CS2TL_INVALID_ARGUMENT, "Segment tolerances must be finite and nonnegative.");
    require(2.0 * segment.endpoint_epsilon < length, CS2TL_INVALID_ARGUMENT,
            "Endpoint tolerance excludes the entire segment.");
    return {a,
            (b - a) / length,
            length,
            segment.endpoint_epsilon,
            length - segment.endpoint_epsilon,
            segment.merge_epsilon};
}

bool intersects_triangle(const Triangle &tri, const Ray &ray, double &distance, double *barycentric_margin) {
    // Double intermediates avoid the old absolute determinant threshold, whose
    // result depended on triangle size. Boundary duplicates are handled later.
    const Vec3 e1 = tri.b - tri.a, e2 = tri.c - tri.a;
    const Vec3 p = ray.direction.cross(e2);
    const double det = e1.dot(p);
    const double tolerance = 32 * std::numeric_limits<double>::epsilon() * e1.length() * e2.length();
    if (std::abs(det) <= tolerance) {
        return false;
    }
    const double inverse = 1 / det;
    const Vec3 s = ray.origin - tri.a;
    const double u = s.dot(p) * inverse;
    constexpr double edge_tolerance = 64 * std::numeric_limits<double>::epsilon();
    if (u < -edge_tolerance || u > 1 + edge_tolerance) {
        return false;
    }
    const Vec3 q = s.cross(e1);
    const double v = ray.direction.dot(q) * inverse;
    if (v < -edge_tolerance || u + v > 1 + edge_tolerance) {
        return false;
    }
    distance = e2.dot(q) * inverse;
    if (barycentric_margin) {
        *barycentric_margin = std::min({u, v, 1 - u - v});
    }
    return std::isfinite(distance);
}

bool point_on_triangle(Vec3 p, const Triangle &tri, double epsilon) {
    const auto ab = tri.b - tri.a, ac = tri.c - tri.a, ap = p - tri.a;
    const auto normal = ab.cross(ac);
    const double normal_length = normal.length();
    if (std::abs(ap.dot(normal)) > epsilon * normal_length) {
        return false;
    }
    const double aa = ab.dot(ab), bb = ac.dot(ac), cc = ab.dot(ac);
    const double determinant = normal.dot(normal);
    if (determinant == 0) {
        return false;
    }
    const double u = (ap.dot(ab) * bb - ap.dot(ac) * cc) / determinant;
    const double v = (ap.dot(ac) * aa - ap.dot(ab) * cc) / determinant;
    const double bary_epsilon = epsilon / std::max(ab.length(), ac.length());
    return u >= -bary_epsilon && v >= -bary_epsilon && u + v <= 1 + bary_epsilon;
}

bool contains(const Scene &scene, const Solid &solid, Vec3 point) {
    if (!solid.closed || !solid.bounds.contains(point)) {
        return false;
    }
    const double scale = std::max({(solid.bounds.hi - solid.bounds.lo).length(), std::abs(point.x),
                                   std::abs(point.y), std::abs(point.z)});
    const double boundary_epsilon = 64 * std::numeric_limits<double>::epsilon() * scale;
    if (!scene.nodes.empty()) {
        const auto solid_id = static_cast<uint32_t>(&solid - scene.solids.data());
        const double distance_limit = 2 * (solid.bounds.hi - solid.bounds.lo).length();
        const std::array directions{Vec3{1, 0.3713906763541037, 0.6947465906068658},
                                    Vec3{0.238617320472852, 1, 0.5176380902050415},
                                    Vec3{0.6180339887498949, 0.4142135623730951, 1}};
        int previous_winding = -1;
        for (auto direction : directions) {
            direction = direction / direction.length();
            const Ray test{point, direction, distance_limit, 0, distance_limit, 0};
            double high = distance_limit;
            int winding = 0;
            bool ambiguous = false, boundary = false;
            traverse(scene, test, -boundary_epsilon, high, [&](uint32_t index) {
                const auto &tri = scene.triangles[index];
                if (tri.solid != solid_id) {
                    return true;
                }
                if (tri.bounds().contains(point, boundary_epsilon) &&
                    point_on_triangle(point, tri, boundary_epsilon)) {
                    boundary = true;
                    return false;
                }
                double distance{}, margin{};
                if (intersects_triangle(tri, test, distance, &margin) && distance > boundary_epsilon &&
                    distance < distance_limit) {
                    const double facing = tri.normal().dot(direction);
                    if (margin <= 1e-10 || std::abs(facing) <= 1e-12) {
                        ambiguous = true;
                        return false;
                    }
                    winding += facing > 0 ? 1 : -1;
                }
                return true;
            });
            if (boundary) {
                return false;
            }
            if (!ambiguous && (winding == 0 || winding == 1)) {
                if (previous_winding == winding) {
                    return winding == 1;
                }
                if (previous_winding != -1) {
                    break; // Disagreement is resolved by the direction-independent fallback.
                }
                previous_winding = winding;
            }
        }
    }
    // Shared-edge/near-tangent ambiguity falls back to signed solid angle.
    // It is slower, but independent of the test directions and hit deduplication.
    // Closed components have an outward orientation after topology repair.
    double angle = 0;
    for (uint32_t index : solid.triangles) {
        const auto &tri = scene.triangles[index];
        const Vec3 a = tri.a - point, b = tri.b - point, c = tri.c - point;
        const double la = a.length(), lb = b.length(), lc = c.length();
        if (la == 0 || lb == 0 || lc == 0 || point_on_triangle(point, tri, boundary_epsilon)) {
            return false;
        }
        angle +=
            2 * std::atan2(a.dot(b.cross(c)), la * lb * lc + a.dot(b) * lc + b.dot(c) * la + c.dot(a) * lb);
    }
    return std::abs(angle) > 6.283185307179586;
}
} // namespace cs2::traceline
