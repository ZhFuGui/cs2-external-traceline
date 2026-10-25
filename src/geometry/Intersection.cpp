#include "Intersection.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include <array>
#include <limits>

namespace cs2::traceline {
namespace {
const auto containment_directions = [] {
    std::array directions{Vec3{1, 0.3713906763541037, 0.6947465906068658},
                          Vec3{0.238617320472852, 1, 0.5176380902050415},
                          Vec3{0.6180339887498949, 0.4142135623730951, 1}};
    for (auto &direction : directions) {
        direction = direction / direction.length();
    }
    return directions;
}();
} // namespace
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
    const double tolerance = 32 * std::numeric_limits<double>::epsilon() * e1.length() * e2.length();
    return intersects_triangle(TriangleQuery{tri.a, e1, e2, tolerance}, ray, distance, barycentric_margin);
}
bool intersects_triangle(const TriangleQuery &tri, const Ray &ray, double &distance,
                         double *barycentric_margin) {
    const auto &e1 = tri.edge1, &e2 = tri.edge2;
    const Vec3 p = ray.direction.cross(e2);
    const double det = e1.dot(p);
    if (std::abs(det) <= tri.tolerance) {
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

bool contains_triangles(const Scene &scene, const Solid &solid, Vec3 point) {
    if (!solid.closed || !solid.bounds.contains(point)) {
        return false;
    }
    const double scale = std::max({solid.diagonal, std::abs(point.x), std::abs(point.y), std::abs(point.z)});
    const double boundary_epsilon = 64 * std::numeric_limits<double>::epsilon() * scale;
    {
        const double distance_limit = 2 * solid.diagonal;
        int previous_winding = -1;
        for (const auto direction : containment_directions) {
            const Ray test{point, direction, distance_limit, 0, distance_limit, 0};
            double high = distance_limit;
            int winding = 0;
            bool ambiguous = false, boundary = false;
            auto inspect = [&](uint32_t index) {
                const auto &tri = scene.triangles[index];
                if (tri.bounds().contains(point, boundary_epsilon) &&
                    point_on_triangle(point, tri, boundary_epsilon)) {
                    boundary = true;
                    return false;
                }
                double distance{}, margin{};
                if (intersects_triangle(scene.query_triangles[index], test, distance, &margin) &&
                    distance > boundary_epsilon && distance < distance_limit) {
                    const double facing = scene.triangle_normals[index].dot(direction);
                    if (margin <= 1e-10 || std::abs(facing) <= 1e-12) {
                        ambiguous = true;
                        return false;
                    }
                    winding += facing > 0 ? 1 : -1;
                }
                return true;
            };
            if (solid.point_root != CS2TL_INVALID_ID) {
                traverse_nodes<IndexNode>(scene.point_nodes, scene.point_order, solid.point_root, test,
                                          -boundary_epsilon, high, inspect);
            } else if (solid.triangles.size() == scene.triangles.size() && !scene.nodes.empty()) {
                traverse(scene, test, -boundary_epsilon, high, inspect);
            } else {
                for (const auto index : solid.triangles) {
                    if (!inspect(index)) {
                        break;
                    }
                }
            }
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
bool contains(const Scene &scene, const Solid &solid, Vec3 point) {
    if (solid.triangles.empty()) {
        return solid.closed && solid.bounds.contains(point) &&
               contains(scene.primitives[solid.point_root], point);
    }
    return contains_triangles(scene, solid, point);
}
namespace {
Vec3 apply_affine(const std::array<double, 12> &m, Vec3 p, double w = 1) {
    return {m[0] * p.x + m[1] * p.y + m[2] * p.z + w * m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + w * m[7],
            m[8] * p.x + m[9] * p.y + m[10] * p.z + w * m[11]};
}
Vec3 nearest_axis_point(const Primitive &p, Vec3 local) {
    const auto axis = p.b - p.a;
    const double squared = axis.dot(axis);
    return squared == 0 ? p.a : p.a + axis * std::clamp((local - p.a).dot(axis) / squared, 0.0, 1.0);
}
} // namespace
void prepare_primitive(Primitive &p) {
    require(p.a.finite() && p.b.finite() && std::isfinite(p.radius) && p.radius > 0 && p.surface >= -1 &&
                p.object != CS2TL_INVALID_ID,
            CS2TL_INVALID_GEOMETRY, "Invalid capsule/sphere data.");
    p.a = Vec3(p.a.external());
    p.b = Vec3(p.b.external());
    p.radius = static_cast<float>(p.radius);
    require(p.a.finite() && p.b.finite() && std::isfinite(p.radius) && p.radius > 0, CS2TL_INVALID_GEOMETRY,
            "Primitive coordinates/radius exceed float32 representation.");
    auto &m = p.pose;
    for (auto &x : m) {
        x = static_cast<float>(x);
        require(std::isfinite(x), CS2TL_INVALID_GEOMETRY, "Nonfinite primitive transform.");
    }
    const double determinant = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8]) +
                               m[2] * (m[4] * m[9] - m[5] * m[8]);
    require(std::isfinite(determinant) && determinant != 0, CS2TL_INVALID_GEOMETRY,
            "Singular primitive transform.");
    auto &q = p.inverse;
    q = {(m[5] * m[10] - m[6] * m[9]) / determinant, (m[2] * m[9] - m[1] * m[10]) / determinant,
         (m[1] * m[6] - m[2] * m[5]) / determinant,  0,
         (m[6] * m[8] - m[4] * m[10]) / determinant, (m[0] * m[10] - m[2] * m[8]) / determinant,
         (m[2] * m[4] - m[0] * m[6]) / determinant,  0,
         (m[4] * m[9] - m[5] * m[8]) / determinant,  (m[1] * m[8] - m[0] * m[9]) / determinant,
         (m[0] * m[5] - m[1] * m[4]) / determinant,  0};
    const auto shift = apply_affine(q, Vec3{m[3], m[7], m[11]}, 0);
    q[3] = -shift.x;
    q[7] = -shift.y;
    q[11] = -shift.z;
    const auto a = apply_affine(m, p.a), b = apply_affine(m, p.b);
    for (unsigned axis = 0; axis < 3; ++axis) {
        const size_t row = axis * 4;
        const double extent =
            p.radius * std::sqrt(m[row] * m[row] + m[row + 1] * m[row + 1] + m[row + 2] * m[row + 2]);
        const double low = std::min(a[axis], b[axis]) - extent, high = std::max(a[axis], b[axis]) + extent;
        float lo = static_cast<float>(low), hi = static_cast<float>(high);
        if (lo > low) {
            lo = std::nextafter(lo, -INFINITY);
        }
        if (hi < high) {
            hi = std::nextafter(hi, INFINITY);
        }
        p.bounds.lo[axis] = lo;
        p.bounds.hi[axis] = hi;
    }
    require(p.bounds.lo.finite() && p.bounds.hi.finite(), CS2TL_INVALID_GEOMETRY,
            "Primitive bounds exceed float32 representation.");
}
bool contains(const Primitive &p, Vec3 point) {
    const auto local = apply_affine(p.inverse, point);
    const auto delta = local - nearest_axis_point(p, local);
    return delta.dot(delta) < p.radius * p.radius;
}
Vec3 primitive_normal(const Primitive &p, Vec3 point) {
    const auto local = apply_affine(p.inverse, point);
    const auto n = local - nearest_axis_point(p, local);
    const auto &q = p.inverse;
    const Vec3 normal{q[0] * n.x + q[4] * n.y + q[8] * n.z, q[1] * n.x + q[5] * n.y + q[9] * n.z,
                      q[2] * n.x + q[6] * n.y + q[10] * n.z};
    return normal / normal.length();
}
size_t primitive_intersections(const Primitive &p, const Ray &ray, std::array<double, 2> &distances) {
    const auto origin = apply_affine(p.inverse, ray.origin),
               direction = apply_affine(p.inverse, ray.direction, 0);
    const auto axis = p.b - p.a;
    const double length = axis.length();
    std::array<double, 6> roots{};
    size_t count = 0;
    auto solve = [&](double a, double b, double c, auto accept) {
        if (a == 0) {
            return;
        }
        const double disc = std::fma(-a, c, b * b);
        if (disc < 0) {
            return;
        }
        const double q = -b - std::copysign(std::sqrt(disc), b);
        const double first = q == 0 ? -b / a : q / a, second = q == 0 ? first : c / q;
        for (double t : {first, second}) {
            if (std::isfinite(t) && accept(t)) {
                roots[count++] = t;
            }
        }
    };
    const double dd = direction.dot(direction);
    if (length == 0) {
        const auto w = origin - p.a;
        solve(dd, w.dot(direction), w.dot(w) - p.radius * p.radius, [](double) { return true; });
    } else {
        const auto n = axis / length, w = origin - p.a;
        const double dn = direction.dot(n), wn = w.dot(n);
        const auto d = direction - n * dn, v = w - n * wn;
        solve(d.dot(d), v.dot(d), v.dot(v) - p.radius * p.radius, [&](double t) {
            const double y = wn + t * dn;
            return y >= 0 && y <= length;
        });
        for (size_t end = 0; end < 2; ++end) {
            const auto delta = origin - (end ? p.b : p.a);
            solve(dd, delta.dot(direction), delta.dot(delta) - p.radius * p.radius, [&](double t) {
                const double y = wn + t * dn;
                return end ? y >= length : y <= 0;
            });
        }
    }
    if (!count) {
        return 0;
    }
    const auto range = std::minmax_element(roots.begin(), roots.begin() + count);
    distances = {*range.first, *range.second};
    return distances[0] == distances[1] ? 1 : 2;
}
} // namespace cs2::traceline
