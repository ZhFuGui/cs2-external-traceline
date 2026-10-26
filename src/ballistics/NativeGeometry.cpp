#include "NativeGeometry.h"
#include "NativeMath.h"
#include "NativeSpans.h"
#include "diagnostics/Error.h"
#include <bit>
#include <cmath>
#include <limits>

namespace cs2::traceline {
namespace {
namespace f32 = native_f32;
cs2tl_vec3 wrapper_normal(cs2tl_vec3 normal) noexcept {
    // The original wrapper executes a quaternion rotation even for identity.
    // Its additions canonicalize signed zero without changing nonzero values.
    return {f32::add(normal.x, 0), f32::add(normal.y, 0), f32::add(normal.z, 0)};
}
cs2tl_vec3 mesh_normal(cs2tl_vec3 normal) noexcept {
    const auto xy = f32::add(f32::mul(normal.y, normal.y), f32::mul(normal.x, normal.x));
    const auto norm2 = f32::add(xy, f32::mul(normal.z, normal.z));
    constexpr auto threshold = std::bit_cast<float>(uint32_t{0x057a0000});
    if (!(norm2 > threshold)) {
        return {};
    }
    return f32::mul(normal, f32::div(1, std::sqrt(norm2)));
}
struct PlaneInterval {
    float entry{-std::numeric_limits<float>::max()}, exit{std::numeric_limits<float>::max()};
    ptrdiff_t entry_plane{-1}, exit_plane{-1};
    bool starts_inside{true}, valid{};
};
PlaneInterval hull_planes(std::span<const PhysicsPlane> planes, cs2tl_vec3 origin, cs2tl_vec3 delta,
                          float scale) noexcept {
    PlaneInterval result;
    for (size_t i = 0; i < planes.size(); ++i) {
        const auto &plane = planes[i];
        auto from = f32::sub(f32::dot_zyx(plane.normal, origin), f32::mul(scale, plane.distance));
        const auto to = f32::add(f32::dot_zyx(plane.normal, delta), from);
        if (from > 0 && to > 0) {
            return result;
        }
        if (from <= 0 && to <= 0) {
            continue;
        }
        if (from > 0) {
            result.starts_inside = false;
        }
        const auto divisor = f32::sub(from, to);
        if (from > to) {
            from = f32::div(f32::sub(from, .03125f), divisor);
            if (from > result.entry) {
                result.entry = from;
                result.entry_plane = static_cast<ptrdiff_t>(i);
            }
        } else {
            from = f32::div(f32::add(from, .03125f), divisor);
            if (from < result.exit) {
                result.exit = from;
                result.exit_plane = static_cast<ptrdiff_t>(i);
            }
        }
    }
    result.valid = result.exit > result.entry;
    return result;
}
struct HullSingle {
    NativeGeometryEvent event{};
    bool hit{};
};
HullSingle hull_single(std::span<const PhysicsPlane> planes, cs2tl_vec3 origin, cs2tl_vec3 delta,
                       float scale) {
    HullSingle result;
    const auto interval = hull_planes(planes, origin, delta, scale);
    if (!interval.valid || planes.empty()) {
        return result;
    }
    if (interval.starts_inside) {
        const auto length = native_ray_length(delta);
        auto normal = length == 0 ? cs2tl_vec3{} : f32::mul(delta, f32::div(1, length));
        normal = {-normal.x, -normal.y, -normal.z};
        result.event = {normal, 0, 0};
        result.hit = true;
    } else if (interval.entry_plane >= 0) {
        result.event = {planes[static_cast<size_t>(interval.entry_plane)].normal, interval.entry, 0};
        result.hit = 1 > interval.entry;
    }
    return result;
}
} // namespace
cs2tl_vec3 native_triangle_normal(const PhysicsTriangle &triangle) noexcept {
    const auto normal = f32::cross(f32::sub(triangle.b, triangle.a), f32::sub(triangle.c, triangle.a));
    return wrapper_normal(mesh_normal(normal));
}
NativeGeometryResult trace_native_triangle(const PhysicsTriangle &triangle, cs2tl_vec3 origin,
                                           cs2tl_vec3 delta, float max_fraction, bool compute_normal,
                                           bool plane_first) {
    NativeGeometryResult result;
    const std::array vertices{triangle.a, triangle.b, triangle.c};
    cs2tl_vec3 normal{};
    float denominator{}, fraction{};
    auto plane = [&] {
        normal = f32::cross(f32::sub(vertices[1], vertices[0]), f32::sub(vertices[2], vertices[0]));
        denominator = f32::dot_xyz(normal, delta);
        fraction = f32::div(f32::dot_xyz(f32::sub(vertices[0], origin), normal), denominator);
        if (fraction <= 0) {
            return false;
        }
        fraction = f32::cap_one(fraction);
        return fraction < 1 && !(max_fraction < fraction);
    };
    if (plane_first && !plane()) {
        return result;
    }
    bool negative = false, positive = false;
    for (size_t i = 0; i < 3; ++i) {
        const auto a = vertices[(i + 1) % 3], b = vertices[(i + 2) % 3];
        const auto edge = f32::sub(b, a);
        const auto midpoint = f32::sub(f32::mul(f32::add(a, b), .5f), origin);
        const auto sign = f32::dot_xyz(f32::cross(edge, midpoint), delta);
        negative |= sign < 0;
        positive |= sign > 0;
    }
    if (negative && positive) {
        return result;
    }
    if (!plane_first && !plane()) {
        return result;
    }
    result.events[0] = {compute_normal ? wrapper_normal(mesh_normal(normal)) : cs2tl_vec3{}, fraction,
                        denominator >= 0 ? 1u : 0u};
    result.count = 1;
    result.returned_fraction = fraction;
    return result;
}
NativeGeometryResult trace_native_hull(std::span<const PhysicsPlane> planes, cs2tl_vec3 origin,
                                       cs2tl_vec3 delta, float max_fraction, float scale) {
    require(!planes.empty(), CS2TL_INCOMPLETE_GEOMETRY, "Original convex hull planes are required.");
    NativeGeometryResult result;
    const auto interval = hull_planes(planes, origin, delta, scale);
    if (interval.valid && max_fraction > interval.entry && interval.entry_plane >= 0 &&
        interval.exit > interval.entry && interval.exit_plane >= 0) {
        result.events[0] = {planes[static_cast<size_t>(interval.entry_plane)].normal, interval.entry, 0};
        result.events[1] = {planes[static_cast<size_t>(interval.exit_plane)].normal, interval.exit, 1};
    } else {
        const auto first = hull_single(planes, origin, delta, scale);
        if (!first.hit || max_fraction < first.event.fraction) {
            return result;
        }
        const auto end = f32::add(origin, delta);
        const auto reverse = hull_single(planes, end, {-delta.x, -delta.y, -delta.z}, scale);
        if (!reverse.hit) {
            return result;
        }
        const auto exit = f32::sub(1, reverse.event.fraction);
        if (!(exit > first.event.fraction)) {
            return result;
        }
        result.events[0] = first.event;
        result.events[1] = {reverse.event.normal, exit, 1};
    }
    for (auto &event : result.events) {
        event.normal = wrapper_normal(event.normal);
    }
    result.count = 2;
    result.returned_fraction = result.events[0].fraction;
    return result;
}
} // namespace cs2::traceline
