#pragma once
#include "scene/PhysicsData.h"
#include <span>

namespace cs2::traceline {
struct NativeGeometryEvent {
    cs2tl_vec3 normal{};
    float fraction{};
    uint32_t kind{};
};
struct NativeGeometryResult {
    std::array<NativeGeometryEvent, 2> events{};
    uint32_t count{};
    float returned_fraction{1};
};
cs2tl_vec3 native_triangle_normal(const PhysicsTriangle &triangle) noexcept;
// Build 26144732, vphysics2.dll 0x246370 and 0x252050. These entry points
// reproduce unit-transform wrappers in float32 source-local space. The caller
// must resolve collision attributes and any nonidentity transform separately.
NativeGeometryResult trace_native_triangle(const PhysicsTriangle &triangle, cs2tl_vec3 origin,
                                           cs2tl_vec3 delta, float max_fraction = 1,
                                           bool compute_normal = true, bool plane_first = false);
NativeGeometryResult trace_native_hull(std::span<const PhysicsPlane> planes, cs2tl_vec3 origin,
                                       cs2tl_vec3 delta, float max_fraction = 1, float scale = 1);
} // namespace cs2::traceline
