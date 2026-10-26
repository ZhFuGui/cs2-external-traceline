#include "NativeMesh.h"
#include "NativeMath.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace cs2::traceline {
namespace {
namespace f32 = native_f32;
PhysicsMeshNode read_node(const PhysicsCollider &collider, size_t index) noexcept {
    PhysicsMeshNode result;
    std::memcpy(&result, collider.mesh_nodes.data() + index * sizeof(result), sizeof(result));
    return result;
}
float component(cs2tl_vec3 value, uint32_t axis) noexcept {
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}
cs2tl_vec3 vector_min(cs2tl_vec3 a, cs2tl_vec3 b) noexcept {
    return {a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z};
}
cs2tl_vec3 vector_max(cs2tl_vec3 a, cs2tl_vec3 b) noexcept {
    return {a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z};
}
bool overlap(cs2tl_vec3 low, cs2tl_vec3 high, cs2tl_vec3 segment_low, cs2tl_vec3 segment_high) noexcept {
    return !(segment_high.x < low.x || segment_high.y < low.y || segment_high.z < low.z ||
             high.x < segment_low.x || high.y < segment_low.y || high.z < segment_low.z);
}
struct MeshRay {
#if defined(_MSC_VER) || defined(__SSE__)
    __m128 origin4, delta_yzx, delta_zxy, absolute_yzx, absolute_zxy, low4, high4;
    static __m128 vector(cs2tl_vec3 v) noexcept { return _mm_set_ps(0, v.z, v.y, v.x); }
    void bounds(cs2tl_vec3 first, cs2tl_vec3 last) noexcept {
        low4 = vector(first);
        high4 = vector(last);
    }
    MeshRay(cs2tl_vec3 at, cs2tl_vec3 direction, cs2tl_vec3 first, cs2tl_vec3 last) noexcept {
        origin4 = vector(at);
        const auto d = vector(direction);
        delta_yzx = _mm_shuffle_ps(d, d, _MM_SHUFFLE(3, 0, 2, 1));
        delta_zxy = _mm_shuffle_ps(d, d, _MM_SHUFFLE(3, 1, 0, 2));
        const auto sign = _mm_set1_ps(-0.0f);
        absolute_yzx = _mm_andnot_ps(sign, delta_yzx);
        absolute_zxy = _mm_andnot_ps(sign, delta_zxy);
        bounds(first, last);
    }
#else
    cs2tl_vec3 origin, delta, low, high;
    void bounds(cs2tl_vec3 first, cs2tl_vec3 last) noexcept {
        low = first;
        high = last;
    }
    MeshRay(cs2tl_vec3 at, cs2tl_vec3 direction, cs2tl_vec3 first, cs2tl_vec3 last) noexcept
        : origin(at), delta(direction), low(first), high(last) {}
#endif
};
bool triangle_box(const PhysicsTriangle &triangle, const MeshRay &ray) noexcept {
#if defined(_MSC_VER) || defined(__SSE__)
    __m128 a, b, c;
    const auto *bytes = reinterpret_cast<const uint8_t *>(&triangle);
    std::memcpy(&a, bytes, 16);
    std::memcpy(&b, bytes + 12, 16);
    std::memcpy(&c, bytes + 24, 16);
    const auto xyz = _mm_castsi128_ps(_mm_set_epi32(0, -1, -1, -1));
    a = _mm_and_ps(a, xyz);
    b = _mm_and_ps(b, xyz);
    c = _mm_and_ps(c, xyz);
    const auto margin = _mm_set1_ps(.03125f);
    const auto low = _mm_sub_ps(_mm_min_ps(a, _mm_min_ps(b, c)), margin);
    const auto high = _mm_add_ps(_mm_max_ps(a, _mm_max_ps(b, c)), margin);
    return (_mm_movemask_ps(_mm_or_ps(_mm_cmplt_ps(ray.high4, low), _mm_cmplt_ps(high, ray.low4))) & 7) == 0;
#else
    const auto low =
        f32::sub(vector_min(triangle.a, vector_min(triangle.b, triangle.c)), {.03125f, .03125f, .03125f});
    const auto high =
        f32::add(vector_max(triangle.a, vector_max(triangle.b, triangle.c)), {.03125f, .03125f, .03125f});
    return overlap(low, high, ray.low, ray.high);
#endif
}
#if defined(_MSC_VER)
__forceinline
#else
inline
#endif
    bool
    line_box(const PhysicsMeshNode &node, const MeshRay &ray) noexcept {
#if defined(_MSC_VER) || defined(__SSE__)
    // Lane-wise SSE preserves the scalar float32 operation order. The packed
    // node's integer fourth lanes are masked before any floating arithmetic.
    __m128 low, high;
    std::memcpy(&low, &node, 16);
    std::memcpy(&high, reinterpret_cast<const uint8_t *>(&node) + 16, 16);
    const auto xyz = _mm_castsi128_ps(_mm_set_epi32(0, -1, -1, -1));
    low = _mm_and_ps(low, xyz);
    high = _mm_and_ps(high, xyz);
    if ((_mm_movemask_ps(_mm_or_ps(_mm_cmplt_ps(ray.high4, low), _mm_cmplt_ps(high, ray.low4))) & 7) != 0) {
        return false;
    }
    const auto center = _mm_mul_ps(_mm_add_ps(high, low), _mm_set1_ps(.5f));
    const auto extent = _mm_sub_ps(high, center), offset = _mm_sub_ps(ray.origin4, center);
    const auto offset_yzx = _mm_shuffle_ps(offset, offset, _MM_SHUFFLE(3, 0, 2, 1));
    const auto offset_zxy = _mm_shuffle_ps(offset, offset, _MM_SHUFFLE(3, 1, 0, 2));
    const auto cross =
        _mm_sub_ps(_mm_mul_ps(offset_yzx, ray.delta_zxy), _mm_mul_ps(offset_zxy, ray.delta_yzx));
    const auto extent_zxy = _mm_shuffle_ps(extent, extent, _MM_SHUFFLE(3, 1, 0, 2));
    const auto extent_yzx = _mm_shuffle_ps(extent, extent, _MM_SHUFFLE(3, 0, 2, 1));
    const auto radius =
        _mm_add_ps(_mm_mul_ps(extent_zxy, ray.absolute_yzx), _mm_mul_ps(extent_yzx, ray.absolute_zxy));
    const auto absolute_cross = _mm_andnot_ps(_mm_set1_ps(-0.0f), cross);
    return (_mm_movemask_ps(_mm_cmple_ps(_mm_sub_ps(absolute_cross, radius), _mm_setzero_ps())) & 7) == 7;
#else
    const auto origin = ray.origin, delta = ray.delta, segment_low = ray.low, segment_high = ray.high;
    if (!overlap(node.low, node.high, segment_low, segment_high)) {
        return false;
    }
    const auto center = f32::mul(f32::add(node.high, node.low), .5f);
    const auto extent = f32::sub(node.high, center), offset = f32::sub(origin, center);
    const auto cross = f32::cross(offset, delta);
    const auto x = f32::add(f32::mul(extent.z, std::abs(delta.y)), f32::mul(extent.y, std::abs(delta.z)));
    const auto y = f32::add(f32::mul(extent.x, std::abs(delta.z)), f32::mul(extent.z, std::abs(delta.x)));
    const auto z = f32::add(f32::mul(extent.y, std::abs(delta.x)), f32::mul(extent.x, std::abs(delta.y)));
    return f32::sub(std::abs(cross.x), x) <= 0 && f32::sub(std::abs(cross.y), y) <= 0 &&
           f32::sub(std::abs(cross.z), z) <= 0;
#endif
}
} // namespace
void validate_native_mesh_nodes(const PhysicsCollider &collider) {
    require(collider.mesh_nodes.size() % sizeof(PhysicsMeshNode) == 0, CS2TL_INVALID_GEOMETRY,
            "Invalid source mesh node byte count.");
    const auto count = collider.mesh_nodes.size() / sizeof(PhysicsMeshNode);
    if (!count) {
        return;
    }
    std::vector<bool> seen(count);
    std::vector<size_t> pending{0};
    while (!pending.empty()) {
        const auto index = pending.back();
        pending.pop_back();
        require(index < count && !seen[index], CS2TL_INVALID_GEOMETRY,
                "Invalid or overlapping source mesh hierarchy.");
        seen[index] = true;
        const auto node = read_node(collider, index);
        for (uint32_t axis = 0; axis < 3; ++axis) {
            require(std::isfinite(component(node.low, axis)) && std::isfinite(component(node.high, axis)) &&
                        component(node.low, axis) <= component(node.high, axis),
                    CS2TL_INVALID_GEOMETRY, "Invalid source mesh node bounds.");
        }
        const auto payload = node.data & 0x3fffffffu;
        if ((node.data >> 30) == 3) {
            require(uint64_t(node.first) + payload <= collider.triangles.size(), CS2TL_INVALID_GEOMETRY,
                    "Source mesh leaf exceeds its triangle table.");
        } else {
            require(payload > 1 && payload < count - index && index + 1 < count, CS2TL_INVALID_GEOMETRY,
                    "Invalid source mesh child offset.");
            pending.push_back(index + payload);
            pending.push_back(index + 1);
        }
    }
    require(std::all_of(seen.begin(), seen.end(), [](bool value) { return value; }), CS2TL_INVALID_GEOMETRY,
            "Unreachable source mesh nodes.");
}
float trace_native_mesh(const PhysicsCollider &collider, cs2tl_vec3 origin, cs2tl_vec3 delta,
                        float max_fraction, float allowance, NativeMeshScratch &scratch,
                        float event_fraction_limit) {
    scratch.clear();
    require(collider.shape == PhysicsShape::mesh && collider.resource_flags == 0, CS2TL_INCOMPLETE_GEOMETRY,
            "Native mesh flags are outside the audited local query domain.");
    if (collider.triangles.empty()) {
        return 1;
    }
    // Full static queries never shorten their traversal window (allowance>=1).
    // A point prefix may reject an exact native plane fraction before the edge
    // tests; this changes no accepted <=B hit and never shortens node bounds.
    // Unmasked FP exceptions retain the original edge-first evaluation order.
    const bool prefix_only =
        event_fraction_limit < 1 && allowance >= 1 && (f32::control() & 0x1f80u) == 0x1f80u;
    const auto fraction_limit = prefix_only ? std::min(max_fraction, event_fraction_limit) : max_fraction;
    require(!collider.mesh_nodes.empty(), CS2TL_INCOMPLETE_GEOMETRY,
            "Original source mesh hierarchy is unavailable.");
    // The mutable traversal window starts at one. The original max_fraction
    // remains a separate per-triangle acceptance bound even after this window
    // shrinks; a large leaf may therefore still emit a farther triangle.
    float limit = 1;
    auto segment_end = f32::add(origin, f32::mul(delta, limit));
    auto segment_low = vector_min(origin, segment_end), segment_high = vector_max(origin, segment_end);
    MeshRay mesh_ray(origin, delta, segment_low, segment_high);
    size_t index = 0;
    for (;;) {
        const auto node = read_node(collider, index);
        if (line_box(node, mesh_ray)) {
            const auto payload = node.data & 0x3fffffffu, axis = node.data >> 30;
            if (axis != 3) {
                const auto first_child = component(delta, axis) > 0 ? index + 1 : index + payload;
                const auto second_child = component(delta, axis) > 0 ? index + payload : index + 1;
                scratch.pending.push_back(second_child);
                index = first_child;
                continue;
            }
            for (uint32_t i = 0; i < payload; ++i) {
                const auto triangle_id = node.first + i;
                const auto &triangle = collider.triangles[triangle_id];
                if (!triangle_box(triangle, mesh_ray)) {
                    continue;
                }
                const auto hit =
                    trace_native_triangle(triangle, origin, delta, fraction_limit, false, prefix_only);
                if (!hit.count) {
                    continue;
                }
                scratch.hits.push_back({hit.events[0].fraction, triangle_id, hit.events[0].kind});
                const auto next_limit = f32::add(hit.events[0].fraction, allowance);
                if (limit > next_limit) {
                    limit = next_limit;
                    segment_end = f32::add(origin, f32::mul(delta, limit));
                    segment_low = vector_min(origin, segment_end);
                    segment_high = vector_max(origin, segment_end);
                    mesh_ray.bounds(segment_low, segment_high);
                }
            }
        }
        if (scratch.pending.empty()) {
            break;
        }
        index = scratch.pending.back();
        scratch.pending.pop_back();
    }
    std::sort(scratch.hits.begin(), scratch.hits.end(),
              [](const auto &a, const auto &b) { return a.fraction < b.fraction; });
    return scratch.hits.empty() ? 1 : scratch.hits.front().fraction;
}
} // namespace cs2::traceline
