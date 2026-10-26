#pragma once
#include "NativeGeometry.h"

namespace cs2::traceline {
struct NativeMeshHit {
    float fraction{};
    uint32_t triangle{}, kind{};
};
struct NativeMeshScratch {
    std::vector<size_t> pending;
    std::vector<NativeMeshHit> hits;
    void clear() noexcept {
        pending.clear();
        hits.clear();
    }
    size_t retained_bytes() const noexcept {
        return pending.capacity() * sizeof(size_t) + hits.capacity() * sizeof(NativeMeshHit);
    }
};
void validate_native_mesh_nodes(const PhysicsCollider &collider);
// Original local-space mesh hierarchy and traversal. The audited domain is a
// unit transform with resource_flags=0. allowance is in fraction units, as in
// vphysics2.dll!0x246370; the caller decides the world-distance query policy.
float trace_native_mesh(const PhysicsCollider &collider, cs2tl_vec3 origin, cs2tl_vec3 delta,
                        float max_fraction, float allowance, NativeMeshScratch &scratch,
                        float event_fraction_limit = 1);
} // namespace cs2::traceline
