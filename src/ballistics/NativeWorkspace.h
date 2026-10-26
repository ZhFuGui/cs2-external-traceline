#pragma once
#include "NativeDamage.h"
#include "NativeMesh.h"
#include "geometry/Aabb.h"

namespace cs2::traceline {
struct NativeWorkspace {
    NativeQueryContext query;
    NativeMeshScratch mesh;
    // Exactly one scalar result per simultaneous workspace. No profile/data
    // pointers are retained, and thread churn cannot accumulate entries.
    struct Flight {
        Ray ray{};
        cs2tl_shot_prediction result{};
        uint64_t identity{};
        uint32_t weapon{}, mode{};
        uint32_t float_control{};
    } flight;
    size_t retained_bytes() const noexcept {
        return query.events.capacity() * sizeof(BallisticEvent) +
               query.spans.capacity() * sizeof(BallisticSpan) + mesh.retained_bytes();
    }
};
} // namespace cs2::traceline
