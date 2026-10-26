#pragma once
#include "ballistics/NativeWorkspace.h"
#include "geometry/Intersection.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <span>

namespace cs2::traceline {
struct Hit {
    double distance{};
    uint32_t triangle{}, kind{};
    bool curved() const { return (kind & 0x80000000u) != 0; }
    uint32_t type() const { return kind & 0x7fffffffu; }
};
template <bool Curved> inline uint32_t hit_solid_as(const Scene &scene, const Hit &hit) {
    if constexpr (Curved) {
        if (hit.curved()) {
            return scene.primitives[hit.triangle].solid;
        }
    }
    return scene.triangles[hit.triangle].solid;
}
template <bool Curved> inline int32_t hit_surface_as(const Scene &scene, const Hit &hit) {
    if constexpr (Curved) {
        if (hit.curved()) {
            return scene.primitives[hit.triangle].surface;
        }
    }
    return scene.triangles[hit.triangle].surface;
}
inline uint32_t hit_solid(const Scene &scene, const Hit &hit) {
    return hit.curved() ? scene.primitives[hit.triangle].solid : scene.triangles[hit.triangle].solid;
}
inline int32_t hit_surface(const Scene &scene, const Hit &hit) {
    return hit.curved() ? scene.primitives[hit.triangle].surface : scene.triangles[hit.triangle].surface;
}
struct PreciseInterval {
    double entry_distance{}, exit_distance{};
    uint32_t flags{};
    int32_t entry_surface_id{}, exit_surface_id{};
    uint32_t solid_id{};
};
struct QueryWorkspace {
    std::vector<Hit> raw_hits, hits;
    std::vector<uint32_t> candidates, inside;
    std::vector<cs2tl_interval> intervals;
    std::vector<PreciseInterval> precise_intervals;
    // Allocated only for native ballistic queries; ordinary geometry queries
    // keep their existing clear path and do not initialize native buffers.
    std::unique_ptr<NativeWorkspace> native;
    void clear_results() noexcept {
        raw_hits.clear();
        hits.clear();
        candidates.clear();
        inside.clear();
        intervals.clear();
        precise_intervals.clear();
    }
    size_t retained_bytes() const {
        return (raw_hits.capacity() + hits.capacity()) * sizeof(Hit) +
               (candidates.capacity() + inside.capacity()) * sizeof(uint32_t) +
               intervals.capacity() * sizeof(cs2tl_interval) +
               precise_intervals.capacity() * sizeof(PreciseInterval) +
               (native ? native->retained_bytes() : 0);
    }
};
std::span<const PreciseInterval> query_precise_intervals_into(const Scene &scene, const Ray &ray,
                                                              QueryWorkspace &workspace);
// A scene owns its workspaces. The pool grows only with simultaneous queries,
// including reentrancy, not with thread churn or query history.
class QueryWorkspacePool {
    struct Slot {
        QueryWorkspace workspace;
        std::atomic<bool> busy{};
    };
    struct Hint {
        uint64_t identity{};
        Slot *slot{};
    };
    std::mutex mutex_;
    std::vector<std::unique_ptr<Slot>> slots_;
    uint64_t identity_;
    Slot *acquire();
    void release(Slot *slot) noexcept;
    static Hint &thread_hint();
    friend class QueryLease;

  public:
    QueryWorkspacePool();
    QueryWorkspacePool(const QueryWorkspacePool &) = delete;
    QueryWorkspacePool &operator=(const QueryWorkspacePool &) = delete;
    // Caller must quiesce queries, just as before destroying a scene.
    void release_memory();
};
class QueryLease {
    QueryWorkspacePool *pool_;
    QueryWorkspacePool::Slot *slot_;

  public:
    explicit QueryLease(QueryWorkspacePool &pool);
    ~QueryLease();
    QueryLease(const QueryLease &) = delete;
    QueryLease &operator=(const QueryLease &) = delete;
    QueryWorkspace &get() { return slot_->workspace; }
};
std::span<const Hit> trace_all_into(const Scene &scene, const Ray &ray, QueryWorkspace &workspace,
                                    bool distance_order = true);
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray, QueryWorkspace &workspace);
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray, QueryWorkspacePool &pool);
bool is_occluded(const Scene &scene, const Ray &ray, QueryWorkspace &workspace);
bool is_occluded(const Scene &scene, const Ray &ray, QueryWorkspacePool &pool);
std::span<const cs2tl_interval> query_intervals_into(const Scene &scene, const Ray &ray,
                                                     QueryWorkspace &workspace);
std::vector<Hit> trace_all(const Scene &scene, const Ray &ray);
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray);
bool is_occluded(const Scene &scene, const Ray &ray);
cs2tl_hit external_curved_hit(const Scene &scene, const Ray &ray, const Hit &hit);
template <bool Curved = true>
#if defined(_MSC_VER) && defined(NDEBUG)
__forceinline
#else
inline
#endif
    cs2tl_hit
    external_hit(const Scene &scene, const Ray &ray, const Hit &hit) {
    if constexpr (Curved) {
        if (hit.curved()) [[unlikely]] {
            return external_curved_hit(scene, ray, hit);
        }
    }
    const auto &tri = scene.triangles[hit.triangle];
    return {static_cast<float>(hit.distance),
            static_cast<float>(hit.distance / ray.length),
            ray.at(hit.distance).external(),
            scene.triangle_normals[hit.triangle].external(),
            tri.surface,
            tri.object,
            tri.solid,
            tri.id,
            hit.kind};
}
std::span<const Hit> trace_all_triangles_into(const Scene &scene, const Ray &ray, QueryWorkspace &workspace,
                                              bool distance_order);

std::vector<cs2tl_interval> query_intervals(const Scene &scene, const Ray &ray);
} // namespace cs2::traceline
