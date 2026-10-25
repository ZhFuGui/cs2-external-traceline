#pragma once
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
};
struct QueryWorkspace {
    std::vector<Hit> raw_hits, hits;
    std::vector<uint32_t> candidates, inside;
    std::vector<cs2tl_interval> intervals;
    void clear_results() noexcept {
        raw_hits.clear();
        hits.clear();
        candidates.clear();
        inside.clear();
        intervals.clear();
    }
    size_t retained_bytes() const {
        return (raw_hits.capacity() + hits.capacity()) * sizeof(Hit) +
               (candidates.capacity() + inside.capacity()) * sizeof(uint32_t) +
               intervals.capacity() * sizeof(cs2tl_interval);
    }
};
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
cs2tl_hit external_hit(const Scene &scene, const Ray &ray, const Hit &hit);
std::vector<cs2tl_interval> query_intervals(const Scene &scene, const Ray &ray);
} // namespace cs2::traceline
