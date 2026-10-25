#pragma once
#include "geometry/Intersection.h"
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
    size_t retained_bytes() const {
        return (raw_hits.capacity() + hits.capacity()) * sizeof(Hit) +
               (candidates.capacity() + inside.capacity()) * sizeof(uint32_t) +
               intervals.capacity() * sizeof(cs2tl_interval);
    }
};
// Every calling thread reuses its own scratch. A reentrant call leases a local
// fallback. Scratch contains only values/IDs, never pointers into a scene.
class QueryLease {
    QueryWorkspace fallback_;
    QueryWorkspace *workspace_{};
    bool primary_{};

  public:
    QueryLease();
    ~QueryLease();
    QueryLease(const QueryLease &) = delete;
    QueryLease &operator=(const QueryLease &) = delete;
    QueryWorkspace &get() { return *workspace_; }
};
std::span<const Hit> trace_all_into(const Scene &scene, const Ray &ray, QueryWorkspace &workspace,
                                    bool distance_order = true);
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray, QueryWorkspace &workspace);
bool is_occluded(const Scene &scene, const Ray &ray, QueryWorkspace &workspace);
std::span<const cs2tl_interval> query_intervals_into(const Scene &scene, const Ray &ray,
                                                     QueryWorkspace &workspace);
std::vector<Hit> trace_all(const Scene &scene, const Ray &ray);
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray);
bool is_occluded(const Scene &scene, const Ray &ray);
cs2tl_hit external_hit(const Scene &scene, const Ray &ray, const Hit &hit);
std::vector<cs2tl_interval> query_intervals(const Scene &scene, const Ray &ray);
} // namespace cs2::traceline
