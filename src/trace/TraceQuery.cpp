#include "TraceQuery.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <array>
#include <limits>

namespace cs2::traceline {
namespace {
template <bool Extended, class Visitor>
void query_traverse(const Scene &scene, const Ray &ray, double low, double &high, Visitor &&visitor) {
    if constexpr (Extended) {
        traverse(scene, ray, low, high, visitor);
    } else {
        traverse_nodes<BvhNode>(scene.nodes, scene.order, 0, ray, low, high, visitor);
    }
}
bool shared_feature(const Triangle &a, const Triangle &b) {
    for (const auto p : std::array{a.a, a.b, a.c}) {
        for (const auto q : std::array{b.a, b.b, b.c}) {
            if (p.x == q.x && p.y == q.y && p.z == q.z) {
                return true;
            }
        }
    }
    return false;
}
template <bool Curved>
void collect(const Scene &scene, const Ray &ray, double low, double high, std::vector<Hit> &hits) {
    hits.clear();
    query_traverse<Curved>(scene, ray, low, high, [&](uint32_t index) {
        double distance{};
        if (intersects_triangle(scene.query_triangles[index], ray, distance) && distance > ray.low &&
            distance < ray.high && distance >= low && distance <= high) {
            hits.push_back({distance, index, 0});
        }
        return true;
    });
    if constexpr (Curved) {
        traverse_nodes<IndexNode>(
            scene.primitive_nodes, scene.primitive_order, 0, ray, low, high, [&](uint32_t index) {
                const auto &primitive = scene.primitives[index];
                std::array<double, 2> distances{};
                const auto count = primitive_intersections(primitive, ray, distances);
                for (size_t i = 0; i < count; ++i) {
                    const auto distance = distances[i];
                    if (distance > ray.low && distance < ray.high && distance >= low && distance <= high) {
                        const auto facing = primitive_normal(primitive, ray.at(distance)).dot(ray.direction);
                        const uint32_t kind =
                            count == 1 || std::abs(facing) <= 64 * std::numeric_limits<double>::epsilon()
                                ? CS2TL_HIT_TANGENT
                                : (facing < 0 ? CS2TL_HIT_ENTER : CS2TL_HIT_EXIT);
                        hits.push_back({distance, index, kind | 0x80000000u});
                    }
                }
                return true;
            });
    }
}
template <bool Curved>
void consolidate(const Scene &scene, const Ray &ray, std::vector<Hit> &hits, std::vector<Hit> &result,
                 bool distance_order) {
    std::sort(hits.begin(), hits.end(), [&](const Hit &a, const Hit &b) {
        const auto sa = hit_solid_as<Curved>(scene, a), sb = hit_solid_as<Curved>(scene, b);
        if (sa != sb) {
            return sa < sb;
        }
        return a.distance != b.distance ? a.distance < b.distance : a.triangle < b.triangle;
    });
    result.clear();
    const double scale =
        std::max({ray.length, std::abs(ray.origin.x), std::abs(ray.origin.y), std::abs(ray.origin.z)});
    const double coincidence =
        std::min(ray.merge_epsilon, 64 * std::numeric_limits<double>::epsilon() * scale);
    for (size_t first = 0; first < hits.size();) {
        const auto solid = hit_solid_as<Curved>(scene, hits[first]);
        Hit hit = hits[first];
        if constexpr (Curved) {
            if (hit.curved()) {
                result.push_back(hit);
                ++first;
                continue;
            }
        }
        size_t next = first;
        bool enter = false, exit = false;
        while (next < hits.size() && hit_solid_as<Curved>(scene, hits[next]) == solid &&
               hits[next].distance - hit.distance <= coincidence &&
               shared_feature(scene.triangles[hit.triangle], scene.triangles[hits[next].triangle])) {
            const double facing = scene.triangle_normals[hits[next].triangle].dot(ray.direction);
            enter |= facing < -1e-12;
            exit |= facing > 1e-12;
            // A deterministic representative preserves one real surface identifier.
            if (hits[next].triangle < hit.triangle) {
                hit.triangle = hits[next].triangle;
            }
            ++next;
        }
        hit.kind = enter && !exit ? CS2TL_HIT_ENTER : (exit && !enter ? CS2TL_HIT_EXIT : CS2TL_HIT_TANGENT);
        if (enter && exit && scene.solids[solid].closed) {
            double delta = std::max({ray.merge_epsilon * 2, ray.length * 1e-10, 1e-9});
            if (first > 0 && hit_solid_as<Curved>(scene, hits[first - 1]) == solid) {
                delta = std::min(delta, (hit.distance - hits[first - 1].distance) * 0.25);
            }
            if (next < hits.size() && hit_solid_as<Curved>(scene, hits[next]) == solid) {
                delta = std::min(delta, (hits[next].distance - hit.distance) * 0.25);
            }
            const bool before = contains_as<Curved>(scene, scene.solids[solid], ray.at(hit.distance - delta));
            const bool after = contains_as<Curved>(scene, scene.solids[solid], ray.at(hit.distance + delta));
            if (before != after) {
                hit.kind = after ? CS2TL_HIT_ENTER : CS2TL_HIT_EXIT;
            }
        }
        result.push_back(hit);
        first = next;
    }
    if (distance_order) {
        std::sort(result.begin(), result.end(), [&](const Hit &a, const Hit &b) {
            if (a.distance != b.distance) {
                return a.distance < b.distance;
            }
            const auto sa = hit_solid_as<Curved>(scene, a), sb = hit_solid_as<Curved>(scene, b);
            return sa != sb ? sa < sb : a.triangle < b.triangle;
        });
    }
}
uint64_t workspace_identity() {
    static std::atomic<uint64_t> sequence{1};
    return sequence.fetch_add(1, std::memory_order_relaxed);
}
} // namespace
QueryWorkspacePool::QueryWorkspacePool() : identity_(workspace_identity()) {
}
QueryWorkspacePool::Hint &QueryWorkspacePool::thread_hint() {
    thread_local Hint hint;
    return hint;
}
QueryWorkspacePool::Slot *QueryWorkspacePool::acquire() {
    // This non-owning hint is never dereferenced unless its unique pool identity
    // matches. Destruction/reset invalidates hints even if addresses are reused.
    auto &hint = thread_hint();
    bool expected = false;
    if (hint.identity == identity_ &&
        hint.slot->busy.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        return hint.slot;
    }
    std::lock_guard lock(mutex_);
    for (auto &slot : slots_) {
        expected = false;
        if (slot->busy.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
            hint = {identity_, slot.get()};
            return slot.get();
        }
    }
    auto slot = std::make_unique<Slot>();
    slot->busy.store(true, std::memory_order_relaxed);
    auto *result = slot.get();
    slots_.push_back(std::move(slot));
    hint = {identity_, result};
    return result;
}
void QueryWorkspacePool::release_memory() {
    std::lock_guard lock(mutex_);
    for (const auto &slot : slots_) {
        require(!slot->busy.load(std::memory_order_acquire), CS2TL_INVALID_ARGUMENT,
                "Finish scene queries before releasing query memory.");
    }
    std::vector<std::unique_ptr<Slot>>{}.swap(slots_);
    identity_ = workspace_identity();
}
void QueryWorkspacePool::release(Slot *slot) noexcept {
    slot->workspace.clear_results();
    slot->busy.store(false, std::memory_order_release);
    // Prefer the outer query's buffers when a reentrant query finishes first.
    thread_hint() = {identity_, slot};
}
QueryLease::QueryLease(QueryWorkspacePool &pool) : pool_(&pool), slot_(pool.acquire()) {
}
QueryLease::~QueryLease() {
    pool_->release(slot_);
}
std::span<const Hit> trace_all_triangles_into(const Scene &scene, const Ray &ray, QueryWorkspace &workspace,
                                              bool distance_order) {
    collect<false>(scene, ray, ray.low, ray.high, workspace.raw_hits);
    consolidate<false>(scene, ray, workspace.raw_hits, workspace.hits, distance_order);
    return workspace.hits;
}
std::span<const Hit> trace_all_into(const Scene &scene, const Ray &ray, QueryWorkspace &workspace,
                                    bool distance_order) {
    if (!scene.extended_queries) {
        return trace_all_triangles_into(scene, ray, workspace, distance_order);
    }
    collect<true>(scene, ray, ray.low, ray.high, workspace.raw_hits);
    consolidate<true>(scene, ray, workspace.raw_hits, workspace.hits, distance_order);
    return workspace.hits;
}
std::vector<Hit> trace_all(const Scene &scene, const Ray &ray) {
    QueryWorkspace workspace;
    trace_all_into(scene, ray, workspace);
    return std::move(workspace.hits);
}
namespace {
template <bool Curved> std::optional<double> nearest_distance_impl(const Scene &scene, const Ray &ray) {
    double best = ray.high;
    bool found = false;
    query_traverse<Curved>(scene, ray, ray.low, best, [&](uint32_t index) {
        double distance{};
        if (intersects_triangle(scene.query_triangles[index], ray, distance) && distance > ray.low &&
            distance < ray.high && distance <= best) {
            best = distance;
            found = true;
        }
        return true;
    });
    if constexpr (Curved) {
        traverse_nodes<IndexNode>(
            scene.primitive_nodes, scene.primitive_order, 0, ray, ray.low, best, [&](uint32_t index) {
                std::array<double, 2> distances{};
                const auto count = primitive_intersections(scene.primitives[index], ray, distances);
                for (size_t i = 0; i < count; ++i) {
                    if (distances[i] > ray.low && distances[i] < ray.high && distances[i] <= best) {
                        best = distances[i];
                        found = true;
                    }
                }
                return true;
            });
    }
    if (!found) {
        return {};
    }
    return best;
}
std::optional<double> nearest_distance(const Scene &scene, const Ray &ray) {
    return !scene.extended_queries ? nearest_distance_impl<false>(scene, ray)
                                   : nearest_distance_impl<true>(scene, ray);
}
std::optional<Hit> gather_nearest(const Scene &scene, const Ray &ray, double best,
                                  QueryWorkspace &workspace) {
    // Consolidate every surface in the first distance cluster, even when a
    // nearest-first traversal visited another adjacent triangle first.
    if (!scene.extended_queries) {
        collect<false>(scene, ray, std::max(ray.low, best - ray.merge_epsilon),
                       std::min(ray.high, best + ray.merge_epsilon), workspace.raw_hits);
        consolidate<false>(scene, ray, workspace.raw_hits, workspace.hits, true);
    } else {
        collect<true>(scene, ray, std::max(ray.low, best - ray.merge_epsilon),
                      std::min(ray.high, best + ray.merge_epsilon), workspace.raw_hits);
        consolidate<true>(scene, ray, workspace.raw_hits, workspace.hits, true);
    }
    const auto &hits = workspace.hits;
    return hits.empty() ? std::optional<Hit>{} : hits.front();
}
template <bool Curved> bool surface_occluded_impl(const Scene &scene, const Ray &ray) {
    bool hit = false;
    double high = ray.high;
    query_traverse<Curved>(scene, ray, ray.low, high, [&](uint32_t index) {
        double distance{};
        if (intersects_triangle(scene.query_triangles[index], ray, distance) && distance > ray.low &&
            distance < ray.high) {
            hit = true;
            return false;
        }
        return true;
    });
    if constexpr (Curved) {
        if (!hit) {
            traverse_nodes<IndexNode>(
                scene.primitive_nodes, scene.primitive_order, 0, ray, ray.low, high, [&](uint32_t index) {
                    std::array<double, 2> distances{};
                    const auto count = primitive_intersections(scene.primitives[index], ray, distances);
                    for (size_t i = 0; i < count; ++i) {
                        if (distances[i] > ray.low && distances[i] < ray.high) {
                            hit = true;
                            return false;
                        }
                    }
                    return true;
                });
        }
    }
    return hit;
}
bool surface_occluded(const Scene &scene, const Ray &ray) {
    return !scene.extended_queries ? surface_occluded_impl<false>(scene, ray)
                                   : surface_occluded_impl<true>(scene, ray);
}
template <bool Extended, class Workspace>
bool interior_occluded(const Scene &scene, Vec3 midpoint, Workspace &&obtain) {
    std::vector<uint32_t> *inside = nullptr;
    visit_index(
        scene.solid_nodes, scene.solid_order, 0,
        [&](const Aabb &bounds) { return bounds.contains(midpoint); },
        [&](uint32_t id) {
            if (contains_as<Extended>(scene, scene.solids[id], midpoint)) {
                if (!inside) {
                    inside = &obtain().inside;
                    inside->clear();
                }
                inside->push_back(id);
            }
            return true;
        });
    if (!inside) {
        return false;
    }
    std::sort(inside->begin(), inside->end());
    require(!has_nested_inside(scene, *inside), CS2TL_INCOMPLETE_GEOMETRY,
            "Nested shells of one source object have ambiguous interior occupancy.");
    return true;
}
} // namespace
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray, QueryWorkspace &workspace) {
    const auto best = nearest_distance(scene, ray);
    return best ? gather_nearest(scene, ray, *best, workspace) : std::optional<Hit>{};
}
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray, QueryWorkspacePool &pool) {
    const auto best = nearest_distance(scene, ray);
    if (!best) {
        return {};
    }
    QueryLease lease(pool);
    return gather_nearest(scene, ray, *best, lease.get());
}
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray) {
    QueryWorkspace workspace;
    return trace_closest(scene, ray, workspace);
}
bool is_occluded(const Scene &scene, const Ray &ray, QueryWorkspace &workspace) {
    if (surface_occluded(scene, ray)) {
        return true;
    }
    workspace.inside.clear();
    auto obtain = [&]() -> QueryWorkspace & { return workspace; };
    const auto midpoint = ray.at((ray.low + ray.high) * 0.5);
    return scene.extended_queries ? interior_occluded<true>(scene, midpoint, obtain)
                                  : interior_occluded<false>(scene, midpoint, obtain);
}
bool is_occluded(const Scene &scene, const Ray &ray, QueryWorkspacePool &pool) {
    if (surface_occluded(scene, ray)) {
        return true;
    }
    std::optional<QueryLease> lease;
    auto obtain = [&]() -> QueryWorkspace & {
        lease.emplace(pool);
        return lease->get();
    };
    const auto midpoint = ray.at((ray.low + ray.high) * 0.5);
    return scene.extended_queries ? interior_occluded<true>(scene, midpoint, obtain)
                                  : interior_occluded<false>(scene, midpoint, obtain);
}
bool is_occluded(const Scene &scene, const Ray &ray) {
    QueryWorkspace workspace;
    return is_occluded(scene, ray, workspace);
}
#if defined(_MSC_VER)
__declspec(noinline)
#endif
cs2tl_hit
external_curved_hit(const Scene &scene, const Ray &ray, const Hit &hit) {
    const auto &p = scene.primitives[hit.triangle];
    return {static_cast<float>(hit.distance),
            static_cast<float>(hit.distance / ray.length),
            ray.at(hit.distance).external(),
            primitive_normal(p, ray.at(hit.distance)).external(),
            p.surface,
            p.object,
            p.solid,
            CS2TL_INVALID_ID,
            hit.type()};
}
} // namespace cs2::traceline
