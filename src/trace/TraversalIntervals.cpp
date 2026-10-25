#include "TraceQuery.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <type_traits>

namespace cs2::traceline {
template <bool Curved, bool Precise = false>
std::span<const std::conditional_t<Precise, PreciseInterval, cs2tl_interval>>
query_intervals_impl(const Scene &scene, const Ray &ray, QueryWorkspace &workspace) {
    require((scene.flags & CS2TL_SCENE_APPROXIMATE) == 0, CS2TL_INCOMPLETE_GEOMETRY,
            "Exact thickness is unavailable for a scene containing approximated hulls.");
    auto &result = [&]() -> auto & {
        if constexpr (Precise) {
            return workspace.precise_intervals;
        } else {
            return workspace.intervals;
        }
    }();
    result.clear();
    auto &candidates = workspace.candidates;
    candidates.clear();
    visit_index(
        scene.solid_nodes, scene.solid_order, 0,
        [&](const Aabb &bounds) {
            double entry{};
            return intersects(bounds, ray, ray.low, ray.high, entry);
        },
        [&](uint32_t id) {
            double entry{};
            if (intersects(scene.solids[id].bounds, ray, ray.low, ray.high, entry)) {
                candidates.push_back(id);
            }
            return true;
        });
    std::sort(candidates.begin(), candidates.end());
    if (candidates.empty()) {
        return result;
    }
    for (const auto id : candidates) {
        require_lazy(scene.solids[id].closed, CS2TL_INCOMPLETE_GEOMETRY, [&] {
            return "Segment overlaps the bounds of open/nonmanifold solid " + std::to_string(id);
        });
    }
    // A disconnected inner shell may represent a cavity rather than another
    // filled solid. The text format supplies no explicit CSG semantics here.
    // Reject this ambiguity instead of assigning false thickness to empty space.
    // If a candidate has a same-object containing box, that containing box MUST
    // also intersect this segment. One precomputed witness exactly replaces the
    // old candidate-pair predicate; no pair list or quadratic query is needed.
    for (const auto id : candidates) {
        require(scene.solids[id].container == CS2TL_INVALID_ID, CS2TL_INCOMPLETE_GEOMETRY,
                "Nested shells of one source object require explicit solid/cavity decomposition.");
    }
    auto hits = Curved ? trace_all_into(scene, ray, workspace, false)
                       : trace_all_triangles_into(scene, ray, workspace, false);
    size_t cursor = 0;
    for (uint32_t id : candidates) {
        const auto &solid = scene.solids[id];
        while (cursor < hits.size() && hit_solid_as<Curved>(scene, hits[cursor]) < id) {
            ++cursor;
        }
        const size_t first_event = cursor;
        while (cursor < hits.size() && hit_solid_as<Curved>(scene, hits[cursor]) == id) {
            ++cursor;
        }
        const auto events = hits.subspan(first_event, cursor - first_event);
        const double first = events.empty() ? ray.high : events.front().distance;
        bool inside = contains_as<Curved>(scene, solid, ray.at(ray.low + (first - ray.low) * 0.5));
        double entry = ray.low;
        int32_t entry_surface = CS2TL_UNKNOWN_SURFACE;
        uint32_t flags = inside ? CS2TL_INTERVAL_START_INSIDE : 0u;
        auto emit = [&](double exit, int32_t exit_surface, uint32_t extra_flags) {
            if (exit <= entry) {
                return;
            }
            // A ray sliding exactly along a face/edge contacts its end faces,
            // but traverses no interior volume. Do not invent wall thickness.
            if (!contains_as<Curved>(scene, solid, ray.at(entry + (exit - entry) * 0.5))) {
                return;
            }
            if constexpr (Precise) {
                result.push_back({entry, exit, flags | extra_flags, entry_surface, exit_surface, id});
            } else {
                result.push_back({static_cast<float>(entry), static_cast<float>(exit),
                                  static_cast<float>(exit - entry), flags | extra_flags,
                                  ray.at(entry).external(), ray.at(exit).external(), entry_surface,
                                  exit_surface, solid.object, id});
            }
        };
        for (const auto &event : events) {
            const uint32_t kind = Curved ? event.type() : event.kind;
            if (kind == CS2TL_HIT_TANGENT) {
                continue;
            }
            const auto surface = hit_surface_as<Curved>(scene, event);
            if (kind == CS2TL_HIT_ENTER) {
                require(!inside, CS2TL_INCOMPLETE_GEOMETRY,
                        "Inconsistent entry/exit topology; decrease merge tolerance or inspect geometry.");
                inside = true;
                entry = event.distance;
                entry_surface = surface;
                flags = 0;
            } else {
                require(inside, CS2TL_INCOMPLETE_GEOMETRY,
                        "Inconsistent exit/entry topology; inspect geometry.");
                emit(event.distance, surface, 0);
                inside = false;
            }
        }
        if (inside) {
            emit(ray.high, CS2TL_UNKNOWN_SURFACE, CS2TL_INTERVAL_END_INSIDE);
        }
    }
    std::sort(result.begin(), result.end(), [](const auto &a, const auto &b) {
        return a.entry_distance != b.entry_distance ? a.entry_distance < b.entry_distance
                                                    : a.solid_id < b.solid_id;
    });
    return result;
}
std::span<const cs2tl_interval> query_intervals_into(const Scene &scene, const Ray &ray,
                                                     QueryWorkspace &workspace) {
    return !scene.extended_queries ? query_intervals_impl<false>(scene, ray, workspace)
                                   : query_intervals_impl<true>(scene, ray, workspace);
}
std::span<const PreciseInterval> query_precise_intervals_into(const Scene &scene, const Ray &ray,
                                                              QueryWorkspace &workspace) {
    return !scene.extended_queries ? query_intervals_impl<false, true>(scene, ray, workspace)
                                   : query_intervals_impl<true, true>(scene, ray, workspace);
}
std::vector<cs2tl_interval> query_intervals(const Scene &scene, const Ray &ray) {
    QueryWorkspace workspace;
    query_intervals_into(scene, ray, workspace);
    return std::move(workspace.intervals);
}
} // namespace cs2::traceline
