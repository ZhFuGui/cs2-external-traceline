#include "TraceQuery.h"
#include "diagnostics/Error.h"
#include <algorithm>

namespace cs2::traceline {
std::vector<cs2tl_interval> query_intervals(const Scene &scene, const Ray &ray) {
    require((scene.flags & CS2TL_SCENE_APPROXIMATE) == 0, CS2TL_INCOMPLETE_GEOMETRY,
            "Exact thickness is unavailable for a scene containing approximated hulls.");
    auto hits = trace_all(scene, ray);
    std::vector<std::vector<Hit>> by_solid(scene.solids.size());
    for (const auto &hit : hits) {
        by_solid[scene.triangles[hit.triangle].solid].push_back(hit);
    }
    std::vector<uint32_t> candidates;
    for (uint32_t id = 0; id < scene.solids.size(); ++id) {
        const auto &solid = scene.solids[id];
        double entry{};
        if (!intersects(solid.bounds, ray, ray.low, ray.high, entry)) {
            continue;
        }
        require(solid.closed, CS2TL_INCOMPLETE_GEOMETRY,
                "Segment overlaps the bounds of open/nonmanifold solid " + std::to_string(id));
        candidates.push_back(id);
    }
    // A disconnected inner shell may represent a cavity rather than another
    // filled solid. The text format supplies no explicit CSG semantics here.
    // Reject this ambiguity instead of assigning false thickness to empty space.
    for (size_t i = 0; i < candidates.size(); ++i) {
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            const auto &a = scene.solids[candidates[i]];
            const auto &b = scene.solids[candidates[j]];
            require(a.object != b.object || (!a.bounds.contains(b.bounds) && !b.bounds.contains(a.bounds)),
                    CS2TL_INCOMPLETE_GEOMETRY,
                    "Nested shells of one source object require explicit solid/cavity decomposition.");
        }
    }
    std::vector<cs2tl_interval> result;
    for (uint32_t id : candidates) {
        const auto &solid = scene.solids[id];
        const auto &events = by_solid[id];
        const double first = events.empty() ? ray.high : events.front().distance;
        bool inside = contains(scene, solid, ray.at(ray.low + (first - ray.low) * 0.5));
        double entry = ray.low;
        int32_t entry_surface = CS2TL_UNKNOWN_SURFACE;
        uint32_t flags = inside ? CS2TL_INTERVAL_START_INSIDE : 0u;
        auto emit = [&](double exit, int32_t exit_surface, uint32_t extra_flags) {
            if (exit <= entry) {
                return;
            }
            // A ray sliding exactly along a face/edge contacts its end faces,
            // but traverses no interior volume. Do not invent wall thickness.
            if (!contains(scene, solid, ray.at(entry + (exit - entry) * 0.5))) {
                return;
            }
            result.push_back({static_cast<float>(entry), static_cast<float>(exit),
                              static_cast<float>(exit - entry), flags | extra_flags, ray.at(entry).external(),
                              ray.at(exit).external(), entry_surface, exit_surface, solid.object, id});
        };
        for (const auto &event : events) {
            if (event.kind == CS2TL_HIT_TANGENT) {
                continue;
            }
            const auto surface = scene.triangles[event.triangle].surface;
            if (event.kind == CS2TL_HIT_ENTER) {
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
} // namespace cs2::traceline
