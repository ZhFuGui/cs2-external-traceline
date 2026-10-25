#include "Bvh.h"
#include <algorithm>

namespace cs2::traceline {
bool has_nested_inside(const Scene &scene, std::span<const uint32_t> sorted_inside) {
    for (const auto id : sorted_inside) {
        const auto &solid = scene.solids[id];
        if (solid.container == CS2TL_INVALID_ID) {
            continue;
        }
        const auto &group = scene.object_groups[solid.group];
        bool nested = false;
        visit_index(
            scene.object_nodes, scene.object_order, group.root,
            [&](const Aabb &bounds) { return bounds.contains(solid.bounds); },
            [&](uint32_t other) {
                if (other != id && scene.solids[other].bounds.contains(solid.bounds) &&
                    std::binary_search(sorted_inside.begin(), sorted_inside.end(), other)) {
                    nested = true;
                    return false;
                }
                return true;
            });
        if (nested) {
            return true;
        }
    }
    return false;
}
} // namespace cs2::traceline
