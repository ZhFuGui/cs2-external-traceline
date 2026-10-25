#include "Bvh.h"
#include <algorithm>
#include <bit>
#include <numeric>

namespace cs2::traceline {
namespace {
constexpr uint32_t small_solid_limit = 32;
uint64_t node_count(uint32_t count, uint32_t leaf_size) {
    if (!count) {
        return 0;
    }
    if (count <= leaf_size) {
        return 1;
    }
    const uint32_t width = std::bit_floor(count / leaf_size);
    const uint32_t leaves = count / width == leaf_size ? width + count % width : 2 * width;
    return uint64_t(leaves) * 2 - 1;
}
template <class Bounds>
uint32_t build_index(std::vector<IndexNode> &nodes, std::vector<uint32_t> &order, uint32_t first,
                     uint32_t count, uint32_t leaf_size, Bounds &&bounds) {
    const uint32_t index = static_cast<uint32_t>(nodes.size());
    nodes.emplace_back();
    Aabb box, centers;
    for (uint32_t i = first; i < first + count; ++i) {
        const auto b = bounds(order[i]);
        box.extend(b);
        centers.extend(b.center());
    }
    nodes[index].lo = box.lo.external();
    nodes[index].hi = box.hi.external();
    if (count <= leaf_size) {
        nodes[index].first = first;
        nodes[index].count = count;
        return index;
    }
    const auto extent = centers.hi - centers.lo;
    const unsigned axis =
        extent.x >= extent.y && extent.x >= extent.z ? 0u : (extent.y >= extent.z ? 1u : 2u);
    const uint32_t middle = first + count / 2;
    std::nth_element(order.begin() + first, order.begin() + middle, order.begin() + first + count,
                     [&](uint32_t a, uint32_t b) {
                         const double ca = bounds(a).center()[axis], cb = bounds(b).center()[axis];
                         return ca != cb ? ca < cb : a < b;
                     });
    const uint32_t left = build_index(nodes, order, first, count / 2, leaf_size, bounds);
    const uint32_t right = build_index(nodes, order, middle, count - count / 2, leaf_size, bounds);
    nodes[index].left = left;
    nodes[index].right = right;
    return index;
}
} // namespace
void build_query_indices(Scene &scene) {
    scene.extended_queries = !scene.primitives.empty() || scene.deep_bvh;
    scene.primitive_nodes.clear();
    scene.primitive_order.resize(scene.primitives.size());
    if (!scene.primitives.empty()) {
        std::iota(scene.primitive_order.begin(), scene.primitive_order.end(), 0u);
        const auto size = static_cast<uint32_t>(scene.primitives.size());
        scene.primitive_nodes.reserve(static_cast<size_t>(node_count(size, 4)));
        build_index(scene.primitive_nodes, scene.primitive_order, 0, size, 4,
                    [&](uint32_t id) { return scene.primitives[id].bounds; });
    }
    const uint32_t count = static_cast<uint32_t>(scene.solids.size());
    scene.solid_nodes.clear();
    scene.solid_order.resize(count);
    scene.object_nodes.clear();
    scene.object_order.resize(count);
    scene.object_groups.clear();
    scene.point_nodes.clear();
    scene.point_order.clear();
    if (!count) {
        return;
    }
    std::iota(scene.solid_order.begin(), scene.solid_order.end(), 0u);
    auto solid_bounds = [&](uint32_t id) { return scene.solids[id].bounds; };
    scene.solid_nodes.reserve(static_cast<size_t>(node_count(count, 4)));
    build_index(scene.solid_nodes, scene.solid_order, 0, count, 4, solid_bounds);
    std::iota(scene.object_order.begin(), scene.object_order.end(), 0u);
    std::sort(scene.object_order.begin(), scene.object_order.end(), [&](uint32_t a, uint32_t b) {
        const auto oa = scene.solids[a].object, ob = scene.solids[b].object;
        return oa != ob ? oa < ob : a < b;
    });
    uint64_t object_nodes = 0, point_nodes = 0, point_order = 0;
    for (uint32_t first = 0; first < count;) {
        uint32_t last = first + 1;
        while (last < count && scene.solids[scene.object_order[last]].object ==
                                   scene.solids[scene.object_order[first]].object) {
            ++last;
        }
        const uint32_t group = static_cast<uint32_t>(scene.object_groups.size());
        scene.object_groups.push_back({first, last - first, CS2TL_INVALID_ID});
        if (last - first > 1) {
            object_nodes += node_count(last - first, 4);
        }
        for (uint32_t i = first; i < last; ++i) {
            auto &solid = scene.solids[scene.object_order[i]];
            solid.group = group;
            solid.container = CS2TL_INVALID_ID;
            if (!solid.triangles.empty()) {
                solid.point_root = CS2TL_INVALID_ID;
            }
            if (solid.closed && solid.triangles.size() > small_solid_limit &&
                solid.triangles.size() != scene.triangles.size()) {
                point_nodes += node_count(static_cast<uint32_t>(solid.triangles.size()), 8);
                point_order += solid.triangles.size();
            }
        }
        first = last;
    }
    scene.object_nodes.reserve(static_cast<size_t>(object_nodes));
    scene.point_nodes.reserve(static_cast<size_t>(point_nodes));
    scene.point_order.reserve(static_cast<size_t>(point_order));
    for (auto &group : scene.object_groups) {
        if (group.count > 1) {
            group.root = build_index(scene.object_nodes, scene.object_order, group.first, group.count, 4,
                                     solid_bounds);
        }
    }
    for (uint32_t id = 0; id < count; ++id) {
        auto &solid = scene.solids[id];
        const auto &group = scene.object_groups[solid.group];
        if (group.count > 1) {
            visit_index(
                scene.object_nodes, scene.object_order, group.root,
                [&](const Aabb &bounds) { return bounds.contains(solid.bounds); },
                [&](uint32_t other) {
                    const auto &candidate = scene.solids[other];
                    if (other != id && !(candidate.native_surface_mesh && !candidate.closed) &&
                        candidate.bounds.contains(solid.bounds)) {
                        solid.container = other;
                        return false;
                    }
                    return true;
                });
        }
        if (solid.closed && solid.triangles.size() > small_solid_limit &&
            solid.triangles.size() != scene.triangles.size()) {
            const uint32_t first = static_cast<uint32_t>(scene.point_order.size());
            scene.point_order.insert(scene.point_order.end(), solid.triangles.begin(), solid.triangles.end());
            solid.point_root = build_index(
                scene.point_nodes, scene.point_order, first, static_cast<uint32_t>(solid.triangles.size()), 8,
                [&](uint32_t triangle) { return scene.triangles[triangle].bounds(); });
        }
    }
}
} // namespace cs2::traceline
