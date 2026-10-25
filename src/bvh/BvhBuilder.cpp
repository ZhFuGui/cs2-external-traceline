#include "Bvh.h"
#include <algorithm>
#include <array>
#include <numeric>

namespace cs2::traceline {
namespace {
class Builder {
    Scene &scene;
    std::vector<Aabb> bounds;
    std::vector<Vec3> centers;
    static constexpr size_t bin_count = 12;
    struct Bin {
        Aabb bounds;
        uint32_t count{};
    };

  public:
    explicit Builder(Scene &value) : scene(value) {
        bounds.reserve(scene.triangles.size());
        centers.reserve(scene.triangles.size());
        for (const auto &tri : scene.triangles) {
            bounds.push_back(tri.bounds());
            centers.push_back(bounds.back().center());
        }
    }
    uint32_t node(uint32_t begin, uint32_t end, uint32_t depth) {
        const uint32_t index = static_cast<uint32_t>(scene.nodes.size());
        scene.nodes.emplace_back();
        Aabb box, centroids;
        for (uint32_t i = begin; i < end; ++i) {
            box.extend(bounds[scene.order[i]]);
            centroids.extend(centers[scene.order[i]]);
        }
        scene.nodes[index].bounds = box;
        if (end - begin <= scene.leaf_size || depth >= 48) {
            scene.nodes[index].first = begin;
            scene.nodes[index].count = end - begin;
            return index;
        }
        double best = INFINITY;
        unsigned best_axis = 0;
        size_t best_split = 0;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const double extent = centroids.hi[axis] - centroids.lo[axis];
            if (extent <= 0) {
                continue;
            }
            std::array<Bin, bin_count> bins{};
            for (uint32_t i = begin; i < end; ++i) {
                const uint32_t id = scene.order[i];
                const auto b =
                    std::min(bin_count - 1, static_cast<size_t>((centers[id][axis] - centroids.lo[axis]) /
                                                                extent * bin_count));
                ++bins[b].count;
                bins[b].bounds.extend(bounds[id]);
            }
            for (size_t split = 0; split + 1 < bin_count; ++split) {
                Aabb left, right;
                uint32_t nl = 0, nr = 0;
                for (size_t b = 0; b <= split; ++b) {
                    if (bins[b].count) {
                        left.extend(bins[b].bounds);
                        nl += bins[b].count;
                    }
                }
                for (size_t b = split + 1; b < bin_count; ++b) {
                    if (bins[b].count) {
                        right.extend(bins[b].bounds);
                        nr += bins[b].count;
                    }
                }
                if (!nl || !nr) {
                    continue;
                }
                const double cost = left.area() * nl + right.area() * nr;
                if (cost < best) {
                    best = cost;
                    best_axis = axis;
                    best_split = split;
                }
            }
        }
        uint32_t middle = begin;
        if (std::isfinite(best)) {
            const double extent = centroids.hi[best_axis] - centroids.lo[best_axis];
            const auto it =
                std::partition(scene.order.begin() + begin, scene.order.begin() + end, [&](uint32_t id) {
                    const auto bin =
                        std::min(bin_count - 1,
                                 static_cast<size_t>((centers[id][best_axis] - centroids.lo[best_axis]) /
                                                     extent * bin_count));
                    return bin <= best_split;
                });
            middle = static_cast<uint32_t>(it - scene.order.begin());
        }
        if (middle == begin || middle == end) {
            middle = begin + (end - begin) / 2;
            const auto e = centroids.hi - centroids.lo;
            const unsigned axis = e.x >= e.y && e.x >= e.z ? 0u : (e.y >= e.z ? 1u : 2u);
            std::nth_element(scene.order.begin() + begin, scene.order.begin() + middle,
                             scene.order.begin() + end, [&](uint32_t a, uint32_t b) {
                                 return centers[a][axis] != centers[b][axis]
                                            ? centers[a][axis] < centers[b][axis]
                                            : a < b;
                             });
        }
        const uint32_t left = node(begin, middle, depth + 1);
        const uint32_t right = node(middle, end, depth + 1);
        scene.nodes[index].left = left;
        scene.nodes[index].right = right;
        return index;
    }
};
} // namespace
void build_bvh(Scene &scene) {
    scene.nodes.clear();
    scene.order.resize(scene.triangles.size());
    std::iota(scene.order.begin(), scene.order.end(), 0u);
    if (scene.triangles.empty()) {
        return;
    }
    scene.nodes.reserve(scene.triangles.size() * 2);
    Builder(scene).node(0, static_cast<uint32_t>(scene.triangles.size()), 0);
}
} // namespace cs2::traceline
