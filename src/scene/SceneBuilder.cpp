#include "Scene.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include <array>
#include <bit>
#include <limits>
#include <numeric>
#include <unordered_map>

namespace cs2::traceline {
namespace {
struct VertexKey {
    uint32_t x, y, z, object;
    bool operator==(const VertexKey &) const = default;
};
struct VertexHash {
    size_t operator()(const VertexKey &k) const noexcept {
        size_t h = k.object;
        for (const auto v : {k.x, k.y, k.z}) {
            h ^= v + 0x9e3779b9u + (h << 6) + (h >> 2);
        }
        return h;
    }
};
struct Edge {
    uint32_t first{}, second{UINT32_MAX}, count{};
    bool first_forward{}, second_forward{};
};
class Components {
    std::vector<uint32_t> parent, rank;

  public:
    explicit Components(uint32_t count) : parent(count), rank(count) {
        std::iota(parent.begin(), parent.end(), 0u);
    }
    uint32_t find(uint32_t i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    }
    void join(uint32_t a, uint32_t b) {
        a = find(a);
        b = find(b);
        if (a == b) {
            return;
        }
        if (rank[a] < rank[b]) {
            std::swap(a, b);
        }
        parent[b] = a;
        if (rank[a] == rank[b]) {
            ++rank[a];
        }
    }
};
uint32_t bits(double value) {
    const float f = static_cast<float>(value);
    return f == 0 ? 0u : std::bit_cast<uint32_t>(f);
}
} // namespace

Scene build_scene(std::vector<Triangle> triangles, BuildOptions options, bool build_acceleration) {
    require(triangles.size() <= max_triangles, CS2TL_INVALID_GEOMETRY, "Triangle limit exceeded.");
    require(options.leaf_size >= 1 && options.leaf_size <= 64, CS2TL_INVALID_ARGUMENT,
            "BVH leaf size must be in 1..64.");
    Scene scene;
    scene.leaf_size = options.leaf_size;
    scene.triangles = std::move(triangles);
    auto &tris = scene.triangles;
    const auto count = static_cast<uint32_t>(tris.size());
    Components components(count);
    std::unordered_map<VertexKey, uint32_t, VertexHash> vertices;
    std::unordered_map<uint64_t, Edge> edges;
    std::vector<std::vector<std::pair<uint32_t, bool>>> adjacent(count);

    for (uint32_t i = 0; i < count; ++i) {
        auto &tri = tris[i];
        require(tri.object != CS2TL_INVALID_ID && tri.surface >= -1, CS2TL_INVALID_GEOMETRY,
                "Invalid object or surface identifier at triangle " + std::to_string(i));
        require(tri.a.finite() && tri.b.finite() && tri.c.finite(), CS2TL_INVALID_GEOMETRY,
                "Nonfinite vertex at triangle " + std::to_string(i));
        for (auto *p : {&tri.a, &tri.b, &tri.c}) {
            *p = Vec3(p->external()); // The persisted/API coordinate contract is float32.
            require(p->finite(), CS2TL_INVALID_GEOMETRY, "Vertex exceeds float32 range.");
        }
        const auto ab = tri.b - tri.a, ac = tri.c - tri.a;
        require(ab.cross(ac).length() >
                    32 * std::numeric_limits<double>::epsilon() * ab.length() * ac.length(),
                CS2TL_INVALID_GEOMETRY, "Degenerate triangle " + std::to_string(i));
        tri.id = i;
        std::array<uint32_t, 3> ids{};
        const std::array points{tri.a, tri.b, tri.c};
        for (size_t v = 0; v < 3; ++v) {
            auto [it, inserted] = vertices.try_emplace(
                VertexKey{bits(points[v].x), bits(points[v].y), bits(points[v].z), tri.object},
                static_cast<uint32_t>(vertices.size()));
            (void)inserted;
            ids[v] = it->second;
        }
        for (size_t v = 0; v < 3; ++v) {
            const uint32_t a = ids[v], b = ids[(v + 1) % 3];
            const uint64_t key = (uint64_t(std::min(a, b)) << 32) | std::max(a, b);
            auto &edge = edges[key];
            if (edge.count == 0) {
                edge.first = i;
                edge.first_forward = a < b;
            } else {
                components.join(i, edge.first);
                if (edge.count == 1) {
                    edge.second = i;
                    edge.second_forward = a < b;
                }
            }
            ++edge.count;
        }
    }
    std::unordered_map<uint32_t, uint32_t> solid_ids;
    for (uint32_t i = 0; i < count; ++i) {
        const auto root = components.find(i);
        const auto [it, inserted] = solid_ids.try_emplace(root, static_cast<uint32_t>(scene.solids.size()));
        if (inserted) {
            Solid solid;
            solid.object = tris[i].object;
            solid.closed = true;
            scene.solids.push_back(std::move(solid));
        }
        const uint32_t solid_id = it->second;
        tris[i].solid = solid_id;
        auto &solid = scene.solids[solid_id];
        solid.triangles.push_back(i);
        solid.bounds.extend(tris[i].bounds());
    }
    for (const auto &[key, edge] : edges) {
        (void)key;
        if (edge.count != 2) {
            scene.solids[tris[edge.first].solid].closed = false;
            continue;
        }
        const bool parity = edge.first_forward == edge.second_forward;
        adjacent[edge.first].emplace_back(edge.second, parity);
        adjacent[edge.second].emplace_back(edge.first, parity);
    }
    std::vector<int8_t> flips(count, -1);
    std::vector<uint32_t> stack;
    for (uint32_t seed = 0; seed < count; ++seed) {
        if (flips[seed] != -1) {
            continue;
        }
        flips[seed] = 0;
        stack.push_back(seed);
        while (!stack.empty()) {
            const uint32_t i = stack.back();
            stack.pop_back();
            for (const auto &[next, parity] : adjacent[i]) {
                const int8_t desired = static_cast<int8_t>(flips[i] ^ static_cast<int8_t>(parity));
                if (flips[next] == -1) {
                    flips[next] = desired;
                    stack.push_back(next);
                } else if (flips[next] != desired) {
                    scene.solids[tris[i].solid].closed = false;
                }
            }
        }
    }
    for (uint32_t i = 0; i < count; ++i) {
        if (flips[i] == 1) {
            std::swap(tris[i].b, tris[i].c);
        }
    }
    for (auto &solid : scene.solids) {
        if (solid.closed) {
            const auto reference = tris[solid.triangles.front()].a;
            double volume = 0;
            for (const auto index : solid.triangles) {
                const auto &tri = tris[index];
                volume += (tri.a - reference).dot((tri.b - reference).cross(tri.c - reference));
            }
            if (volume == 0 || !std::isfinite(volume)) {
                solid.closed = false;
            } else if (volume < 0) {
                for (auto index : solid.triangles) {
                    std::swap(tris[index].b, tris[index].c);
                }
            }
        }
        if (!solid.closed) {
            scene.flags |= CS2TL_SCENE_HAS_OPEN_GEOMETRY;
        }
    }
    if (build_acceleration) {
        build_bvh(scene);
    }
    return scene;
}
} // namespace cs2::traceline
