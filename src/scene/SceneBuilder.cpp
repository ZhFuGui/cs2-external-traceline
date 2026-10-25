#include "Scene.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include "geometry/Intersection.h"
#include <array>
#include <bit>
#include <limits>
#include <memory_resource>
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
struct Neighbor {
    uint32_t triangle{};
    bool parity{};
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

Scene build_scene(std::vector<Triangle> triangles, BuildOptions options, bool build_acceleration,
                  std::vector<Primitive> primitives, bool preserve_orientation) {
    require(triangles.size() <= UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
            "Triangle indices exceed uint32 representation.");
    if (triangles.size() > large_triangle_notice) {
        warn("Large scene (>4000000 triangles); processing continues.");
    }
    require(options.leaf_size >= 1, CS2TL_INVALID_ARGUMENT, "BVH leaf size must be positive.");
    if (options.leaf_size > 64) {
        warn("Large BVH leaf size (>64) may reduce query performance; processing continues.");
    }
    Scene scene;
    scene.leaf_size = options.leaf_size;
    scene.triangles = std::move(triangles);
    auto &tris = scene.triangles;
    const auto count = static_cast<uint32_t>(tris.size());
    {
        Components components(count);
        // Only construction temporaries use this arena; no Scene allocation
        // refers to it after this scope. Buckets are reserved before inserts.
        std::pmr::monotonic_buffer_resource arena;
        std::pmr::unordered_map<VertexKey, uint32_t, VertexHash> vertices(&arena);
        std::pmr::unordered_map<uint64_t, Edge> edges(&arena);
        vertices.reserve(count);
        edges.reserve(size_t(count) * 2);
        // A triangle has three edges, each contributing at most one neighbor.
        std::vector<std::array<Neighbor, 3>> adjacent(count);
        std::vector<uint8_t> adjacent_counts(count);

        for (uint32_t i = 0; i < count; ++i) {
            auto &tri = tris[i];
            require_lazy(tri.object != CS2TL_INVALID_ID && tri.surface >= -1, CS2TL_INVALID_GEOMETRY, [&] {
                return "Invalid object or surface identifier at triangle " + std::to_string(i);
            });
            require_lazy(tri.a.finite() && tri.b.finite() && tri.c.finite(), CS2TL_INVALID_GEOMETRY,
                         [&] { return "Nonfinite vertex at triangle " + std::to_string(i); });
            for (auto *p : {&tri.a, &tri.b, &tri.c}) {
                *p = Vec3(p->external()); // The persisted/API coordinate contract is float32.
                require(p->finite(), CS2TL_INVALID_GEOMETRY, "Vertex exceeds float32 range.");
            }
            const auto ab = tri.b - tri.a, ac = tri.c - tri.a;
            require_lazy(ab.cross(ac).length() >
                             32 * std::numeric_limits<double>::epsilon() * ab.length() * ac.length(),
                         CS2TL_INVALID_GEOMETRY, [&] { return "Degenerate triangle " + std::to_string(i); });
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
        std::vector<uint32_t> solid_ids(count, CS2TL_INVALID_ID), solid_sizes;
        for (uint32_t i = 0; i < count; ++i) {
            const auto root = components.find(i);
            if (solid_ids[root] == CS2TL_INVALID_ID) {
                solid_ids[root] = static_cast<uint32_t>(scene.solids.size());
                Solid solid;
                solid.object = tris[i].object;
                solid.closed = true;
                scene.solids.push_back(std::move(solid));
                solid_sizes.push_back(0);
            }
            const uint32_t solid_id = solid_ids[root];
            tris[i].solid = solid_id;
            auto &solid = scene.solids[solid_id];
            ++solid_sizes[solid_id];
            solid.bounds.extend(tris[i].bounds());
        }
        for (size_t i = 0; i < scene.solids.size(); ++i) {
            scene.solids[i].triangles.reserve(solid_sizes[i]);
        }
        for (uint32_t i = 0; i < count; ++i) {
            scene.solids[tris[i].solid].triangles.push_back(i);
        }
        auto add_neighbor = [&](uint32_t from, uint32_t to, bool parity) {
            auto &size = adjacent_counts[from];
            require(size < 3, CS2TL_INTERNAL_ERROR, "Triangle adjacency exceeds three edges.");
            adjacent[from][size++] = {to, parity};
        };
        for (const auto &[key, edge] : edges) {
            (void)key;
            if (edge.count != 2) {
                scene.solids[tris[edge.first].solid].closed = false;
                continue;
            }
            const bool parity = edge.first_forward == edge.second_forward;
            add_neighbor(edge.first, edge.second, parity);
            add_neighbor(edge.second, edge.first, parity);
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
                for (uint8_t neighbor = 0; neighbor < adjacent_counts[i]; ++neighbor) {
                    const auto [next, parity] = adjacent[i][neighbor];
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
            if (!preserve_orientation && flips[i] == 1) {
                std::swap(tris[i].b, tris[i].c);
            }
        }
        for (auto &solid : scene.solids) {
            solid.diagonal = (solid.bounds.hi - solid.bounds.lo).length();
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
                    require(!preserve_orientation, CS2TL_CORRUPT_CACHE,
                            "Cached closed solid has inward winding.");
                    for (auto index : solid.triangles) {
                        std::swap(tris[index].b, tris[index].c);
                    }
                }
            }
            if (!solid.closed) {
                scene.flags |= CS2TL_SCENE_HAS_OPEN_GEOMETRY;
            }
        }
    }
    scene.query_triangles.reserve(count);
    scene.triangle_normals.reserve(count);
    for (const auto &tri : tris) {
        const auto e1 = tri.b - tri.a, e2 = tri.c - tri.a;
        scene.query_triangles.push_back(
            {tri.a, e1, e2, 32 * std::numeric_limits<double>::epsilon() * e1.length() * e2.length()});
        scene.triangle_normals.push_back(tri.normal());
    }
    scene.primitives = std::move(primitives);
    for (uint32_t i = 0; i < scene.primitives.size(); ++i) {
        require(scene.solids.size() < UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
                "Solid indices exceed uint32 representation.");
        auto &primitive = scene.primitives[i];
        prepare_primitive(primitive);
        primitive.solid = static_cast<uint32_t>(scene.solids.size());
        Solid solid;
        solid.object = primitive.object;
        solid.closed = true;
        solid.bounds = primitive.bounds;
        solid.diagonal = (solid.bounds.hi - solid.bounds.lo).length();
        solid.point_root = i; // Empty triangle lists identify analytic solids.
        scene.solids.push_back(std::move(solid));
    }
    if (build_acceleration) {
        build_bvh(scene);
    }
    build_query_indices(scene);
    return scene;
}
} // namespace cs2::traceline
