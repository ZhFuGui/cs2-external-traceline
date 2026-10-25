#pragma once
#include "geometry/Aabb.h"
#include <cstdint>
#include <vector>

namespace cs2::traceline {
struct Triangle {
    Vec3 a, b, c;
    int32_t surface{-1};
    uint32_t object{}, solid{}, id{};
    Aabb bounds() const {
        Aabb bnd;
        bnd.extend(a);
        bnd.extend(b);
        bnd.extend(c);
        return bnd;
    }
    Vec3 normal() const {
        const auto n = (b - a).cross(c - a);
        return n / n.length();
    }
};
struct Solid {
    uint32_t object{};
    bool closed{};
    Aabb bounds;
    std::vector<uint32_t> triangles;
};
struct BvhNode {
    Aabb bounds;
    uint32_t first{}, count{}, left{}, right{};
    bool leaf() const { return count != 0; }
};
struct BuildOptions {
    uint32_t flags{}, leaf_size{4};
};
struct Scene {
    std::vector<Triangle> triangles;
    std::vector<Solid> solids;
    std::vector<BvhNode> nodes;
    std::vector<uint32_t> order;
    uint32_t flags{};
    uint32_t leaf_size{4};
    uint64_t fingerprint{};
    std::vector<uint32_t> surface_hashes;
};
Scene build_scene(std::vector<Triangle> triangles, BuildOptions options = {}, bool build_acceleration = true);
} // namespace cs2::traceline
