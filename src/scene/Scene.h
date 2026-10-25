#pragma once
#include "geometry/Aabb.h"
#include <array>
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
    uint32_t group{CS2TL_INVALID_ID};
    uint32_t container{CS2TL_INVALID_ID};
    uint32_t point_root{CS2TL_INVALID_ID};
    double diagonal{};
};
// An affine image of a capsule. Equal endpoints encode a sphere. Curved
// surfaces stay analytic: tessellation would alter visibility and thickness.
struct Primitive {
    Vec3 a, b;
    double radius{};
    int32_t surface{-1};
    uint32_t object{}, solid{};
    std::array<double, 12> pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    std::array<double, 12> inverse{};
    Aabb bounds;
};
// Query-only data is prepared after quantization and winding repair. Keeping it
// separate avoids fetching topology/source vertices for each intersection.
struct TriangleQuery {
    Vec3 a, edge1, edge2;
    double tolerance{};
};
struct BvhNode {
    Aabb bounds;
    uint32_t first{}, count{}, left{}, right{};
    bool leaf() const { return count != 0; }
};
struct BuildOptions {
    uint32_t flags{}, leaf_size{4};
};
// Bounds are extrema of float32 vertices; slab arithmetic remains double.
struct IndexNode {
    cs2tl_vec3 lo{}, hi{};
    uint32_t first{}, count{}, left{}, right{};
    bool leaf() const { return count != 0; }
    Aabb bounds() const { return {Vec3(lo), Vec3(hi)}; }
};
struct ObjectGroup {
    uint32_t first{}, count{}, root{CS2TL_INVALID_ID};
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
    std::vector<IndexNode> solid_nodes;
    std::vector<uint32_t> solid_order;
    std::vector<ObjectGroup> object_groups;
    std::vector<IndexNode> object_nodes;
    std::vector<uint32_t> object_order;
    std::vector<IndexNode> point_nodes;
    std::vector<uint32_t> point_order;
    std::vector<TriangleQuery> query_triangles;
    std::vector<Vec3> triangle_normals;
    std::vector<Primitive> primitives;
    std::vector<IndexNode> primitive_nodes;
    std::vector<uint32_t> primitive_order;
    bool deep_bvh{}; // Unusually deep external caches use traversal stack spill.
    bool extended_queries{};
};
Scene build_scene(std::vector<Triangle> triangles, BuildOptions options = {}, bool build_acceleration = true,
                  std::vector<Primitive> primitives = {}, bool preserve_orientation = false);
} // namespace cs2::traceline
