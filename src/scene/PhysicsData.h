#pragma once
#include "cs2-external-traceline/types.h"
#include <array>
#include <cstdint>
#include <vector>

namespace cs2::traceline {
// Lossless inputs for the native collision path. These hashes are source
// identifiers, not preassigned interaction bits or collision-group ordinals.
struct PhysicsAttribute {
    uint32_t group_hash{}, include_detail_count{}, flags{};
    std::vector<uint32_t> interact_as, interact_with, exclude, detail_layers;
};
enum class PhysicsShape : uint32_t { mesh, hull, sphere, capsule };
struct PhysicsTriangle {
    cs2tl_vec3 a{}, b{}, c{};
    int32_t surface{-1};
};
struct PhysicsPlane {
    cs2tl_vec3 normal{};
    float distance{};
};
struct PhysicsMeshNode {
    cs2tl_vec3 low{};
    uint32_t data{};
    cs2tl_vec3 high{};
    uint32_t first{};
};
static_assert(sizeof(PhysicsMeshNode) == 32);
struct PhysicsCollider {
    PhysicsShape shape{};
    uint32_t object{}, attribute{CS2TL_INVALID_ID}, flags{}, resource_flags{};
    int32_t surface{-1};
    std::array<float, 12> pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    cs2tl_vec3 a{}, b{};
    float radius{};
    // Original local-space winding/planes. The separate geometric path may
    // repair winding and transform vertices; that must not alter these inputs.
    std::vector<PhysicsTriangle> triangles;
    std::vector<PhysicsPlane> planes;
    cs2tl_vec3 bounds_min{}, bounds_max{};
    // Source mesh hierarchy preserves the original candidate/triangle order.
    std::vector<uint8_t> mesh_nodes;
};
struct PhysicsData {
    std::vector<PhysicsAttribute> attributes;
    std::vector<PhysicsCollider> colliders;
};
inline constexpr uint32_t physics_attribute_complete = 1;
inline constexpr uint32_t physics_legacy_cover = 1;
inline constexpr uint32_t physics_approximate = 2;
inline constexpr uint32_t physics_bounds_available = 4;
} // namespace cs2::traceline
