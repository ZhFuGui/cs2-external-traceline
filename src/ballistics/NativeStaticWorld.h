#pragma once
#include "NativePhysicsAttributes.h"
#include "NativeWorkspace.h"
#include "WeaponProfile.h"
#include "scene/Scene.h"
#include <memory>

namespace cs2::traceline {
struct MaterialLookup;
struct NativeAttribute {
    NativeShapeFilter filter;
    cs2tl_status status{CS2TL_OK};
};
struct NativeColliderView {
    Aabb bounds;
    uint32_t source{};
    size_t normal_first{};
    cs2tl_status status{CS2TL_OK};
};
// Immutable geometric data, shared per scene.
struct NativeStaticWorld {
    const PhysicsData *source{};
    std::vector<NativeAttribute> attributes;
    std::vector<NativeColliderView> colliders;
    std::vector<cs2tl_vec3> normals;
    std::vector<BvhNode> nodes;
    std::vector<uint32_t> order;
    bool complete_attributes{true};
};
std::unique_ptr<NativeStaticWorld> build_native_static_world(const Scene &scene);
// Complete static event collection; preserves each original mesh hierarchy.
// The source-independent outer index culls candidates without limiting range.
// Same-key material ambiguities are checked at consumption, not assigned an
// invented surface priority. This is an offline static query, not live bodies.
void query_native_static_world(const NativeStaticWorld &world, cs2tl_vec3 origin, cs2tl_vec3 delta,
                               NativeWorkspace &workspace, float max_event_fraction = 1);
void simulate_native_remaining_damage(const NativeStaticWorld &world, const Ray &ray,
                                      const WeaponProfile &weapon, const MaterialLookup &materials,
                                      NativeWorkspace &workspace, cs2tl_shot_prediction &result);
} // namespace cs2::traceline
