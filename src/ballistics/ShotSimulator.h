#pragma once
#include "WeaponProfile.h"
#include "profiles/BallisticsProfiles.h"
#include "scene/Scene.h"
#include "trace/TraceQuery.h"
namespace cs2::traceline {
struct MaterialLookup {
    std::span<const ResolvedMaterial> bound;
    std::span<const cs2tl_surface_profile> explicit_profiles;
    ResolvedMaterial get(int32_t surface) const;
};
void reset_prediction(cs2tl_shot_prediction &result) noexcept;
void reset_shot_result(cs2tl_shot_result &result) noexcept;
// Ordinary material scalar block, server.dll RVA 0xa9d73a..0xa9d881.
// Caller supplies validated material/weapon inputs and a certified thickness.
float penetration_damage(float damage, float power, float thickness, float minimum_penetration,
                         const ResolvedMaterial &entry, const ResolvedMaterial &exit) noexcept;
void simulate_remaining_damage(const Scene &scene, const Ray &ray, const WeaponProfile &weapon,
                               const MaterialLookup &materials, QueryWorkspace &workspace,
                               cs2tl_shot_prediction &result);
void simulate_shot(const Scene &scene, const cs2tl_shot_request &request, cs2tl_shot_result &result,
                   QueryWorkspace &workspace);
} // namespace cs2::traceline
