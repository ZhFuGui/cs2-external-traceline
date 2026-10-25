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
void simulate_remaining_damage(const Scene &scene, const Ray &ray, const WeaponProfile &weapon,
                               const MaterialLookup &materials, QueryWorkspace &workspace,
                               cs2tl_shot_prediction &result);
void simulate_shot(const Scene &scene, const cs2tl_shot_request &request, cs2tl_shot_result &result,
                   QueryWorkspace &workspace);
} // namespace cs2::traceline
