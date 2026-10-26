#pragma once
#include "NativeQueryExtent.h"
#include "profiles/BallisticsProfiles.h"

namespace cs2::traceline {
struct NativeDamageState {
    float damage{}, penetration_power{}, range_modifier{}, query_length{};
    uint32_t remaining_penetrations{4}, stop_reason{};
};
struct NativeMaterialProvider {
    void *user{};
    ResolvedMaterial (*get)(void *, const BallisticEvent &){};
};
enum class NativePointStatus { ready, outside_extent, origin_inside, incomplete_boundary, stopped, reached };
// A private endpoint marker selects the air prefix ending at B. It is neither
// a physical collider nor a player/hitbox, and never supplies material data.
NativePointStatus plan_native_point_energy(NativeQueryContext &context, float point_fraction,
                                           float point_distance, float weapon_range);
// a9d150 ordinary static-material domain, without live entity side effects.
// Returns true when the native step stops. Air decay has no damage<1 cutoff.
bool advance_native_damage(const NativeQueryContext &context, const BallisticSpan &span, cs2tl_vec3 origin,
                           NativeMaterialProvider materials, NativeDamageState &state);
bool precompute_native_damage(NativeQueryContext &context, cs2tl_vec3 origin,
                              NativeMaterialProvider materials, NativeDamageState &state);
NativePointStatus consume_native_point_energy(NativeQueryContext &context, cs2tl_vec3 origin,
                                              NativeMaterialProvider materials, NativeDamageState &state);
} // namespace cs2::traceline
