#include "ShotSimulator.h"
#include "diagnostics/Error.h"
#include "profiles/Ruleset.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace cs2::traceline {
void reset_prediction(cs2tl_shot_prediction &out) noexcept {
    const auto size = out.struct_size;
    out = {};
    out.struct_size = size;
    out.outcome = CS2TL_SHOT_NOT_COMPUTED;
    out.remaining_damage = std::numeric_limits<float>::quiet_NaN();
    out.ruleset_version = CS2TL_RULESET_CS2_STATIC_V1;
}
void reset_shot_result(cs2tl_shot_result &out) noexcept {
    const auto size = out.struct_size;
    out = {};
    out.struct_size = size;
    out.remaining_damage = out.target_damage = std::numeric_limits<float>::quiet_NaN();
}
ResolvedMaterial MaterialLookup::get(int32_t id) const {
    require(id >= 0, CS2TL_DATA_NOT_FOUND, "A traversed boundary has an unknown surface identifier.");
    if (!bound.empty()) {
        require(static_cast<size_t>(id) < bound.size() && bound[id].valid, CS2TL_DATA_NOT_FOUND,
                "A traversed surface hash has no resolved material profile.");
        return bound[id];
    }
    const cs2tl_surface_profile *profile = nullptr;
    if (static_cast<size_t>(id) < explicit_profiles.size() && explicit_profiles[id].surface_id == id) {
        profile = &explicit_profiles[id];
    } else {
        const auto it = std::find_if(explicit_profiles.begin(), explicit_profiles.end(),
                                     [&](const auto &m) { return m.surface_id == id; });
        if (it != explicit_profiles.end()) {
            profile = &*it;
        }
    }
    require(profile != nullptr, CS2TL_DATA_NOT_FOUND,
            "A traversed surface has no supplied material profile.");
    return {profile->penetration_modifier, profile->damage_modifier, profile->flags & UINT16_MAX,
            (profile->flags >> 16) & CS2TL_MATERIAL_NATIVE_FLAG13, true};
}
namespace {
void validate(const WeaponProfile &w) {
    require(w.struct_size >= sizeof(w) && std::isfinite(w.base_damage) && w.base_damage > 0 &&
                std::isfinite(w.max_range) && w.max_range > 0 && std::isfinite(w.range_modifier) &&
                w.range_modifier > 0 && std::isfinite(w.penetration_power) && w.penetration_power >= 0 &&
                w.pellets_per_shot > 0,
            CS2TL_INVALID_ARGUMENT, "Invalid bullet weapon profile.");
}
float attenuate(float damage, float air_distance, float modifier) {
    const auto value =
        air_distance > 0 ? damage * std::pow(modifier, air_distance / range_decay_units) : damage;
    require(std::isfinite(value), CS2TL_INVALID_ARGUMENT,
            "Damage attenuation exceeds the finite float32 range.");
    return value;
}
} // namespace
void simulate_remaining_damage(const Scene &scene, const Ray &ray, const WeaponProfile &weapon,
                               const MaterialLookup &materials, QueryWorkspace &workspace,
                               cs2tl_shot_prediction &out) {
    reset_prediction(out);
    validate(weapon);
    auto stopped = [&](uint32_t reason, uint32_t count) {
        out.outcome = CS2TL_SHOT_STOPPED;
        out.remaining_damage = 0;
        out.stop_reason = reason;
        out.penetration_count = count;
    };
    if (ray.length > weapon.max_range) {
        stopped(CS2TL_STOP_RANGE, 0);
        return;
    }
    const auto intervals = query_precise_intervals_into(scene, ray, workspace);
    float damage = weapon.base_damage;
    double previous_exit = 0;
    uint32_t penetrations = 0;
    for (size_t first = 0; first < intervals.size();) {
        const auto &begin = intervals[first];
        require(begin.flags == 0, CS2TL_INCOMPLETE_GEOMETRY,
                "A or B lies inside a solid; a complete penetration boundary is unavailable.");
        auto entry = materials.get(begin.entry_surface_id);
        auto exit = materials.get(begin.exit_surface_id);
        float minimum_penetration = std::min(entry.penetration, exit.penetration);
        double end = begin.exit_distance;
        size_t next = first + 1;
        // Overlapping/touching volumes form one physical material span. The
        // native rule takes the minimum modifier over intermediate surfaces.
        while (next < intervals.size() && intervals[next].entry_distance <= end) {
            const auto &span = intervals[next++];
            require(span.flags == 0, CS2TL_INCOMPLETE_GEOMETRY, "Incomplete material span.");
            const auto inner_entry = materials.get(span.entry_surface_id);
            const auto inner_exit = materials.get(span.exit_surface_id);
            minimum_penetration =
                std::min({minimum_penetration, inner_entry.penetration, inner_exit.penetration});
            if (span.exit_distance > end) {
                end = span.exit_distance;
                exit = inner_exit;
            } else if (span.exit_distance == end) {
                require(exit.material == inner_exit.material && exit.flags == inner_exit.flags,
                        CS2TL_INCOMPLETE_GEOMETRY,
                        "Coincident exits disagree about material/interaction flags.");
            }
        }
        for (size_t i = first + 1; i < next && intervals[i].entry_distance == begin.entry_distance; ++i) {
            const auto other = materials.get(intervals[i].entry_surface_id);
            require(entry.material == other.material && entry.flags == other.flags, CS2TL_INCOMPLETE_GEOMETRY,
                    "Coincident entries disagree about material/interaction flags.");
        }
        damage = attenuate(damage, static_cast<float>(begin.entry_distance - previous_exit),
                           weapon.range_modifier);
        if (penetrations >= game_penetrations) {
            stopped(CS2TL_STOP_PENETRATION_COUNT, penetrations);
            return;
        }
        if (end > max_penetration_distance) {
            stopped(CS2TL_STOP_PENETRATION_DISTANCE, penetrations);
            return;
        }
        if (weapon.penetration_power == 0) {
            stopped(CS2TL_STOP_PENETRATION_POWER, penetrations);
            return;
        }
        if (minimum_penetration < minimum_material_penetration) {
            stopped(CS2TL_STOP_MATERIAL, penetrations);
            return;
        }
        const float thickness = static_cast<float>(end - begin.entry_distance);
        float combined = minimum_penetration, damage_fraction = 0.16f;
        if (entry.material == exit.material) {
            if (entry.material == 'W' || entry.material == 'U') {
                combined = 3;
            } else if (entry.material == 'L') {
                combined = 2;
            }
            if (thickness < 6 && (entry.material == 'G' || entry.material == 'Y')) {
                combined = 3;
                damage_fraction = 0.05f;
            }
            if ((entry.flags & exit.flags & CS2TL_MATERIAL_NATIVE_FLAG13) != 0) {
                combined = thickness < 6 ? 32.0f : 3.0f;
                if (thickness < 6) {
                    damage_fraction = 0.00001f;
                }
            }
        }
        const float inverse = std::max(0.0f, 1.0f / combined);
        const float power_cost = std::max(0.0f, (3.0f / weapon.penetration_power) * 1.25f);
        const float fixed_loss = power_cost * (inverse * 3.0f) + damage * damage_fraction;
        const float loss = std::max(0.0f, (thickness * thickness * inverse) / 24.0f + fixed_loss);
        damage -= loss;
        if (damage < 1) {
            stopped(CS2TL_STOP_DAMAGE, penetrations);
            return;
        }
        ++penetrations;
        previous_exit = end;
        first = next;
    }
    damage = attenuate(damage, static_cast<float>(ray.length - previous_exit), weapon.range_modifier);
    out.outcome = CS2TL_SHOT_REACHED;
    out.remaining_damage = damage;
    out.penetration_count = penetrations;
    out.stop_reason = CS2TL_STOP_NONE;
}
void simulate_shot(const Scene &scene, const cs2tl_shot_request &request, cs2tl_shot_result &result,
                   QueryWorkspace &workspace) {
    reset_shot_result(result);
    require(request.segment.struct_size >= sizeof(cs2tl_segment) && request.weapon && request.reserved == 0 &&
                (!request.surface_count || request.surfaces),
            CS2TL_INVALID_ARGUMENT, "Invalid explicit-profile shot request.");
    require(request.ruleset_version == 0 || request.ruleset_version == CS2TL_RULESET_CS2_STATIC_V1,
            CS2TL_NOT_IMPLEMENTED, "Requested damage ruleset is not implemented.");
    require(request.target == nullptr, CS2TL_NOT_IMPLEMENTED,
            "Player health damage is not computed; use a NULL target for remaining bullet damage.");
    for (uint32_t i = 0; i < request.surface_count; ++i) {
        const auto &m = request.surfaces[i];
        require(m.struct_size >= sizeof(m) && m.surface_id >= 0 && std::isfinite(m.penetration_modifier) &&
                    m.penetration_modifier >= 0 && std::isfinite(m.damage_modifier) &&
                    m.damage_modifier >= 0 && (m.flags & ~0x1ffffu) == 0,
                CS2TL_INVALID_ARGUMENT, "Invalid explicit material profile.");
    }
    cs2tl_shot_prediction computed{};
    computed.struct_size = sizeof(computed);
    simulate_remaining_damage(scene, make_ray(request.segment), *request.weapon,
                              {{}, {request.surfaces, request.surface_count}}, workspace, computed);
    result.outcome = computed.outcome;
    result.remaining_damage = computed.remaining_damage;
    result.penetration_count = computed.penetration_count;
    result.stop_reason = computed.stop_reason;
    result.ruleset_version = computed.ruleset_version;
}
} // namespace cs2::traceline
