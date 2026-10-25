#include "TargetDamage.h"
#include "ShotSimulator.h"
#include "diagnostics/Error.h"
#include "profiles/Ruleset.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace cs2::traceline {
cs2tl_damage_rules default_damage_rules() noexcept {
    return {sizeof(cs2tl_damage_rules), 0, audited_steam_build, 1, 1, 1, 1, 0.1f, 0, 0, 0};
}
void reset_target_damage(cs2tl_target_damage_result &out) noexcept {
    const auto size = out.struct_size;
    out = {};
    out.struct_size = size;
    out.incoming_damage = out.scaled_damage = out.health_damage = out.armor_damage =
        out.damage_accumulator_after = std::numeric_limits<float>::quiet_NaN();
    out.armor_loss = out.health_loss_min = out.health_loss_max = UINT32_MAX;
    out.armor_after = out.health_after_min = out.health_after_max = -1;
    out.ruleset_version = CS2TL_RULESET_CS2_STATIC_V1;
}
void reset_shot_damage(cs2tl_shot_damage_result &out) noexcept {
    const auto size = out.struct_size;
    out = {};
    out.struct_size = size;
    out.flight.struct_size = sizeof(out.flight);
    reset_prediction(out.flight);
    out.target.struct_size = sizeof(out.target);
    reset_target_damage(out.target);
}
void validate_target_damage(const WeaponProfile &weapon, uint32_t attacker_team,
                            const cs2tl_target_state &target, const cs2tl_damage_rules &rules) {
    require(weapon.struct_size >= sizeof(weapon) && std::isfinite(weapon.armor_ratio) &&
                weapon.armor_ratio >= 0 && std::isfinite(weapon.headshot_multiplier) &&
                weapon.headshot_multiplier >= 0,
            CS2TL_INVALID_ARGUMENT, "Invalid target-damage weapon parameters.");
    require(target.struct_size >= sizeof(target) && target.health >= 0 && target.armor >= 0 &&
                (target.flags &
                 ~(CS2TL_TARGET_HAS_HELMET | CS2TL_TARGET_IMMUNE | CS2TL_TARGET_ACCUMULATOR_KNOWN)) == 0 &&
                target.reserved == 0 && target.hit_group != CS2TL_INVALID_ID,
            CS2TL_INVALID_ARGUMENT, "Invalid target state.");
    require(attacker_team <= UINT8_MAX && target.team <= UINT8_MAX, CS2TL_INVALID_ARGUMENT,
            "Team identifier exceeds native uint8 representation.");
    if (target.flags & CS2TL_TARGET_ACCUMULATOR_KNOWN) {
        require(std::isfinite(target.damage_accumulator) && target.damage_accumulator >= 0 &&
                    target.damage_accumulator < 1,
                CS2TL_INVALID_ARGUMENT, "Known damage accumulator must be finite and in [0,1).");
    }
    require(rules.struct_size >= sizeof(rules) && (rules.flags & ~CS2TL_DAMAGE_RULES_LIVE_SNAPSHOT) == 0 &&
                rules.friendly_fire <= 1 && rules.teammates_are_enemies <= 1 && rules.headshots_only <= 1,
            CS2TL_INVALID_ARGUMENT, "Invalid damage rules snapshot.");
    for (const auto value : {rules.ct_head_scale, rules.ct_body_scale, rules.t_head_scale, rules.t_body_scale,
                             rules.friendly_bullet_scale}) {
        require(std::isfinite(value) && value >= 0, CS2TL_INVALID_ARGUMENT,
                "Damage scalers must be finite and nonnegative.");
    }
}
namespace {
float finite(float value) {
    require(std::isfinite(value) && value >= 0, CS2TL_INVALID_ARGUMENT,
            "Target damage arithmetic cannot be represented as finite nonnegative float32.");
    return value;
}
struct HealthAccounting {
    uint32_t loss;
    float accumulator;
};
HealthAccounting account_health(float damage, float accumulator, int32_t health) {
    // Native RVA 0x3f40bc..0x3f4100 keeps fractional damage separately and
    // carries one point at >=1. Preserve float32 operation order.
    const float floor = std::floor(damage);
    float fraction = damage - floor;
    const float whole = damage - fraction;
    fraction += accumulator;
    float loss = whole;
    if (fraction >= 1) {
        loss += 1;
        fraction -= 1;
    }
    // Compare in double before integer conversion; avoid cvttss2si overflow
    // for valid large custom damage values. Actual HP loss cannot exceed HP.
    return {loss >= static_cast<double>(health) ? static_cast<uint32_t>(health) : static_cast<uint32_t>(loss),
            fraction};
}
bool armored(uint32_t group, const cs2tl_target_state &target) {
    if (!target.armor) {
        return false;
    }
    if (group == CS2TL_HITGROUP_HEAD) {
        return (target.flags & CS2TL_TARGET_HAS_HELMET) != 0;
    }
    return group == CS2TL_HITGROUP_GENERIC ||
           (group >= CS2TL_HITGROUP_CHEST && group <= CS2TL_HITGROUP_RIGHT_ARM) ||
           group == CS2TL_HITGROUP_NECK;
}
} // namespace
void calculate_target_damage(const WeaponProfile &weapon, float incoming, uint32_t attacker_team,
                             const cs2tl_target_state &target, const cs2tl_damage_rules &rules,
                             cs2tl_target_damage_result &out) {
    reset_target_damage(out);
    validate_target_damage(weapon, attacker_team, target, rules);
    finite(incoming);
    out.incoming_damage = incoming;
    out.scaled_damage = out.health_damage = out.armor_damage = 0;
    out.armor_loss = out.health_loss_min = out.health_loss_max = 0;
    out.armor_after = target.armor;
    out.health_after_min = out.health_after_max = target.health;
    out.flags = CS2TL_PREDICTION_NOT_LIVE_VALIDATED;
    if (!(rules.flags & CS2TL_DAMAGE_RULES_LIVE_SNAPSHOT)) {
        out.flags |= CS2TL_PREDICTION_ASSUMED_DAMAGE_RULES;
    }
    const bool known = (target.flags & CS2TL_TARGET_ACCUMULATOR_KNOWN) != 0;
    if (known) {
        out.damage_accumulator_after = target.damage_accumulator;
    } else {
        out.flags |= CS2TL_PREDICTION_UNKNOWN_DAMAGE_ACCUMULATOR;
    }
    auto blocked = [&](uint32_t reason) {
        out.outcome = CS2TL_DAMAGE_BLOCKED;
        out.block_reason = reason;
    };
    if (!target.health) {
        blocked(CS2TL_DAMAGE_BLOCK_DEAD);
        return;
    }
    if (target.flags & CS2TL_TARGET_IMMUNE) {
        blocked(CS2TL_DAMAGE_BLOCK_IMMUNE);
        return;
    }
    const bool friendly = attacker_team == target.team && !rules.teammates_are_enemies;
    if (friendly && !rules.friendly_fire) {
        blocked(CS2TL_DAMAGE_BLOCK_FRIENDLY_FIRE);
        return;
    }
    if (rules.headshots_only && target.hit_group != CS2TL_HITGROUP_HEAD) {
        blocked(CS2TL_DAMAGE_BLOCK_HEADSHOTS_ONLY);
        return;
    }
    float damage = incoming;
    bool ignore_armor = false;
    if (friendly) {
        damage = finite(damage * rules.friendly_bullet_scale);
        // The native low-friendly-fire branch forces one base point and sets
        // IGNORE_ARMOR (RVA 0xa8e980..0xa8e9a7).
        if (rules.friendly_bullet_scale > 0 && rules.friendly_bullet_scale < 0.02f &&
            incoming / rules.friendly_bullet_scale >= 1) {
            damage = 1;
            ignore_armor = true;
        }
    }
    const bool ct = target.team == CS2TL_TEAM_CT;
    const float head = ct ? rules.ct_head_scale : rules.t_head_scale;
    const float body = ct ? rules.ct_body_scale : rules.t_body_scale;
    switch (target.hit_group) {
    case CS2TL_HITGROUP_HEAD:
        damage = finite(finite(damage * weapon.headshot_multiplier) * head);
        break;
    case CS2TL_HITGROUP_STOMACH:
        damage = finite(finite(static_cast<float>(static_cast<double>(damage) * 1.25)) * body);
        break;
    case CS2TL_HITGROUP_LEFT_LEG:
    case CS2TL_HITGROUP_RIGHT_LEG:
        damage = finite(finite(static_cast<float>(static_cast<double>(damage) * .75)) * body);
        break;
    case CS2TL_HITGROUP_CHEST:
    case CS2TL_HITGROUP_LEFT_ARM:
    case CS2TL_HITGROUP_RIGHT_ARM:
    case CS2TL_HITGROUP_NECK:
        damage = finite(damage * body);
        break;
    default:
        break; // Native generic/gear/special/default path applies no body scaler.
    }
    out.scaled_damage = damage;
    if (!ignore_armor && armored(target.hit_group, target)) {
        const float ratio = weapon.armor_ratio * .5f;
        float health_damage = finite(damage * ratio);
        float cost = (damage - health_damage) * .5f;
        const float armor = static_cast<float>(target.armor);
        if (cost > armor) {
            health_damage = damage - armor / .5f;
            cost = armor;
            out.armor_after = 0;
        } else {
            if (cost < 0) {
                cost = 1; // Native unusual-ratio branch, not an import cap.
            }
            const auto loss = cost >= static_cast<double>(target.armor) ? static_cast<uint32_t>(target.armor)
                                                                        : static_cast<uint32_t>(cost);
            out.armor_after = target.armor - static_cast<int32_t>(loss);
        }
        out.armor_damage = cost;
        out.armor_loss = static_cast<uint32_t>(target.armor - out.armor_after);
        damage = finite(health_damage);
    }
    out.health_damage = damage;
    const auto low = account_health(damage, known ? target.damage_accumulator : 0, target.health);
    constexpr float largest_accumulator = 0x1.fffffep-1f; // Largest float32 below 1.
    const auto high = known ? low : account_health(damage, largest_accumulator, target.health);
    out.health_loss_min = low.loss;
    out.health_loss_max = high.loss;
    out.health_after_min = target.health - static_cast<int32_t>(high.loss);
    out.health_after_max = target.health - static_cast<int32_t>(low.loss);
    if (known) {
        out.damage_accumulator_after = low.accumulator;
    }
    out.lethal = out.health_after_max == 0;
    out.possible_lethal = out.health_after_min == 0 && out.health_after_max != 0;
    out.outcome = CS2TL_DAMAGE_EVALUATED;
}
} // namespace cs2::traceline
