#pragma once
#include "WeaponProfile.h"
namespace cs2::traceline {
cs2tl_damage_rules default_damage_rules() noexcept;
void reset_target_damage(cs2tl_target_damage_result &result) noexcept;
void reset_shot_damage(cs2tl_shot_damage_result &result) noexcept;
void validate_target_damage(const WeaponProfile &weapon, uint32_t attacker_team,
                            const cs2tl_target_state &target, const cs2tl_damage_rules &rules);
void calculate_target_damage(const WeaponProfile &weapon, float incoming_damage, uint32_t attacker_team,
                             const cs2tl_target_state &target, const cs2tl_damage_rules &rules,
                             cs2tl_target_damage_result &out);
} // namespace cs2::traceline
