#pragma once
#include "cs2-external-traceline/types.h"
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace cs2::traceline {
struct ResolvedMaterial {
    float penetration{}, damage{};
    uint32_t material{}, flags{};
    bool valid{};
};
struct WeaponEntry {
    cs2tl_weapon_profile profile{};
    std::string name;
    std::string ammo_name;
};
struct MaterialEntry {
    cs2tl_material_profile profile{};
    std::string name;
};
struct BallisticsProfiles {
    cs2tl_profiles_info info{};
    std::vector<WeaponEntry> weapons;
    std::vector<MaterialEntry> materials;
    std::unordered_map<uint64_t, size_t> weapon_indices;
    std::unordered_map<uint32_t, size_t> material_indices;
    const WeaponEntry &weapon(uint32_t id, uint32_t mode) const;
    const MaterialEntry &material(uint32_t hash) const;
};
BallisticsProfiles load_profiles(const std::filesystem::path &path);
} // namespace cs2::traceline
