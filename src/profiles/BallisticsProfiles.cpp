#include "BallisticsProfiles.h"
#include "diagnostics/Error.h"

namespace cs2::traceline {
const WeaponEntry &BallisticsProfiles::weapon(uint32_t id, uint32_t mode) const {
    const auto it = weapon_indices.find((uint64_t(id) << 32) | mode);
    require(it != weapon_indices.end(), CS2TL_DATA_NOT_FOUND,
            "Weapon/firing mode is absent from the profile package.");
    return weapons[it->second];
}
const MaterialEntry &BallisticsProfiles::material(uint32_t hash) const {
    const auto it = material_indices.find(hash);
    require(it != material_indices.end(), CS2TL_DATA_NOT_FOUND,
            "Surface hash is absent from the profile package.");
    return materials[it->second];
}
} // namespace cs2::traceline
