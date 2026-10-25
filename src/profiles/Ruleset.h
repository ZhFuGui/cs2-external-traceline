#pragma once
#include <cstdint>
namespace cs2::traceline {
// Audited from the installed server.dll, Steam build 26144732. Ordinary static
// paths only; live player/destruction state is outside this model. Resource
// modifiers are loaded from the explicit profile package, never a weapon table.
inline constexpr uint64_t audited_steam_build = 26144732;
inline constexpr float range_decay_units = 500;
inline constexpr float max_penetration_distance = 3000;
// Native FireBullet callers pass 4 at RVA 0x2abd90, 0x2abf28 and 0xfa3103.
inline constexpr uint32_t game_penetrations = 4;
inline constexpr float minimum_material_penetration = 0.1f;
} // namespace cs2::traceline
