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
// Target hitgroups: RVA 0xa8ed24..0xa8eef9; armor: 0xad7ba0 and
// 0xa8e69f..0xa8e72c; fractional HP carry: 0x3f40bc..0x3f4100.
// Native weapon-free bot_hurt (RVA 0x2f5f32) also validates generic live HP
// accounting; it does not establish full weapon/trajectory equivalence.
} // namespace cs2::traceline
