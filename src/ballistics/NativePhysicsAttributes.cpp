#include "NativePhysicsAttributes.h"
#include "diagnostics/Error.h"

namespace cs2::traceline {
namespace {
struct IndexedHash {
    uint32_t hash;
    uint8_t index;
};
// vphysics2!296fa0 followed by server!940f40. These are independent registries.
constexpr IndexedHash interactions[] = {
    {0xcc9643e1u, 0},  // solid
    {0x78cabd8cu, 1},  // hitboxes
    {0x41bc2b3du, 2},  // trigger
    {0xf6c4aab0u, 3},  // sky
    {0xe8abec43u, 4},  // playerclip
    {0x1117afabu, 5},  // npcclip
    {0x5a49ad0au, 6},  // blocklos
    {0x4e9e93e0u, 7},  // blocklight
    {0xe2a5b837u, 8},  // ladder
    {0x43bfca3du, 9},  // pickup
    {0x7034929cu, 10}, // blocksound
    {0x8782b69bu, 11}, // nodraw
    {0x1b4c4bfcu, 12}, // window (alias takes precedence below)
    {0x6575c41cu, 15}, // water
    {0x9a594106u, 16}, // slime
    {0xcbe8b56bu, 13}, // passbullets
    {0x21c2be47u, 14}, // worldGeometry
    {0xb8ee3e91u, 17}, // touchall
    {0x8521b8b9u, 18}, // player
    {0x6e541570u, 19}, // npc
    {0x5d9e579cu, 20}, // debris
    {0x264a6736u, 21}, // physics_prop
    {0x82da3bdfu, 26}, // CarriedObject
    {0xcfe4d4afu, 27}, // pushaway
    {0xfc00935eu, 28}, // serverentityonclient
    {0x3398dae4u, 29}, // CarriedWeapon
    {0x0a7324e1u, 30}, // StaticLevel
    {0x6510a9a1u, 22}, // NavIgnore
    {0x1d8d784au, 23}, // NavLocalIgnore
    {0x95cf1768u, 24}, // PostProcessingVolume
    {0x156ce901u, 25}, // vehicleclip
    {0x214e1e2au, 31}, // csgo_team1
    {0xf7ce9ed4u, 32}, // csgo_team2
    {0x95fc8ab0u, 33}, // csgo_grenadeclip
    {0x2cc56f2bu, 34}, // csgo_droneclip
    {0x86788891u, 35}, // csgo_moveable
    {0xcf393d07u, 36}, // csgo_opaque
    {0x7a864d76u, 37}, // csgo_monster
    {0x80e86906u, 38}, // csgo_chicken
    {0xe18026c9u, 39}, // csgo_thrown_grenade
    {0x560758e6u, 40}  // csgo_dropped_hostage
};
constexpr IndexedHash groups[] = {
    {0x93d08c51u, 0},  // always
    {0x9245a66eu, 1},  // never
    {0x41bc2b3du, 2},  // trigger
    {0xfdd4dfdbu, 3},  // ConditionallySolid
    {0x75de364eu, 4},  // Default
    {0x5d9e579cu, 5},  // debris
    {0x5d43a832u, 6},  // interactive_debris
    {0x351f6880u, 7},  // interactive
    {0x8521b8b9u, 8},  // Player
    {0x95d1761bu, 9},  // breakable_glass
    {0xc5513736u, 10}, // vehicle
    {0xc13fe2eeu, 11}, // player_movement
    {0x6e541570u, 12}, // npc
    {0x40ec3c6fu, 13}, // in_vehicle
    {0xebde2b13u, 14}, // weapon
    {0xd7e8b8d5u, 16}, // Projectile
    {0xd219d3b8u, 17}, // door_blocker
    {0xdc3555c2u, 18}, // passable_door
    {0x5c6530eau, 19}, // dissolving
    {0xcfe4d4afu, 20}, // pushaway
    {0x971fbfa4u, 21}, // npc_actor
    {0xd3f4c5ffu, 22}, // npc_scripted
    {0x459c6515u, 23}, // pz_clip
    {0x3a656ed3u, 24}  // props
};
uint8_t group_index(uint32_t hash) {
    for (const auto &entry : groups) {
        if (entry.hash == hash) {
            return entry.index;
        }
    }
    throw Error(CS2TL_DATA_NOT_FOUND, "Physics collision group is absent from the audited registry.");
}
uint64_t interaction(uint32_t hash) {
    // 294110 resolves compound aliases before atomic names, including window.
    switch (hash) {
    case 4166480373u:
        return 0x4c1; // CONTENTS_SOLID
    case 1291942539u:
        return 0x481; // CONTENTS_SOLID_NO_BLOCK_LOS
    case 457985020u:
        return 0x1400; // window | blocksound
    default:
        break;
    }
    for (const auto &entry : interactions) {
        if (entry.hash == hash) {
            return uint64_t{1} << entry.index;
        }
    }
    throw Error(CS2TL_DATA_NOT_FOUND, "Physics interaction tag is absent from the audited registry.");
}
uint64_t compile_tags(std::span<const uint32_t> tags) {
    uint64_t mask = 0;
    for (const auto hash : tags) {
        mask |= interaction(hash);
    }
    return mask;
}
struct MatrixRow {
    uint64_t colliding, responding;
};
// 2d7990/2d79a0 initialization + server!941140 overrides. Each column is
// either 0, 0x14 or 0x15. Groups 25..63 only respond to the always group.
constexpr MatrixRow rows[] = {
    {0xfffffffffffffffbull, 0x0000000000000004ull}, {0x0000000000000001ull, 0x0000000000000000ull},
    {0x0000000000000000ull, 0x0000000001fffff9ull}, {0x0000000001fffff9ull, 0x0000000000000004ull},
    {0x00000000017ddff9ull, 0x0000000000000004ull}, {0x0000000001100019ull, 0x0000000000000004ull},
    {0x000000000175d619ull, 0x0000000000000004ull}, {0x000000000175df99ull, 0x0000000000000004ull},
    {0x0000000001e19f99ull, 0x0000000000004004ull}, {0x000000000175ddd9ull, 0x0000000000002004ull},
    {0x0000000001759fd9ull, 0x0000000000004004ull}, {0x0000000001e59f99ull, 0x0000000000000004ull},
    {0x0000000001379fd9ull, 0x0000000000004004ull}, {0x0000000001010009ull, 0x0000000000004204ull},
    {0x00000000011482d9ull, 0x0000000000603504ull}, {0x0000000001755fd9ull, 0x0000000000000004ull},
    {0x000000000174bfd9ull, 0x0000000000000004ull}, {0x0000000000701009ull, 0x0000000000000004ull},
    {0x000000000175ded9ull, 0x0000000000000004ull}, {0x0000000001000019ull, 0x0000000000000004ull},
    {0x000000000177d6f9ull, 0x0000000000000004ull}, {0x0000000001779fd9ull, 0x0000000000004004ull},
    {0x0000000001378fd9ull, 0x0000000000004004ull}, {0x0000000000000909ull, 0x0000000000000004ull},
    {0x00000000017dfff9ull, 0x0000000000000004ull}};
constexpr auto make_rules() {
    std::array<uint16_t, 4096> result{};
    for (size_t query = 0; query < 64; ++query) {
        const auto row = query < std::size(rows) ? rows[query] : MatrixRow{1, 0};
        for (size_t shape = 0; shape < 64; ++shape) {
            const auto bit = uint64_t{1} << shape;
            result[query * 64 + shape] = (row.colliding & bit) ? 0x15u : (row.responding & bit) ? 0x14u : 0u;
        }
    }
    return result;
}
constexpr auto rules = make_rules();
} // namespace
NativeShapeFilter compile_native_attribute(const PhysicsAttribute &source, bool apply_static_shape) {
    require((source.flags & physics_attribute_complete) != 0, CS2TL_INCOMPLETE_GEOMETRY,
            "Physics attributes lack complete numeric source tags.");
    require(source.detail_layers.empty() && source.include_detail_count == 0, CS2TL_INCOMPLETE_GEOMETRY,
            "Native physics detail-layer compilation has not been audited for this source.");
    NativeShapeFilter value;
    value.group = group_index(source.group_hash);
    value.interact_as = compile_tags(source.interact_as);
    value.interact_with = compile_tags(source.interact_with);
    value.exclude = compile_tags(source.exclude);
    value.id = 0;
    value.owner_id = UINT32_MAX;
    value.type_mask = 0;
    value.type_mask_inclusive = 0;
    value.flags = 7;
    if (apply_static_shape && value.group == 4 && value.interact_as == 0) {
        value.interact_as = 0x4c1;
    }
    return value;
}
std::span<const uint16_t, 4096> native_collision_rules() noexcept {
    return rules;
}
} // namespace cs2::traceline
