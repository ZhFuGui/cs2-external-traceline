#include "FileIO.h"
#include "diagnostics/Error.h"
#include "profiles/BallisticsProfiles.h"
#include "profiles/Ruleset.h"
#include <bit>
#include <cmath>
#include <cstring>

namespace cs2::traceline {
namespace {
class ProfileReader {
    std::span<const uint8_t> data;

  public:
    size_t position{};
    explicit ProfileReader(std::span<const uint8_t> bytes) : data(bytes) {}
    uint32_t u32() {
        require(position <= data.size() && data.size() - position >= 4, CS2TL_CORRUPT_PROFILES,
                "Truncated profile package.", position);
        uint32_t out = 0;
        for (unsigned i = 0; i < 4; ++i) {
            out |= uint32_t(data[position++]) << (8 * i);
        }
        return out;
    }
    uint64_t u64() {
        const uint64_t lo = u32();
        return lo | (uint64_t(u32()) << 32);
    }
    float f32() {
        const float out = std::bit_cast<float>(u32());
        require(std::isfinite(out), CS2TL_CORRUPT_PROFILES, "Nonfinite tuning value.", position - 4);
        return out;
    }
};
std::string name(std::span<const uint8_t> pool, uint32_t offset) {
    require(offset < pool.size(), CS2TL_CORRUPT_PROFILES, "Profile name offset exceeds the string table.");
    const auto *start = reinterpret_cast<const char *>(pool.data() + offset);
    const auto *end = static_cast<const char *>(std::memchr(start, 0, pool.size() - offset));
    require(end && end != start, CS2TL_CORRUPT_PROFILES, "Unterminated/empty profile name.");
    // Resource tuning identifiers are UTF-8. Validate codepoints without
    // retaining decoded strings or imposing a name-length policy.
    auto cursor = reinterpret_cast<const uint8_t *>(start);
    const auto *stop = reinterpret_cast<const uint8_t *>(end);
    while (cursor != stop) {
        const auto lead = *cursor++;
        if (lead < 0x80) {
            continue;
        }
        uint32_t code{};
        unsigned rest{};
        uint32_t minimum{};
        if ((lead & 0xe0) == 0xc0) {
            code = lead & 0x1f;
            rest = 1;
            minimum = 0x80;
        } else if ((lead & 0xf0) == 0xe0) {
            code = lead & 0x0f;
            rest = 2;
            minimum = 0x800;
        } else if ((lead & 0xf8) == 0xf0) {
            code = lead & 7;
            rest = 3;
            minimum = 0x10000;
        } else {
            throw Error(CS2TL_CORRUPT_PROFILES, "Invalid UTF-8 profile name.");
        }
        require(static_cast<size_t>(stop - cursor) >= rest, CS2TL_CORRUPT_PROFILES,
                "Truncated UTF-8 profile name.");
        for (unsigned i = 0; i < rest; ++i) {
            require((*cursor & 0xc0) == 0x80, CS2TL_CORRUPT_PROFILES, "Invalid UTF-8 continuation.");
            code = (code << 6) | (*cursor++ & 0x3f);
        }
        require(code >= minimum && code <= 0x10ffff && !(code >= 0xd800 && code <= 0xdfff),
                CS2TL_CORRUPT_PROFILES, "Invalid UTF-8 codepoint.");
    }
    return {start, end};
}
} // namespace
BallisticsProfiles load_profiles(const std::filesystem::path &path) {
    const auto bytes = read_file(path);
    require(bytes.size() >= 64, CS2TL_CORRUPT_PROFILES, "Truncated profile header.");
    require(std::memcmp(bytes.data(), "CS2TLPRF", 8) == 0, CS2TL_UNSUPPORTED_FORMAT,
            "Unknown profile container.");
    ProfileReader reader(bytes);
    reader.position = 8;
    const auto version = reader.u32(), header = reader.u32();
    require(version == 1 && header == 64, CS2TL_UNSUPPORTED_FORMAT, "Unsupported profile format version.");
    BallisticsProfiles result;
    auto &info = result.info;
    info.struct_size = sizeof(info);
    info.format_version = version;
    info.steam_build_id = reader.u64();
    info.client_version = reader.u32();
    info.ruleset_version = reader.u32();
    info.weapon_profile_count = reader.u32();
    info.surface_count = reader.u32();
    const auto strings = reader.u32();
    info.flags = reader.u32();
    const auto payload = reader.u64();
    info.source_fingerprint = reader.u64();
    const uint64_t records = uint64_t(info.weapon_profile_count) * 48 + uint64_t(info.surface_count) * 24;
    require(records + strings == payload && payload == bytes.size() - header, CS2TL_CORRUPT_PROFILES,
            "Invalid profile lengths/counts/version identity.");
    require(fingerprint(std::span(bytes).subspan(header)) == info.source_fingerprint, CS2TL_CORRUPT_PROFILES,
            "Profile payload checksum mismatch.");
    const auto pool = std::span(bytes).subspan(static_cast<size_t>(header + records), strings);
    result.weapons.reserve(info.weapon_profile_count);
    result.weapon_indices.reserve(info.weapon_profile_count);
    result.materials.reserve(info.surface_count);
    result.material_indices.reserve(info.surface_count);
    for (uint32_t i = 0; i < info.weapon_profile_count; ++i) {
        WeaponEntry entry;
        auto &w = entry.profile;
        w.struct_size = sizeof(w);
        w.data_version = info.client_version;
        w.weapon_id = reader.u32();
        w.ammo_id = reader.u32();
        w.fire_mode = reader.u32();
        entry.name = name(pool, reader.u32());
        w.base_damage = reader.f32();
        w.max_range = reader.f32();
        w.range_modifier = reader.f32();
        w.penetration_power = reader.f32();
        w.armor_ratio = reader.f32();
        w.headshot_multiplier = reader.f32();
        w.pellets_per_shot = reader.u32();
        entry.ammo_name = name(pool, reader.u32());
        require(w.weapon_id != CS2TL_INVALID_ID && w.base_damage > 0 && w.max_range > 0 &&
                    w.range_modifier > 0 && w.penetration_power >= 0 && w.armor_ratio >= 0 &&
                    w.headshot_multiplier >= 0 && w.pellets_per_shot > 0,
                CS2TL_CORRUPT_PROFILES, "Invalid weapon profile.");
        require(result.weapon_indices.emplace((uint64_t(w.weapon_id) << 32) | w.fire_mode, i).second,
                CS2TL_CORRUPT_PROFILES, "Duplicate weapon/firing-mode profile.");
        result.weapons.push_back(std::move(entry));
    }
    for (uint32_t i = 0; i < info.surface_count; ++i) {
        MaterialEntry entry;
        auto &m = entry.profile;
        m.struct_size = sizeof(m);
        m.data_version = info.client_version;
        m.surface_hash = reader.u32();
        entry.name = name(pool, reader.u32());
        m.game_material = reader.u32();
        m.flags = reader.u32();
        m.penetration_distance_modifier = reader.f32();
        m.penetration_damage_modifier = reader.f32();
        require(m.game_material <= UINT16_MAX && (m.flags & ~CS2TL_MATERIAL_NATIVE_FLAG13) == 0 &&
                    m.penetration_distance_modifier >= 0 && m.penetration_damage_modifier >= 0,
                CS2TL_CORRUPT_PROFILES, "Invalid material profile.");
        require(result.material_indices.emplace(m.surface_hash, i).second, CS2TL_CORRUPT_PROFILES,
                "Duplicate surface hash.");
        result.materials.push_back(std::move(entry));
    }
    info.flags |= CS2TL_PROFILES_NOT_LIVE_VALIDATED;
    if (info.steam_build_id != audited_steam_build || (info.flags & CS2TL_PROFILES_BUILD_MISMATCH) != 0) {
        info.flags |= CS2TL_PROFILES_BUILD_MISMATCH;
        warn("Profile data belongs to a different build than the binary-audited static rules; prediction is "
             "flagged.");
    }
    if (info.flags & CS2TL_PROFILES_NOT_LIVE_VALIDATED) {
        warn("Static damage rules have a binary audit, but no live-server differential validation.");
    }
    return result;
}
} // namespace cs2::traceline
