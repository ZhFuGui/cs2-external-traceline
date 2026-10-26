#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace cs2::traceline {
// Compiled query/shape attributes for build 26144732. Resource hashes are not
// bit numbers: the importer must resolve them through the audited registries.
struct NativeQueryFilter {
    uint64_t include{}, exclude{}, interact_with{};
    std::array<uint32_t, 4> excluded_ids{UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    std::array<uint16_t, 2> group_filter{};
    uint16_t type_mask{UINT16_MAX};
    uint8_t type{}, reserved{}, group{}, flags{};
    std::array<uint8_t, 6> padding{};
};
struct NativeShapeFilter {
    uint64_t interact_as{}, interact_with{}, exclude{};
    uint32_t id{UINT32_MAX}, owner_id{UINT32_MAX};
    uint16_t group_filter{}, type_mask{UINT16_MAX};
    uint8_t type_mask_inclusive{1}, type{}, group{}, flags{2};
};
static_assert(sizeof(NativeQueryFilter) == 56 && offsetof(NativeQueryFilter, flags) == 0x31);
static_assert(sizeof(NativeShapeFilter) == 40 && offsetof(NativeShapeFilter, flags) == 0x27);
// Optional native pair-exclusion lookup. Static-world callers use nullptr.
using CollisionPairExcluded = bool (*)(uint32_t shape_id, uint32_t query_id, void *user) noexcept;
// vphysics2.dll RVA 0x2facf0 plus its group helper 0x2facb0. group_table is the
// compiled matrix, indexed by query.group*64 + shape.group. This operation does
// not infer that matrix or replace the separate world/body/shape query checks.
bool accept_native_collision(std::span<const uint16_t> group_table, const NativeQueryFilter &query,
                             const NativeShapeFilter &shape, CollisionPairExcluded excluded = nullptr,
                             void *user = nullptr);
} // namespace cs2::traceline
