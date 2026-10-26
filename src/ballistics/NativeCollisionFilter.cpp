#include "NativeCollisionFilter.h"
#include "diagnostics/Error.h"

namespace cs2::traceline {
namespace {
bool matches_group(const NativeQueryFilter &query, uint16_t group, bool exact) noexcept {
    if (group == 0 || group == UINT16_MAX || (query.group_filter[0] == 0 && query.group_filter[1] == 0)) {
        return false;
    }
    return group == query.group_filter[0] || group == query.group_filter[1] || !exact;
}
// BT with a 32-bit register masks its bit index to five bits. The source word
// is zero-extended, so type indices 16..31 deliberately see zero bits.
bool type_in_mask(uint16_t mask, uint8_t type) noexcept {
    return (uint32_t(mask) & (1u << (type & 31u))) != 0;
}
} // namespace
bool accept_native_collision(std::span<const uint16_t> group_table, const NativeQueryFilter &query,
                             const NativeShapeFilter &shape, CollisionPairExcluded excluded, void *user) {
    if ((query.flags & 0x20u) != 0) {
        return true;
    }
    if (shape.id != UINT32_MAX && (shape.id == query.excluded_ids[0] || shape.id == query.excluded_ids[1])) {
        return false;
    }
    if (((query.exclude & shape.interact_as) | (shape.exclude & query.interact_with)) != 0 ||
        (shape.flags & 2u) == 0 || !type_in_mask(query.type_mask, shape.type)) {
        return false;
    }
    const auto type_mask = shape.type_mask_inclusive == 1 ? shape.type_mask : uint16_t(~shape.type_mask);
    if (!type_in_mask(type_mask, query.type)) {
        return false;
    }
    if (shape.group == 0 || query.group == 0) {
        return true;
    }
    if (((shape.interact_as & query.include) | (shape.interact_with & query.interact_with)) == 0 ||
        matches_group(query, shape.group_filter, (shape.flags & 0x80u) == 0)) {
        return false;
    }
    if (shape.id != UINT32_MAX && (shape.id == query.excluded_ids[2] || shape.id == query.excluded_ids[3])) {
        return false;
    }
    if ((query.excluded_ids[0] != UINT32_MAX && query.excluded_ids[0] == shape.owner_id) ||
        (query.excluded_ids[1] != UINT32_MAX && query.excluded_ids[1] == shape.owner_id)) {
        return false;
    }
    const auto index = size_t(query.group) * 64 + shape.group;
    require(index < group_table.size(), CS2TL_INVALID_GEOMETRY,
            "Native collision group has no backing matrix entry.");
    auto interaction = group_table[index];
    bool collides = (interaction & 1u) != 0;
    if ((query.flags & 2u) != 0 && (shape.flags & 1u) == 0 && collides) {
        interaction = 0;
        collides = false;
    }
    if ((query.flags & 8u) != 0 && excluded &&
        ((query.excluded_ids[0] != UINT32_MAX && excluded(shape.id, query.excluded_ids[0], user)) ||
         (query.excluded_ids[1] != UINT32_MAX && excluded(shape.id, query.excluded_ids[1], user))) &&
        collides) {
        interaction = 0;
        collides = false;
    }
    if ((query.flags & 1u) != 0) {
        if (collides) {
            if ((query.include & 2u) != 0 && (shape.flags & 0x10u) != 0) {
                return false;
            }
            return (query.flags & 0x10u) == 0 || (shape.interact_with & 2u) == 0;
        }
    } else if (collides) {
        return false;
    }
    return (interaction & 0x1cu) != 0 && (query.flags & 4u) != 0;
}
} // namespace cs2::traceline
