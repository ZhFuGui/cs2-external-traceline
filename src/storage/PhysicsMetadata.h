#pragma once
#include "scene/PhysicsData.h"
#include <span>

namespace cs2::traceline {
std::vector<uint8_t> encode_physics_metadata(const PhysicsData &physics);
PhysicsData decode_physics_metadata(std::span<const uint8_t> bytes);
void validate_physics_metadata(const PhysicsData &physics);
} // namespace cs2::traceline
