#pragma once
#include "NativeCollisionFilter.h"
#include "scene/PhysicsData.h"

namespace cs2::traceline {
// Build 26144732: registered names/aliases -> resource attributes -> static
// shape defaults. Instance/body overrides and nonempty detail are separate.
NativeShapeFilter compile_native_attribute(const PhysicsAttribute &source, bool apply_static_shape = true);
// Complete default matrix after the audited CS2 initialization overrides.
std::span<const uint16_t, 4096> native_collision_rules() noexcept;
} // namespace cs2::traceline
