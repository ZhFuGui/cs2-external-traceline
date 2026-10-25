#pragma once
#include "scene/Scene.h"
#include <functional>

namespace cs2::traceline {
void build_bvh(Scene &scene);
// Visitor may lower high for nearest-hit pruning; returning false stops traversal.
void traverse(const Scene &scene, const Ray &ray, double low, double &high,
              const std::function<bool(uint32_t)> &visitor);
} // namespace cs2::traceline
