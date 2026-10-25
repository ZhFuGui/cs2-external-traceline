#pragma once
#include "geometry/Intersection.h"
#include <optional>

namespace cs2::traceline {
struct Hit {
    double distance{};
    uint32_t triangle{}, kind{};
};
std::vector<Hit> trace_all(const Scene &scene, const Ray &ray);
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray);
bool is_occluded(const Scene &scene, const Ray &ray);
cs2tl_hit external_hit(const Scene &scene, const Ray &ray, const Hit &hit);
std::vector<cs2tl_interval> query_intervals(const Scene &scene, const Ray &ray);
} // namespace cs2::traceline
