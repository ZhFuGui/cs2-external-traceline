#include "TraceQuery.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <array>
#include <limits>

namespace cs2::traceline {
namespace {
bool shared_feature(const Triangle &a, const Triangle &b) {
    for (const auto p : std::array{a.a, a.b, a.c}) {
        for (const auto q : std::array{b.a, b.b, b.c}) {
            if (p.x == q.x && p.y == q.y && p.z == q.z) {
                return true;
            }
        }
    }
    return false;
}
std::vector<Hit> collect(const Scene &scene, const Ray &ray, double low, double high) {
    std::vector<Hit> hits;
    traverse(scene, ray, low, high, [&](uint32_t index) {
        double distance{};
        if (intersects_triangle(scene.triangles[index], ray, distance) && distance > ray.low &&
            distance < ray.high && distance >= low && distance <= high) {
            hits.push_back({distance, index, 0});
        }
        return true;
    });
    return hits;
}
std::vector<Hit> consolidate(const Scene &scene, const Ray &ray, std::vector<Hit> hits) {
    std::sort(hits.begin(), hits.end(), [&](const Hit &a, const Hit &b) {
        const auto sa = scene.triangles[a.triangle].solid, sb = scene.triangles[b.triangle].solid;
        if (sa != sb) {
            return sa < sb;
        }
        return a.distance != b.distance ? a.distance < b.distance : a.triangle < b.triangle;
    });
    std::vector<Hit> result;
    const double scale =
        std::max({ray.length, std::abs(ray.origin.x), std::abs(ray.origin.y), std::abs(ray.origin.z)});
    const double coincidence =
        std::min(ray.merge_epsilon, 64 * std::numeric_limits<double>::epsilon() * scale);
    for (size_t first = 0; first < hits.size();) {
        const auto solid = scene.triangles[hits[first].triangle].solid;
        Hit hit = hits[first];
        size_t next = first;
        bool enter = false, exit = false;
        while (next < hits.size() && scene.triangles[hits[next].triangle].solid == solid &&
               hits[next].distance - hit.distance <= coincidence &&
               shared_feature(scene.triangles[hit.triangle], scene.triangles[hits[next].triangle])) {
            const auto &triangle = scene.triangles[hits[next].triangle];
            const double facing = triangle.normal().dot(ray.direction);
            enter |= facing < -1e-12;
            exit |= facing > 1e-12;
            // A deterministic representative preserves one real surface identifier.
            if (hits[next].triangle < hit.triangle) {
                hit.triangle = hits[next].triangle;
            }
            ++next;
        }
        hit.kind = enter && !exit ? CS2TL_HIT_ENTER : (exit && !enter ? CS2TL_HIT_EXIT : CS2TL_HIT_TANGENT);
        if (enter && exit && scene.solids[solid].closed) {
            double delta = std::max({ray.merge_epsilon * 2, ray.length * 1e-10, 1e-9});
            if (first > 0 && scene.triangles[hits[first - 1].triangle].solid == solid) {
                delta = std::min(delta, (hit.distance - hits[first - 1].distance) * 0.25);
            }
            if (next < hits.size() && scene.triangles[hits[next].triangle].solid == solid) {
                delta = std::min(delta, (hits[next].distance - hit.distance) * 0.25);
            }
            const bool before = contains(scene, scene.solids[solid], ray.at(hit.distance - delta));
            const bool after = contains(scene, scene.solids[solid], ray.at(hit.distance + delta));
            if (before != after) {
                hit.kind = after ? CS2TL_HIT_ENTER : CS2TL_HIT_EXIT;
            }
        }
        result.push_back(hit);
        first = next;
    }
    std::sort(result.begin(), result.end(), [&](const Hit &a, const Hit &b) {
        if (a.distance != b.distance) {
            return a.distance < b.distance;
        }
        const auto sa = scene.triangles[a.triangle].solid, sb = scene.triangles[b.triangle].solid;
        return sa != sb ? sa < sb : a.triangle < b.triangle;
    });
    return result;
}
} // namespace
std::vector<Hit> trace_all(const Scene &scene, const Ray &ray) {
    return consolidate(scene, ray, collect(scene, ray, ray.low, ray.high));
}
std::optional<Hit> trace_closest(const Scene &scene, const Ray &ray) {
    double best = ray.high;
    bool found = false;
    traverse(scene, ray, ray.low, best, [&](uint32_t index) {
        double distance{};
        if (intersects_triangle(scene.triangles[index], ray, distance) && distance > ray.low &&
            distance < ray.high && distance <= best) {
            best = distance;
            found = true;
        }
        return true;
    });
    if (!found) {
        return {};
    }
    // Consolidate every surface in the first distance cluster, even when a
    // nearest-first traversal visited another adjacent triangle first.
    auto hits = consolidate(scene, ray,
                            collect(scene, ray, std::max(ray.low, best - ray.merge_epsilon),
                                    std::min(ray.high, best + ray.merge_epsilon)));
    return hits.empty() ? std::optional<Hit>{} : hits.front();
}
bool is_occluded(const Scene &scene, const Ray &ray) {
    bool hit = false;
    double high = ray.high;
    traverse(scene, ray, ray.low, high, [&](uint32_t index) {
        double distance{};
        if (intersects_triangle(scene.triangles[index], ray, distance) && distance > ray.low &&
            distance < ray.high) {
            hit = true;
            return false;
        }
        return true;
    });
    if (hit) {
        return true;
    }
    const Vec3 midpoint = ray.at((ray.low + ray.high) * 0.5);
    std::vector<const Solid *> inside;
    for (const auto &solid : scene.solids) {
        if (contains(scene, solid, midpoint)) {
            inside.push_back(&solid);
        }
    }
    for (size_t i = 0; i < inside.size(); ++i) {
        for (size_t j = i + 1; j < inside.size(); ++j) {
            require(inside[i]->object != inside[j]->object ||
                        (!inside[i]->bounds.contains(inside[j]->bounds) &&
                         !inside[j]->bounds.contains(inside[i]->bounds)),
                    CS2TL_INCOMPLETE_GEOMETRY,
                    "Nested shells of one source object have ambiguous interior occupancy.");
        }
    }
    return !inside.empty();
}
cs2tl_hit external_hit(const Scene &scene, const Ray &ray, const Hit &hit) {
    const auto &tri = scene.triangles[hit.triangle];
    return {static_cast<float>(hit.distance),
            static_cast<float>(hit.distance / ray.length),
            ray.at(hit.distance).external(),
            tri.normal().external(),
            tri.surface,
            tri.object,
            tri.solid,
            tri.id,
            hit.kind};
}
} // namespace cs2::traceline
