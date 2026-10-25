#pragma once
#include "scene/Scene.h"

namespace cs2::traceline {
bool intersects_triangle(const Triangle &tri, const Ray &ray, double &distance,
                         double *barycentric_margin = nullptr);
bool intersects_triangle(const TriangleQuery &tri, const Ray &ray, double &distance,
                         double *barycentric_margin = nullptr);
bool point_on_triangle(Vec3 point, const Triangle &tri, double epsilon);
bool contains(const Scene &scene, const Solid &solid, Vec3 point);
bool contains_triangles(const Scene &scene, const Solid &solid, Vec3 point);
template <bool Extended> inline bool contains_as(const Scene &scene, const Solid &solid, Vec3 point) {
    if constexpr (Extended) {
        return contains(scene, solid, point);
    } else {
        return contains_triangles(scene, solid, point);
    }
}
void prepare_primitive(Primitive &primitive);
bool contains(const Primitive &primitive, Vec3 point);
Vec3 primitive_normal(const Primitive &primitive, Vec3 point);
// At most two line intersections with one convex capsule/ellipsoid.
size_t primitive_intersections(const Primitive &primitive, const Ray &ray, std::array<double, 2> &distances);
Ray make_ray(const cs2tl_segment &segment);
} // namespace cs2::traceline
