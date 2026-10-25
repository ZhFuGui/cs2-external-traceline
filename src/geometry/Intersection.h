#pragma once
#include "scene/Scene.h"

namespace cs2::traceline {
bool intersects_triangle(const Triangle &tri, const Ray &ray, double &distance,
                         double *barycentric_margin = nullptr);
bool intersects_triangle(const TriangleQuery &tri, const Ray &ray, double &distance,
                         double *barycentric_margin = nullptr);
bool point_on_triangle(Vec3 point, const Triangle &tri, double epsilon);
bool contains(const Scene &scene, const Solid &solid, Vec3 point);
Ray make_ray(const cs2tl_segment &segment);
} // namespace cs2::traceline
