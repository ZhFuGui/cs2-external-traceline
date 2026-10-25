#include "Bvh.h"
#include <vector>

namespace cs2::traceline {
void traverse(const Scene &scene, const Ray &ray, double low, double &high,
              const std::function<bool(uint32_t)> &visitor) {
    if (scene.nodes.empty()) {
        return;
    }
    std::vector<uint32_t> stack;
    stack.reserve(64);
    stack.push_back(0);
    while (!stack.empty()) {
        const uint32_t index = stack.back();
        stack.pop_back();
        const auto &node = scene.nodes[index];
        double entry{};
        if (!intersects(node.bounds, ray, low, high, entry)) {
            continue;
        }
        if (node.leaf()) {
            for (uint32_t i = 0; i < node.count; ++i) {
                if (!visitor(scene.order[node.first + i])) {
                    return;
                }
            }
        } else {
            double a{}, b{};
            const bool hit_left = intersects(scene.nodes[node.left].bounds, ray, low, high, a);
            const bool hit_right = intersects(scene.nodes[node.right].bounds, ray, low, high, b);
            if (hit_left && hit_right) {
                stack.push_back(a <= b ? node.right : node.left);
                stack.push_back(a <= b ? node.left : node.right);
            } else if (hit_left) {
                stack.push_back(node.left);
            } else if (hit_right) {
                stack.push_back(node.right);
            }
        }
    }
}
} // namespace cs2::traceline
