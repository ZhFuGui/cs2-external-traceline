#pragma once
#include "scene/Scene.h"
#include <array>
#include <span>

namespace cs2::traceline {
void build_bvh(Scene &scene);
void build_query_indices(Scene &scene);
bool has_nested_inside(const Scene &scene, std::span<const uint32_t> sorted_inside);
inline constexpr uint32_t bvh_max_depth = 48;
inline constexpr size_t traversal_stack_capacity = 64;
static_assert(bvh_max_depth + 2 <= traversal_stack_capacity);
inline const Aabb &node_bounds(const BvhNode &node) {
    return node.bounds;
}
inline Aabb node_bounds(const IndexNode &node) {
    return node.bounds();
}

// Builders and cache validation enforce depth <= 48. Cached entry distances
// avoid testing a child again on pop after a nearest query lowers high.
template <class Node, class Visitor>
void traverse_nodes(std::span<const Node> nodes, std::span<const uint32_t> order, uint32_t root,
                    const Ray &ray, double low, double &high, Visitor &&visitor) {
    if (nodes.empty() || root == CS2TL_INVALID_ID) {
        return;
    }
    double entry{};
    if (!intersects(node_bounds(nodes[root]), ray, low, high, entry)) {
        return;
    }
    struct Pending {
        uint32_t node;
        double entry;
    };
    std::array<Pending, traversal_stack_capacity> stack;
    size_t size = 0;
    stack[size++] = {root, entry};
    while (size) {
        const auto pending = stack[--size];
        if (pending.entry > high) {
            continue;
        }
        const auto &node = nodes[pending.node];
        if (node.leaf()) {
            for (uint32_t i = 0; i < node.count; ++i) {
                if (!visitor(order[node.first + i])) {
                    return;
                }
            }
        } else {
            double a{}, b{};
            const bool left = intersects(node_bounds(nodes[node.left]), ray, low, high, a);
            const bool right = intersects(node_bounds(nodes[node.right]), ray, low, high, b);
            if (left && right) {
                stack[size++] = a <= b ? Pending{node.right, b} : Pending{node.left, a};
                stack[size++] = a <= b ? Pending{node.left, a} : Pending{node.right, b};
            } else if (left) {
                stack[size++] = {node.left, a};
            } else if (right) {
                stack[size++] = {node.right, b};
            }
        }
    }
}
template <class Visitor>
void traverse(const Scene &scene, const Ray &ray, double low, double &high, Visitor &&visitor) {
    traverse_nodes<BvhNode>(scene.nodes, scene.order, 0, ray, low, high, visitor);
}
template <class Predicate, class Visitor>
void visit_index(std::span<const IndexNode> nodes, std::span<const uint32_t> order, uint32_t root,
                 Predicate &&accept, Visitor &&visitor) {
    if (nodes.empty() || root == CS2TL_INVALID_ID) {
        return;
    }
    std::array<uint32_t, traversal_stack_capacity> stack;
    size_t size = 0;
    stack[size++] = root;
    while (size) {
        const auto &node = nodes[stack[--size]];
        if (!accept(node.bounds())) {
            continue;
        }
        if (node.leaf()) {
            for (uint32_t i = 0; i < node.count; ++i) {
                if (!visitor(order[node.first + i])) {
                    return;
                }
            }
        } else {
            stack[size++] = node.right;
            stack[size++] = node.left;
        }
    }
}
} // namespace cs2::traceline
