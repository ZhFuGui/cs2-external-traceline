#include "CacheFormat.h"
#include "FileIO.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <bit>
#include <cstring>

namespace cs2::traceline {
namespace {
class Reader {
    std::span<const uint8_t> data;

  public:
    size_t position{};
    explicit Reader(std::span<const uint8_t> value) : data(value) {}
    uint32_t u32() {
        require(position + 4 <= data.size(), CS2TL_CORRUPT_CACHE, "Truncated cache.", position);
        uint32_t out = 0;
        for (unsigned i = 0; i < 4; ++i) {
            out |= uint32_t(data[position++]) << (8 * i);
        }
        return out;
    }
    uint64_t u64() {
        const uint64_t lo = u32();
        return lo | (uint64_t(u32()) << 32);
    }
    double f32() {
        const double v = std::bit_cast<float>(u32());
        require(std::isfinite(v), CS2TL_CORRUPT_CACHE, "Nonfinite cache coordinate.", position - 4);
        return v;
    }
    Vec3 vector() {
        const double x = f32(), y = f32(), z = f32();
        return {x, y, z};
    }
};
void validate_tree(const Scene &scene) {
    const auto count = static_cast<uint32_t>(scene.triangles.size());
    require(scene.order.size() == count, CS2TL_CORRUPT_CACHE, "Invalid BVH order count.");
    require((count == 0) == scene.nodes.empty(), CS2TL_CORRUPT_CACHE, "Invalid empty BVH.");
    std::vector<bool> seen_tri(count), seen_order(count), seen_node(scene.nodes.size());
    for (auto id : scene.order) {
        require(id < count && !seen_tri[id], CS2TL_CORRUPT_CACHE, "BVH order is not a triangle permutation.");
        seen_tri[id] = true;
    }
    if (!count) {
        return;
    }
    std::vector<std::pair<uint32_t, uint32_t>> pending{{0, 0}};
    while (!pending.empty()) {
        const auto [index, depth] = pending.back();
        pending.pop_back();
        require(index < scene.nodes.size() && !seen_node[index] && depth <= bvh_max_depth,
                CS2TL_CORRUPT_CACHE, "Invalid/cyclic/deep BVH node.");
        seen_node[index] = true;
        const auto &node = scene.nodes[index];
        require(node.bounds.lo.x <= node.bounds.hi.x && node.bounds.lo.y <= node.bounds.hi.y &&
                    node.bounds.lo.z <= node.bounds.hi.z,
                CS2TL_CORRUPT_CACHE, "Inverted BVH bounds.");
        if (node.leaf()) {
            require(uint64_t(node.first) + node.count <= count && node.left == 0 && node.right == 0,
                    CS2TL_CORRUPT_CACHE, "Invalid BVH leaf range.");
            for (uint32_t i = node.first; i < node.first + node.count; ++i) {
                require(!seen_order[i] && node.bounds.contains(scene.triangles[scene.order[i]].bounds()),
                        CS2TL_CORRUPT_CACHE, "Overlapping leaf ranges or incorrect leaf bounds.");
                seen_order[i] = true;
            }
        } else {
            require(node.first == 0 && node.left > index && node.right > index && node.left != node.right &&
                        node.left < scene.nodes.size() && node.right < scene.nodes.size(),
                    CS2TL_CORRUPT_CACHE, "Invalid BVH children.");
            require(node.bounds.contains(scene.nodes[node.left].bounds) &&
                        node.bounds.contains(scene.nodes[node.right].bounds),
                    CS2TL_CORRUPT_CACHE, "Child BVH bounds exceed parent.");
            pending.emplace_back(node.right, depth + 1);
            pending.emplace_back(node.left, depth + 1);
        }
    }
    require(std::all_of(seen_order.begin(), seen_order.end(), [](bool b) { return b; }) &&
                std::all_of(seen_node.begin(), seen_node.end(), [](bool b) { return b; }),
            CS2TL_CORRUPT_CACHE, "Unreachable BVH nodes or triangles.");
}
} // namespace
Scene load_cache(const std::filesystem::path &path) {
    const auto file = read_file(path);
    require(file.size() >= cache_header_size, CS2TL_CORRUPT_CACHE, "Truncated cache header.", 0);
    require(std::memcmp(file.data(), "CS2TLBVH", 8) == 0, CS2TL_UNSUPPORTED_FORMAT,
            "Unknown/legacy cache. Regenerate from physics text with cs2tl_build_cache.", 0);
    Reader reader(file);
    reader.position = 8;
    const uint32_t version = reader.u32(), header_size = reader.u32();
    require(version == cache_version && header_size == cache_header_size, CS2TL_UNSUPPORTED_FORMAT,
            "Unsupported cache version.", 8);
    const uint32_t flags = reader.u32(), leaf_size = reader.u32();
    const uint32_t triangles_count = reader.u32(), nodes_count = reader.u32(), order_count = reader.u32(),
                   hash_count = reader.u32();
    const uint64_t source = reader.u64(), payload_size = reader.u64(), checksum = reader.u64();
    require((flags & ~(CS2TL_SCENE_APPROXIMATE | CS2TL_SCENE_HAS_OPEN_GEOMETRY)) == 0 && leaf_size >= 1 &&
                leaf_size <= 64,
            CS2TL_CORRUPT_CACHE, "Invalid cache flags/build options.", 16);
    const uint64_t expected = uint64_t(triangles_count) * 44 + uint64_t(nodes_count) * 40 +
                              uint64_t(order_count) * 4 + uint64_t(hash_count) * 4;
    require(triangles_count <= max_triangles && nodes_count <= uint64_t(triangles_count) * 2 &&
                order_count == triangles_count && hash_count <= 1'000'000 && expected == payload_size &&
                payload_size == file.size() - cache_header_size,
            CS2TL_CORRUPT_CACHE, "Invalid cache counts/lengths.", 24);
    require(fingerprint(std::span(file).subspan(cache_header_size)) == checksum, CS2TL_CORRUPT_CACHE,
            "Cache payload checksum mismatch.", 56);
    std::vector<Triangle> triangles;
    triangles.reserve(triangles_count);
    for (uint32_t i = 0; i < triangles_count; ++i) {
        Triangle tri;
        tri.a = reader.vector();
        tri.b = reader.vector();
        tri.c = reader.vector();
        tri.surface = std::bit_cast<int32_t>(reader.u32());
        tri.object = reader.u32();
        triangles.push_back(tri);
    }
    Scene scene;
    try {
        scene = build_scene(std::move(triangles), BuildOptions{0, leaf_size}, false);
    } catch (const Error &e) {
        throw Error(CS2TL_CORRUPT_CACHE, e.what(), reader.position);
    }
    require((flags & CS2TL_SCENE_HAS_OPEN_GEOMETRY) == scene.flags, CS2TL_CORRUPT_CACHE,
            "Cache topology flag mismatch.", 16);
    scene.flags = flags;
    scene.fingerprint = source;
    scene.nodes.reserve(nodes_count);
    for (uint32_t i = 0; i < nodes_count; ++i) {
        BvhNode node;
        node.bounds.lo = reader.vector();
        node.bounds.hi = reader.vector();
        node.first = reader.u32();
        node.count = reader.u32();
        node.left = reader.u32();
        node.right = reader.u32();
        scene.nodes.push_back(node);
    }
    scene.order.reserve(order_count);
    for (uint32_t i = 0; i < order_count; ++i) {
        scene.order.push_back(reader.u32());
    }
    scene.surface_hashes.reserve(hash_count);
    for (uint32_t i = 0; i < hash_count; ++i) {
        scene.surface_hashes.push_back(reader.u32());
    }
    if (hash_count) {
        for (const auto &tri : scene.triangles) {
            require(tri.surface < 0 || static_cast<uint32_t>(tri.surface) < hash_count, CS2TL_CORRUPT_CACHE,
                    "Invalid cached surface index.");
        }
    }
    validate_tree(scene);
    return scene;
}
} // namespace cs2::traceline
