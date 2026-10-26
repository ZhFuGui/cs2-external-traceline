#include "CacheFormat.h"
#include "FileIO.h"
#include "diagnostics/Error.h"
#include <bit>

namespace cs2::traceline {
namespace {
class Writer {
  public:
    std::vector<uint8_t> data;
    void u32(uint32_t v) {
        for (unsigned i = 0; i < 4; ++i) {
            data.push_back(static_cast<uint8_t>(v >> (8 * i)));
        }
    }
    void u64(uint64_t v) {
        u32(static_cast<uint32_t>(v));
        u32(static_cast<uint32_t>(v >> 32));
    }
    void f32(double v) { u32(std::bit_cast<uint32_t>(static_cast<float>(v))); }
    void vector(Vec3 v) {
        f32(v.x);
        f32(v.y);
        f32(v.z);
    }
};
} // namespace
void save_cache(const Scene &scene, const std::filesystem::path &path) {
    const bool curved = !scene.primitives.empty();
    const bool surfaces = !scene.native_surface_objects.empty();
    const bool native = !scene.source_winding_flips.empty();
    const uint32_t header_size = native ? native_cache_header_size
                                        : (curved || surfaces ? curved_cache_header_size : cache_header_size);
    require(scene.triangles.size() <= UINT32_MAX && scene.nodes.size() <= UINT32_MAX &&
                scene.order.size() <= UINT32_MAX && scene.surface_hashes.size() <= UINT32_MAX &&
                scene.primitives.size() <= UINT32_MAX && scene.native_surface_objects.size() <= UINT32_MAX &&
                scene.source_winding_flips.size() <= UINT32_MAX,
            CS2TL_UNSUPPORTED_FORMAT, "Cache counts exceed uint32 representation.");
    require(!native || scene.source_winding_flips.size() == scene.triangles.size(), CS2TL_INVALID_GEOMETRY,
            "Source winding count must match the triangle count.");
    const uint64_t metadata_size = native ? uint64_t(scene.source_winding_flips.size()) + 8 : 0;
    const uint64_t payload_size = uint64_t(scene.triangles.size()) * 44 + uint64_t(scene.nodes.size()) * 40 +
                                  uint64_t(scene.order.size()) * 4 +
                                  uint64_t(scene.surface_hashes.size()) * 4 +
                                  uint64_t(scene.primitives.size()) * 84 +
                                  uint64_t(scene.native_surface_objects.size()) * 4 + metadata_size;
    if (payload_size + header_size > large_file_notice) {
        warn("Large BVH cache (>512 MiB); writing continues.");
    }
    require(payload_size <= std::vector<uint8_t>{}.max_size() - header_size, CS2TL_OUT_OF_MEMORY,
            "Cache size cannot be represented by this process.");
    Writer payload;
    payload.data.reserve(static_cast<size_t>(payload_size) + header_size);
    for (const auto &tri : scene.triangles) {
        payload.vector(tri.a);
        payload.vector(tri.b);
        payload.vector(tri.c);
        payload.u32(std::bit_cast<uint32_t>(tri.surface));
        payload.u32(tri.object);
    }
    for (const auto &node : scene.nodes) {
        payload.vector(node.bounds.lo);
        payload.vector(node.bounds.hi);
        payload.u32(node.first);
        payload.u32(node.count);
        payload.u32(node.left);
        payload.u32(node.right);
    }
    for (auto value : scene.order) {
        payload.u32(value);
    }
    for (auto value : scene.surface_hashes) {
        payload.u32(value);
    }
    for (const auto &p : scene.primitives) {
        payload.vector(p.a);
        payload.vector(p.b);
        payload.f32(p.radius);
        payload.u32(std::bit_cast<uint32_t>(p.surface));
        payload.u32(p.object);
        for (auto x : p.pose) {
            payload.f32(x);
        }
    }
    for (auto object : scene.native_surface_objects) {
        payload.u32(object);
    }
    if (native) {
        payload.u32(source_winding_chunk);
        payload.u32(static_cast<uint32_t>(scene.source_winding_flips.size()));
        payload.data.insert(payload.data.end(), scene.source_winding_flips.begin(),
                            scene.source_winding_flips.end());
    }
    Writer file;
    for (char c : std::string_view("CS2TLBVH")) {
        file.data.push_back(static_cast<uint8_t>(c));
    }
    file.u32(native ? native_cache_version
                    : (surfaces ? surface_cache_version : (curved ? curved_cache_version : cache_version)));
    file.u32(header_size);
    file.u32(scene.flags);
    file.u32(scene.leaf_size);
    file.u32(static_cast<uint32_t>(scene.triangles.size()));
    file.u32(static_cast<uint32_t>(scene.nodes.size()));
    file.u32(static_cast<uint32_t>(scene.order.size()));
    file.u32(static_cast<uint32_t>(scene.surface_hashes.size()));
    file.u64(scene.fingerprint);
    file.u64(payload.data.size());
    file.u64(fingerprint(payload.data));
    if (curved || surfaces || native) {
        file.u32(static_cast<uint32_t>(scene.primitives.size()));
        file.u32(static_cast<uint32_t>(scene.native_surface_objects.size()));
    }
    if (native) {
        file.u64(metadata_size);
    }
    payload.data.insert(payload.data.begin(), file.data.begin(), file.data.end());
    atomic_write(path, payload.data);
}
} // namespace cs2::traceline
