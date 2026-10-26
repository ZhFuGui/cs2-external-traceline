#pragma once
#include "scene/Scene.h"
#include <filesystem>

namespace cs2::traceline {
// Collision scene container versions are independent of the preferred .cs2tl
// suffix and the current AABB BVH implementation. Keep existing headers valid.
inline constexpr uint32_t cache_version = 1;
inline constexpr uint32_t cache_header_size = 64;
inline constexpr uint32_t curved_cache_version = 2;
inline constexpr uint32_t curved_cache_header_size = 72;
// v3 keeps the v2 header size and stores a native mesh-object count in its
// former reserved field. Sorted object ids follow the primitive records.
inline constexpr uint32_t surface_cache_version = 3;
// v4 appends bounded, typed metadata chunks. The first chunk preserves source
// winding; future required chunks are rejected by readers that cannot use them.
inline constexpr uint32_t native_cache_version = 4;
inline constexpr uint32_t native_cache_header_size = 80;
inline constexpr uint32_t source_winding_chunk = 1;
inline constexpr uint32_t physics_source_chunk = 2;
inline constexpr uint32_t wide_chunk_length = 0x80000000u;
void save_cache(const Scene &scene, const std::filesystem::path &path);
Scene load_cache(const std::filesystem::path &path);
} // namespace cs2::traceline
