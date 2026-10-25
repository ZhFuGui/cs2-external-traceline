#pragma once
#include "scene/Scene.h"
#include <filesystem>

namespace cs2::traceline {
inline constexpr uint32_t cache_version = 1;
inline constexpr uint32_t cache_header_size = 64;
inline constexpr uint32_t curved_cache_version = 2;
inline constexpr uint32_t curved_cache_header_size = 72;
void save_cache(const Scene &scene, const std::filesystem::path &path);
Scene load_cache(const std::filesystem::path &path);
} // namespace cs2::traceline
