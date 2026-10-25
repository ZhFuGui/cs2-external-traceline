#pragma once
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace cs2::traceline {
std::filesystem::path utf8_path(std::string_view value);
std::vector<uint8_t> read_file(const std::filesystem::path &path);
void atomic_write(const std::filesystem::path &path, std::span<const uint8_t> bytes);
uint64_t fingerprint(std::span<const uint8_t> bytes);
} // namespace cs2::traceline
