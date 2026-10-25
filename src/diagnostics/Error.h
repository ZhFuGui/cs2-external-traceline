#pragma once
#include "cs2-external-traceline/types.h"
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace cs2::traceline {
class Error final : public std::runtime_error {
  public:
    cs2tl_status code;
    uint64_t offset;
    Error(cs2tl_status value, std::string message, uint64_t location = UINT64_MAX)
        : std::runtime_error(std::move(message)), code(value), offset(location) {}
};
inline void require(bool condition, cs2tl_status code, std::string_view message,
                    uint64_t offset = UINT64_MAX) {
    if (!condition) {
        throw Error(code, std::string(message), offset);
    }
}
template <class MessageFactory>
inline void require_lazy(bool condition, cs2tl_status code, MessageFactory &&message,
                         uint64_t offset = UINT64_MAX) {
    if (!condition) {
        throw Error(code, message(), offset);
    }
}
inline constexpr uint32_t max_triangles = 4'000'000;
inline constexpr uint64_t max_file_bytes = 512ull * 1024 * 1024;
} // namespace cs2::traceline
