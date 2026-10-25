#pragma once
#include "cs2-external-traceline/types.h"
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#if defined(_WIN32)
#include <Windows.h>
#else
#include <cstdio>
#endif

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
using WarningCallback = void (*)(void *, const char *, uint64_t);
inline thread_local WarningCallback warning_callback{};
inline thread_local void *warning_user{};
inline void warn(std::string_view message, uint64_t offset = UINT64_MAX) noexcept {
    try {
        std::string text(message);
        if (warning_callback) {
            warning_callback(warning_user, text.c_str(), offset);
        } else {
            text = "cs2-external-traceline warning: " + text + "\n";
#if defined(_WIN32)
            OutputDebugStringA(text.c_str());
#else
            std::fputs(text.c_str(), stderr);
#endif
        }
    } catch (...) {
        // Diagnostics must never turn a successful operation into a failure.
    }
}
inline constexpr uint64_t large_triangle_notice = 4'000'000;
inline constexpr uint64_t large_file_notice = 512ull * 1024 * 1024;
} // namespace cs2::traceline
