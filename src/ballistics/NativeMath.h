#pragma once
#include "cs2-external-traceline/types.h"
#if defined(_MSC_VER) || defined(__SSE__)
#include <xmmintrin.h>
#endif

namespace cs2::traceline::native_f32 {
inline uint32_t control() noexcept {
#if defined(_MSC_VER) || defined(__SSE__)
    return _mm_getcsr() & 0xffc0u; // Ignore sticky exception status flags.
#else
    return 0;
#endif
}
// Compile native arithmetic translation units with /fp:strict. On x64 SSE,
// each float operation rounds directly without spilling every result to memory.
inline float add(float a, float b) noexcept {
    return a + b;
}
inline float sub(float a, float b) noexcept {
    return a - b;
}
inline float mul(float a, float b) noexcept {
    return a * b;
}
inline float div(float a, float b) noexcept {
    return a / b;
}
inline float cap_one(float value) noexcept {
    return value < 1 ? value : 1;
}
inline cs2tl_vec3 add(cs2tl_vec3 a, cs2tl_vec3 b) noexcept {
    return {add(a.x, b.x), add(a.y, b.y), add(a.z, b.z)};
}
inline cs2tl_vec3 sub(cs2tl_vec3 a, cs2tl_vec3 b) noexcept {
    return {sub(a.x, b.x), sub(a.y, b.y), sub(a.z, b.z)};
}
inline cs2tl_vec3 mul(cs2tl_vec3 a, float b) noexcept {
    return {mul(a.x, b), mul(a.y, b), mul(a.z, b)};
}
inline cs2tl_vec3 cross(cs2tl_vec3 a, cs2tl_vec3 b) noexcept {
    return {sub(mul(a.y, b.z), mul(a.z, b.y)), sub(mul(a.z, b.x), mul(a.x, b.z)),
            sub(mul(a.x, b.y), mul(a.y, b.x))};
}
inline float dot_xyz(cs2tl_vec3 a, cs2tl_vec3 b) noexcept {
    return add(add(mul(a.x, b.x), mul(a.y, b.y)), mul(a.z, b.z));
}
inline float dot_zyx(cs2tl_vec3 a, cs2tl_vec3 b) noexcept {
    return add(add(mul(a.z, b.z), mul(a.y, b.y)), mul(a.x, b.x));
}
} // namespace cs2::traceline::native_f32
