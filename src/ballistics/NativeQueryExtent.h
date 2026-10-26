#pragma once
#include "NativeSpans.h"

namespace cs2::traceline {
struct NativeQueryContext {
    std::vector<BallisticEvent> events;
    std::vector<BallisticSpan> spans;
    cs2tl_vec3 delta{};
    float max_fraction{1};
    bool started_inside{};
};
struct NativeRemainingTrace {
    bool hit{};
    cs2tl_vec3 end_position{};
};
struct NativeQueryCallbacks {
    void *user{};
    // Append complete, filtered world events; sort the resulting stream. The
    // callback owns started_inside, but must not construct material spans.
    void (*query)(void *, NativeQueryContext &, cs2tl_vec3 origin, cs2tl_vec3 delta, float parameter,
                  uint32_t body_types){};
    // Zero-radius closest trace, from the supplied point to the original end.
    NativeRemainingTrace (*trace)(void *, cs2tl_vec3 origin, cs2tl_vec3 end){};
};
// server!a8dab0 plus a769b0; build 26144732. Native 90/153 distances and
// penetration-driven retry counts are game rules, not resource-size limits.
void query_native_extent(NativeQueryContext &context, cs2tl_vec3 origin, cs2tl_vec3 original_delta,
                         NativeQueryCallbacks callbacks, int32_t extra_attempts = 4,
                         uint32_t original_body_types = 15, bool append_nonstatic = true);
} // namespace cs2::traceline
