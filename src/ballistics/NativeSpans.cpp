#include "NativeSpans.h"
#include "NativeMath.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <bit>
#include <cmath>

namespace cs2::traceline {
float native_ray_length(cs2tl_vec3 value) noexcept {
    // Native RVA 0x2134f0: (z*z + y*y) + x*x, then sqrtss.
    const auto yz = native_f32::add(native_f32::mul(value.z, value.z), native_f32::mul(value.y, value.y));
    return std::sqrt(native_f32::add(yz, native_f32::mul(value.x, value.x)));
}
NativeSpanDistances native_span_distances(float ray_length, float start, float end) noexcept {
    // MAXSS returns its second operand for equal/unordered values.
    auto max_second = [](float a, float b) { return a > b ? a : b; };
    const auto first = max_second(0, native_f32::mul(ray_length, start));
    return {first, max_second(native_f32::mul(ray_length, end), first)};
}
float native_material_thickness(cs2tl_vec3 origin, cs2tl_vec3 delta, float entry, float exit) noexcept {
    // TraceResult endpoints are formed first in world-space float32 by
    // 0xa79ff0, then subtracted/normalized by 0xa9d6e0..0xa9d73a. Computing
    // (exit-entry)*length instead changes cancellation and rounding.
    const auto first = native_f32::add(origin, native_f32::mul(delta, entry));
    const auto last = native_f32::add(origin, native_f32::mul(delta, exit));
    return native_ray_length(native_f32::sub(last, first));
}
void sort_native_events(std::span<BallisticEvent> events) {
    std::sort(events.begin(), events.end(), [](const auto &a, const auto &b) {
        const auto first = a.fraction(), second = b.fraction();
        if (first == second) {
            return std::bit_cast<int8_t>(a.kind()) < std::bit_cast<int8_t>(b.kind());
        }
        return first < second;
    });
}
NativeSpanBuild build_native_spans(std::span<BallisticEvent> events, cs2tl_vec3 delta, float max_fraction,
                                   std::vector<BallisticSpan> &output) {
    require(events.size() <= UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
            "Ballistic event indices exceed uint32 representation.");
    output.clear();
    const NativeSpanBuild result{native_ray_length(delta), events.size() > 32768};
    if (result.extended_indices) {
        warn("Ballistic events exceed the native 15-bit index domain; continuing with wide indices.");
    }
    constexpr float seam_limit = 0x1p-9f;
    for (size_t i = 1; i < events.size(); ++i) {
        auto &previous = events[i - 1];
        auto &current = events[i];
        if (current.kind() != 0 || previous.kind() != 1) {
            continue;
        }
        const auto previous_fraction = previous.fraction();
        const auto current_fraction = current.fraction();
        const auto gap = native_f32::mul(native_f32::sub(current_fraction, previous_fraction), result.length);
        if (!(gap <= seam_limit)) {
            continue;
        }
        const auto temporary = previous;
        previous = current;
        current = temporary;
        previous.fraction(previous_fraction);
        current.fraction(current_fraction);
    }
    if (events.empty()) {
        return result;
    }
    auto emit = [&](float start, float end, uint32_t entry, uint32_t exit, bool solid) {
        output.push_back({start, end, 1, 0, entry, exit, solid ? 1u : 0u});
    };
    float start = 0;
    uint32_t entry = 0;
    uint8_t previous_kind = 1;
    for (uint32_t i = 0; i < events.size(); ++i) {
        const auto kind = events[i].kind();
        if (kind == previous_kind) {
            continue;
        }
        previous_kind = kind;
        if (kind == 1) {
            continue;
        }
        if (i > 0) {
            const auto prior_fraction = events[i - 1].fraction();
            emit(start, prior_fraction, entry, i - 1, true);
            start = prior_fraction;
            entry = i - 1;
        }
        emit(start, events[i].fraction(), entry, i, false);
        start = events[i].fraction();
        entry = i;
    }
    if (output.empty()) {
        return result;
    }
    if (!output.back().solid()) {
        const auto last = output.back();
        const auto final_index = static_cast<uint32_t>(events.size() - 1);
        if (max_fraction < 1 && last.exit_index < final_index && events.back().kind() == 1) {
            const auto exit_fraction = events.back().fraction();
            emit(last.end, exit_fraction, last.exit_index, final_index, true);
            emit(exit_fraction, 1, final_index, final_index, false);
        } else {
            emit(last.end, 1, last.exit_index, last.exit_index, true);
        }
    }
    if (output.front().solid()) {
        const auto first = output.front();
        output.clear();
        emit(first.end, first.end, first.exit_index, first.exit_index, false);
        emit(first.end, 1, first.exit_index, first.exit_index, true);
    }
    return result;
}
NativeExtent rescale_native_extent(std::span<BallisticEvent> events, std::vector<BallisticSpan> &spans,
                                   cs2tl_vec3 delta, float scale) {
    const float inverse = native_f32::div(1, scale);
    // MINSS returns the second operand for unordered/equal inputs.
    auto cap_at_one = [](float value) { return value < 1.0f ? value : 1.0f; };
    for (auto &event : events) {
        event.fraction(cap_at_one(native_f32::mul(inverse, event.fraction())));
    }
    auto keep_spans = spans.size();
    for (size_t i = 0; i < spans.size(); ++i) {
        auto &span = spans[i];
        span.start = native_f32::mul(inverse, span.start);
        span.end = cap_at_one(native_f32::mul(inverse, span.end));
        if (keep_spans == spans.size() && span.start > 1) {
            keep_spans = i;
        }
    }
    spans.resize(keep_spans);
    auto keep_events = events.size();
    if (!spans.empty()) {
        keep_events = size_t(spans.back().exit_index) + 1;
        require(keep_events <= events.size(), CS2TL_INVALID_GEOMETRY,
                "Native extent exit index has no backing event.");
    }
    return {
        {native_f32::mul(scale, delta.x), native_f32::mul(scale, delta.y), native_f32::mul(scale, delta.z)},
        keep_events,
        1};
}
} // namespace cs2::traceline
