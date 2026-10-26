#include "NativeQueryExtent.h"
#include "NativeMath.h"
#include "diagnostics/Error.h"
#include <cmath>

namespace cs2::traceline {
void query_native_extent(NativeQueryContext &context, cs2tl_vec3 origin, cs2tl_vec3 original_delta,
                         NativeQueryCallbacks callbacks, int32_t extra_attempts, uint32_t original_body_types,
                         bool append_nonstatic) {
    namespace f32 = native_f32;
    require(callbacks.query && callbacks.trace, CS2TL_INVALID_ARGUMENT,
            "Native extent callbacks are required.");
    const auto length = native_ray_length(original_delta);
    require(std::isfinite(length), CS2TL_INVALID_ARGUMENT,
            "Native query length exceeds the finite float32 range.");
    const auto direction = length == 0 ? cs2tl_vec3{} : f32::mul(original_delta, f32::div(1, length));
    auto delta = original_delta;
    const auto original_end = f32::add(origin, original_delta);
    auto query = [&](float parameter, uint32_t body_types) {
        callbacks.query(callbacks.user, context, origin, delta, parameter, body_types);
    };
    auto rebuild = [&] { build_native_spans(context.events, delta, context.max_fraction, context.spans); };
    auto clear = [&] {
        context.events.clear();
        context.started_inside = false;
    };
    clear();
    query(153, 1);
    context.max_fraction = 1;
    if (!context.events.empty() && length > 0) {
        context.max_fraction = f32::add(context.events.front().fraction(), f32::div(153, length));
    }
    rebuild();
    context.delta = delta;
    int32_t passes = 0, remaining_checks = 0;
    for (;;) {
        const auto current_length = native_ray_length(context.delta);
        float solid_sum = 0;
        auto min_second = [](float a, float b) { return a < b ? a : b; };
        for (const auto &span : context.spans) {
            if (span.solid()) {
                solid_sum = f32::add(solid_sum, f32::sub(min_second(span.end, context.max_fraction),
                                                         min_second(span.start, context.max_fraction)));
            }
        }
        if (f32::mul(solid_sum, current_length) >= 90 || ++passes > 4) {
            break;
        }
        const bool ends_at_exit =
            context.events.empty() ? !context.started_inside : context.events.back().kind() == 1;
        if (ends_at_exit) {
            if (++remaining_checks > extra_attempts || context.events.empty()) {
                break;
            }
            const auto &last = context.events.back();
            cs2tl_vec3 normal;
            std::memcpy(&normal, last.bytes.data(), sizeof(normal));
            const auto start =
                f32::add(f32::add(origin, f32::mul(delta, last.fraction())), f32::mul(normal, .09375f));
            const auto remaining = callbacks.trace(callbacks.user, start, original_end);
            delta = remaining.hit
                        ? f32::mul(direction,
                                   f32::add(native_ray_length(f32::sub(remaining.end_position, origin)), 90))
                        : original_delta;
        } else {
            delta = f32::mul(direction, f32::add(native_ray_length(delta), 90));
        }
        clear();
        query(-1, 1);
        context.max_fraction = 1;
        rebuild();
        context.delta = delta;
    }
    if (context.max_fraction < 1) {
        const auto cropped =
            rescale_native_extent(context.events, context.spans, delta, context.max_fraction);
        context.events.resize(cropped.event_count);
        context.max_fraction = cropped.max_fraction;
        context.delta = delta = cropped.delta;
    }
    const auto previous_count = context.events.size();
    if (append_nonstatic) {
        query(-1, original_body_types & ~1u);
        if (context.events.size() != previous_count) {
            rebuild();
        }
    } else {
        clear();
        query(-1, original_body_types);
        context.max_fraction = 1;
        rebuild();
    }
    context.delta = delta;
}
} // namespace cs2::traceline
