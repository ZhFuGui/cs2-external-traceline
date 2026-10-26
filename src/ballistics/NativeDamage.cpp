#include "NativeDamage.h"
#include "NativeMath.h"
#include "ShotSimulator.h"
#include "diagnostics/Error.h"
#include "profiles/Ruleset.h"
#include <cmath>

namespace cs2::traceline {
namespace {
bool endpoint(const BallisticEvent &event) noexcept {
    return event.bytes[51] == std::byte{0xff};
}
} // namespace
float penetration_damage(float damage, float power, float thickness, float minimum_penetration,
                         const ResolvedMaterial &entry, const ResolvedMaterial &exit) noexcept {
    float combined = minimum_penetration, damage_fraction = 0.16f;
    // Native 0xa9d73a skips all same-material overrides when the minimum
    // intermediate penetration modifier is below 0.1.
    if (minimum_penetration >= minimum_material_penetration && entry.material == exit.material) {
        if (entry.material == 'W' || entry.material == 'U') {
            combined = 3;
        } else if (entry.material == 'L') {
            combined = 2;
        }
        if (thickness < 6 && (entry.material == 'G' || entry.material == 'Y')) {
            combined = 3;
            damage_fraction = 0.05f;
        }
        if ((entry.flags & exit.flags & CS2TL_MATERIAL_NATIVE_FLAG13) != 0) {
            combined = thickness < 6 ? 32.0f : 3.0f;
            if (thickness < 6) {
                damage_fraction = 0.00001f;
            }
        }
    }
    const float inverse = std::max(0.0f, 1.0f / combined);
    const float power_cost = std::max(0.0f, (3.0f / power) * 1.25f);
    const float fixed_loss = power_cost * (inverse * 3.0f) + damage * damage_fraction;
    const float loss = std::max(0.0f, (thickness * thickness * inverse) / 24.0f + fixed_loss);
    return damage - loss;
}
NativePointStatus plan_native_point_energy(NativeQueryContext &context, float point_fraction,
                                           float point_distance, float weapon_range) {
    if (!std::isfinite(point_fraction) || !std::isfinite(point_distance) || !std::isfinite(weapon_range) ||
        point_fraction < 0 || point_fraction > 1 || point_distance < 0 || point_distance > weapon_range) {
        return NativePointStatus::outside_extent;
    }
    if (context.started_inside) {
        return NativePointStatus::origin_inside;
    }
    for (const auto &event : context.events) {
        require(!endpoint(event), CS2TL_INVALID_GEOMETRY, "Physical event uses a reserved endpoint source.");
        if (event.fraction() == point_fraction) {
            // Equal-key event ordering cannot certify which side of a real
            // boundary contains B. Point energy is defined for an air prefix.
            return NativePointStatus::incomplete_boundary;
        }
    }
    BallisticEvent marker;
    marker.fraction(point_fraction);
    marker.kind(0);
    marker.bytes[51] = std::byte{0xff};
    context.events.push_back(marker);
    sort_native_events(context.events);
    build_native_spans(context.events, context.delta, context.max_fraction, context.spans);
    for (size_t i = 0; i < context.spans.size(); ++i) {
        const auto &span = context.spans[i];
        require(span.exit_index < context.events.size(), CS2TL_INVALID_GEOMETRY,
                "Point span has no backing endpoint event.");
        if (!span.solid() && endpoint(context.events[span.exit_index])) {
            if (context.events[span.exit_index].fraction() != point_fraction) {
                context.spans.clear();
                return NativePointStatus::incomplete_boundary;
            }
            context.spans.resize(i + 1);
            return NativePointStatus::ready;
        }
    }
    context.spans.clear();
    return NativePointStatus::incomplete_boundary;
}
bool advance_native_damage(const NativeQueryContext &context, const BallisticSpan &span, cs2tl_vec3 origin,
                           NativeMaterialProvider materials, NativeDamageState &state) {
    namespace f32 = native_f32;
    const auto distances = native_span_distances(state.query_length, span.start, span.end);
    if (!span.solid()) {
        const auto exponent = f32::div(f32::sub(distances.end, distances.start), range_decay_units);
        state.damage = f32::mul(state.damage, std::pow(state.range_modifier, exponent));
        require(std::isfinite(state.damage), CS2TL_INVALID_ARGUMENT,
                "Native air attenuation exceeds the finite float32 range.");
        return false;
    }
    require(materials.get && span.entry_index < context.events.size() &&
                span.exit_index < context.events.size(),
            CS2TL_INVALID_GEOMETRY, "Native material span has no backing provider/events.");
    auto material = [&](uint32_t index) {
        require(!endpoint(context.events[index]), CS2TL_INVALID_GEOMETRY,
                "A point endpoint cannot supply physical material data.");
        const auto value = materials.get(materials.user, context.events[index]);
        require(value.valid && std::isfinite(value.penetration) && value.penetration >= 0,
                CS2TL_DATA_NOT_FOUND, "Native boundary material is unavailable or invalid.");
        return value;
    };
    const auto entry = material(span.entry_index);
    if (distances.end > max_penetration_distance) {
        state.remaining_penetrations = 0;
        state.stop_reason = CS2TL_STOP_PENETRATION_DISTANCE;
    } else if (entry.penetration < minimum_material_penetration) {
        state.remaining_penetrations = 0;
        state.stop_reason = CS2TL_STOP_MATERIAL;
    }
    if (state.remaining_penetrations == 0) {
        if (!state.stop_reason) {
            state.stop_reason = CS2TL_STOP_PENETRATION_COUNT;
        }
        return true;
    }
    const auto exit = material(span.exit_index);
    float minimum = entry.penetration;
    for (uint64_t i = span.entry_index; i <= span.exit_index; ++i) {
        const auto next = material(static_cast<uint32_t>(i)).penetration;
        // MINSS takes its second operand on equality, preserving signed zero.
        minimum = minimum < next ? minimum : next;
    }
    const auto thickness =
        native_material_thickness(origin, context.delta, context.events[span.entry_index].fraction(),
                                  context.events[span.exit_index].fraction());
    state.damage = penetration_damage(state.damage, state.penetration_power, thickness, minimum, entry, exit);
    if (state.damage < 1 || !std::isfinite(state.damage)) {
        state.stop_reason = state.penetration_power == 0 ? CS2TL_STOP_PENETRATION_POWER : CS2TL_STOP_DAMAGE;
        return true;
    }
    --state.remaining_penetrations;
    return false;
}
bool precompute_native_damage(NativeQueryContext &context, cs2tl_vec3 origin,
                              NativeMaterialProvider materials, NativeDamageState &state) {
    state.query_length = native_ray_length(context.delta);
    for (size_t i = 0; i < context.spans.size(); ++i) {
        auto &span = context.spans[i];
        if (advance_native_damage(context, span, origin, materials, state)) {
            span.end = 1;
            span.initial_damage = state.damage = 0;
            span.initial_penetrations = state.remaining_penetrations;
            context.spans.resize(i + 1);
            return true;
        }
        span.initial_damage = state.damage;
        span.initial_penetrations = state.remaining_penetrations;
    }
    return false;
}
NativePointStatus consume_native_point_energy(NativeQueryContext &context, cs2tl_vec3 origin,
                                              NativeMaterialProvider materials, NativeDamageState &state) {
    require(!context.spans.empty() && !context.spans.back().solid() &&
                context.spans.back().exit_index < context.events.size() &&
                endpoint(context.events[context.spans.back().exit_index]),
            CS2TL_INVALID_GEOMETRY, "Point energy requires a planned air prefix ending at B.");
    return precompute_native_damage(context, origin, materials, state) ? NativePointStatus::stopped
                                                                       : NativePointStatus::reached;
}
} // namespace cs2::traceline
