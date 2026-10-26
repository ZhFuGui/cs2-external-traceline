#pragma once
#include "cs2-external-traceline/types.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace cs2::traceline {
// Only fraction@12 and kind@50 are interpreted by the restored span builder.
// The full payload follows its event when native seam handling swaps records.
struct BallisticEvent {
    std::array<std::byte, 56> bytes{};
    float fraction() const noexcept {
        float value;
        std::memcpy(&value, bytes.data() + 12, sizeof(value));
        return value;
    }
    void fraction(float value) noexcept { std::memcpy(bytes.data() + 12, &value, sizeof(value)); }
    uint8_t kind() const noexcept { return std::to_integer<uint8_t>(bytes[50]); }
    void kind(uint8_t value) noexcept { bytes[50] = std::byte{value}; }
};
static_assert(sizeof(BallisticEvent) == 56);
struct BallisticSpan {
    float start{}, end{}, initial_damage{1};
    uint32_t initial_penetrations{}, entry_index{}, exit_index{}, flags{};
    bool solid() const noexcept { return (flags & 1u) != 0; }
};
struct NativeSpanBuild {
    float length{};
    bool extended_indices{};
};
struct NativeExtent {
    cs2tl_vec3 delta{};
    size_t event_count{};
    float max_fraction{1};
};
struct NativeSpanDistances {
    float start{}, end{};
};
float native_ray_length(cs2tl_vec3 delta) noexcept;
NativeSpanDistances native_span_distances(float ray_length, float start, float end) noexcept;
float native_material_thickness(cs2tl_vec3 origin, cs2tl_vec3 delta, float entry, float exit) noexcept;
// The audited MSVC introsort compares finite fractions, then signed kind.
// Equal-key payload permutations were checked against both native sort bodies.
void sort_native_events(std::span<BallisticEvent> events);
// Build 26144732, server.dll RVA 0xa7ac10. Inputs are already ordered; records
// are mutated in place. No sorting, deduplication or max_fraction clipping is
// invented here. Public callers must validate their own geometric inputs.
// Indices remain wide beyond the native 15-bit domain: warn and continue,
// without wrapping, discarding events or claiming native overflow equivalence.
NativeSpanBuild build_native_spans(std::span<BallisticEvent> events, cs2tl_vec3 delta, float max_fraction,
                                   std::vector<BallisticSpan> &output);
// RVA 0xa9f780. The upstream caller chooses scale; this only reproduces the
// native rescaling/truncation operation. All input event slots are rescaled,
// including slots past the returned valid event_count. Payloads are not sorted.
NativeExtent rescale_native_extent(std::span<BallisticEvent> events, std::vector<BallisticSpan> &spans,
                                   cs2tl_vec3 delta, float scale);
} // namespace cs2::traceline
