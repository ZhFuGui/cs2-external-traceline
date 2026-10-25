#pragma once
#include "api.h"
#include <exception>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace cs2::traceline {
inline constexpr std::string_view scene_cache_extension{CS2TL_SCENE_CACHE_EXTENSION};
class library_error final : public std::runtime_error {
  public:
    cs2tl_status status;
    uint64_t offset;
    explicit library_error(const cs2tl_error &e)
        : std::runtime_error(e.message), status(e.code), offset(e.offset) {}
};
inline cs2tl_error error_output() {
    cs2tl_error e{};
    e.struct_size = sizeof(e);
    return e;
}
inline void check(cs2tl_status status, const cs2tl_error &e) {
    if (status != CS2TL_OK) {
        throw library_error(e);
    }
}
inline cs2tl_segment segment(cs2tl_vec3 start, cs2tl_vec3 end, float endpoint_epsilon = 1e-5f,
                             float merge_epsilon = 1e-5f) {
    return {sizeof(cs2tl_segment), 0, start, end, endpoint_epsilon, merge_epsilon};
}
namespace detail {
template <class T> struct result_target {
    std::vector<T> values;
    std::exception_ptr failure;
    static T *CS2TL_CALL allocate(uint32_t count, void *user) noexcept {
        auto &target = *static_cast<result_target *>(user);
        try {
            target.values.resize(count);
            return target.values.data();
        } catch (...) {
            target.failure = std::current_exception();
            return nullptr;
        }
    }
    std::vector<T> finish(cs2tl_status status, const cs2tl_error &error) {
        if (failure) {
            std::rethrow_exception(failure);
        }
        check(status, error);
        return std::move(values);
    }
};
} // namespace detail
class scene {
    cs2tl_scene *handle{};
    explicit scene(cs2tl_scene *value) : handle(value) {}

  public:
    scene() = default;
    ~scene() { cs2tl_destroy_scene(handle); }
    scene(const scene &) = delete;
    scene &operator=(const scene &) = delete;
    scene(scene &&other) noexcept : handle(std::exchange(other.handle, nullptr)) {}
    scene &operator=(scene &&other) noexcept {
        if (this != &other) {
            cs2tl_destroy_scene(handle);
            handle = std::exchange(other.handle, nullptr);
        }
        return *this;
    }
    cs2tl_scene *native_handle() const noexcept { return handle; }
    static scene load(std::string_view path) {
        auto e = error_output();
        cs2tl_scene *out{};
        check(cs2tl_load_scene(std::string(path).c_str(), &out, &e), e);
        return scene(out);
    }
    static scene from_text(std::string_view path, const cs2tl_build_options *options = nullptr) {
        auto e = error_output();
        cs2tl_scene *out{};
        check(cs2tl_import_scene(std::string(path).c_str(), options, &out, &e), e);
        return scene(out);
    }
    static scene from_triangles(std::span<const cs2tl_triangle> triangles,
                                const cs2tl_build_options *options = nullptr) {
        if (triangles.size() > UINT32_MAX) {
            throw std::length_error("Too many triangles.");
        }
        auto e = error_output();
        cs2tl_scene *out{};
        check(
            cs2tl_create_scene(triangles.data(), static_cast<uint32_t>(triangles.size()), options, &out, &e),
            e);
        return scene(out);
    }
    static scene from_sources(std::span<const cs2tl_physics_source> sources,
                              const cs2tl_build_options *options = nullptr) {
        if (sources.size() > UINT32_MAX) {
            throw std::length_error("Source count exceeds uint32 representation.");
        }
        auto e = error_output();
        cs2tl_scene *out{};
        check(cs2tl_import_sources(sources.data(), static_cast<uint32_t>(sources.size()), options, &out, &e),
              e);
        return scene(out);
    }
    void save(std::string_view path) const {
        auto e = error_output();
        check(cs2tl_save_scene(handle, std::string(path).c_str(), &e), e);
    }
    cs2tl_scene_info info() const {
        auto e = error_output();
        cs2tl_scene_info out{};
        out.struct_size = sizeof(out);
        check(cs2tl_get_scene_info(handle, &out, &e), e);
        return out;
    }
    bool is_occluded(const cs2tl_segment &query) const {
        auto e = error_output();
        uint32_t out{};
        check(cs2tl_is_occluded(handle, &query, &out, &e), e);
        return out != 0;
    }
    bool is_visible(const cs2tl_segment &query) const { return !is_occluded(query); }
    std::optional<cs2tl_hit> closest(const cs2tl_segment &query) const {
        auto e = error_output();
        cs2tl_hit out{};
        uint32_t found{};
        check(cs2tl_trace_closest(handle, &query, &out, &found, &e), e);
        return found ? std::optional(out) : std::nullopt;
    }
    std::vector<cs2tl_hit> all_hits(const cs2tl_segment &query) const {
        auto e = error_output();
        detail::result_target<cs2tl_hit> target;
        const auto status = cs2tl_trace_all_to(handle, &query, target.allocate, &target, &e);
        return target.finish(status, e);
    }
    std::vector<cs2tl_interval> intervals(const cs2tl_segment &query) const {
        auto e = error_output();
        detail::result_target<cs2tl_interval> target;
        const auto status = cs2tl_query_intervals_to(handle, &query, target.allocate, &target, &e);
        return target.finish(status, e);
    }
    // Quiesce scene queries first. Releasing scratch does not unload geometry.
    void release_query_memory() const {
        auto e = error_output();
        check(cs2tl_release_query_memory(handle, &e), e);
    }
};
class profiles {
    cs2tl_profiles *handle{};
    explicit profiles(cs2tl_profiles *value) : handle(value) {}

  public:
    profiles() = default;
    ~profiles() { cs2tl_destroy_profiles(handle); }
    profiles(const profiles &) = delete;
    profiles &operator=(const profiles &) = delete;
    profiles(profiles &&other) noexcept : handle(std::exchange(other.handle, nullptr)) {}
    profiles &operator=(profiles &&other) noexcept {
        if (this != &other) {
            cs2tl_destroy_profiles(handle);
            handle = std::exchange(other.handle, nullptr);
        }
        return *this;
    }
    cs2tl_profiles *native_handle() const noexcept { return handle; }
    static profiles load(std::string_view path) {
        auto error = error_output();
        cs2tl_profiles *out{};
        check(cs2tl_load_profiles(std::string(path).c_str(), &out, &error), error);
        return profiles(out);
    }
    cs2tl_profiles_info info() const {
        auto error = error_output();
        cs2tl_profiles_info out{};
        out.struct_size = sizeof(out);
        check(cs2tl_get_profiles_info(handle, &out, &error), error);
        return out;
    }
    cs2tl_weapon_profile weapon(uint32_t id, uint32_t mode = 0) const {
        auto error = error_output();
        cs2tl_weapon_profile out{};
        out.struct_size = sizeof(out);
        check(cs2tl_get_weapon_profile(handle, id, mode, &out, &error), error);
        return out;
    }
    cs2tl_weapon_profile weapon_at(uint32_t index) const {
        auto error = error_output();
        cs2tl_weapon_profile out{};
        out.struct_size = sizeof(out);
        check(cs2tl_get_weapon_profile_at(handle, index, &out, &error), error);
        return out;
    }
    std::string weapon_name(uint32_t id) const {
        auto error = error_output();
        uint32_t bytes{};
        check(cs2tl_get_weapon_name(handle, id, nullptr, 0, &bytes, &error), error);
        std::string out(bytes, '\0');
        check(cs2tl_get_weapon_name(handle, id, out.data(), bytes, &bytes, &error), error);
        out.pop_back();
        return out;
    }
    cs2tl_material_profile material(uint32_t hash) const {
        auto error = error_output();
        cs2tl_material_profile out{};
        out.struct_size = sizeof(out);
        check(cs2tl_get_material_profile(handle, hash, &out, &error), error);
        return out;
    }
    std::string ammo_name(uint32_t id) const {
        auto error = error_output();
        uint32_t bytes{};
        check(cs2tl_get_weapon_ammo_name(handle, id, nullptr, 0, &bytes, &error), error);
        std::string out(bytes, '\0');
        check(cs2tl_get_weapon_ammo_name(handle, id, out.data(), bytes, &bytes, &error), error);
        out.pop_back();
        return out;
    }
};
class ballistics_context {
    cs2tl_ballistics *handle{};
    explicit ballistics_context(cs2tl_ballistics *value) : handle(value) {}

  public:
    ballistics_context() = default;
    ~ballistics_context() { cs2tl_destroy_ballistics(handle); }
    ballistics_context(const ballistics_context &) = delete;
    ballistics_context &operator=(const ballistics_context &) = delete;
    ballistics_context(ballistics_context &&other) noexcept : handle(std::exchange(other.handle, nullptr)) {}
    ballistics_context &operator=(ballistics_context &&other) noexcept {
        if (this != &other) {
            cs2tl_destroy_ballistics(handle);
            handle = std::exchange(other.handle, nullptr);
        }
        return *this;
    }
    static ballistics_context bind(const scene &geometry, const profiles &parameters) {
        auto error = error_output();
        cs2tl_ballistics *out{};
        check(cs2tl_create_ballistics(geometry.native_handle(), parameters.native_handle(), &out, &error),
              error);
        return ballistics_context(out);
    }
    cs2tl_ballistics *native_handle() const noexcept { return handle; }
    cs2tl_ballistics_info info() const {
        auto error = error_output();
        cs2tl_ballistics_info out{};
        out.struct_size = sizeof(out);
        check(cs2tl_get_ballistics_info(handle, &out, &error), error);
        return out;
    }
    void release_query_memory() const {
        auto error = error_output();
        check(cs2tl_release_ballistics_memory(handle, &error), error);
    }
    cs2tl_shot_prediction remaining_damage(uint32_t weapon_id, const cs2tl_segment &query,
                                           uint32_t fire_mode = 0) const {
        auto error = error_output();
        const cs2tl_remaining_damage_request request{sizeof(request), weapon_id, fire_mode, 0, query};
        cs2tl_shot_prediction out{};
        out.struct_size = sizeof(out);
        check(cs2tl_calculate_remaining_damage(handle, &request, &out, &error), error);
        return out;
    }
};
} // namespace cs2::traceline
