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
} // namespace cs2::traceline
