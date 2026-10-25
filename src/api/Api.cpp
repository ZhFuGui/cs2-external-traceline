#include "cs2-external-traceline/api.h"
#include "ballistics/ShotSimulator.h"
#include "diagnostics/Error.h"
#include "import/PhysicsTextReader.h"
#include "storage/CacheFormat.h"
#include "storage/FileIO.h"
#include "trace/TraceQuery.h"
#include <algorithm>
#include <cstring>
#include <memory>
#include <new>

namespace tl = cs2::traceline;
struct cs2tl_scene {
    tl::Scene data;
    mutable tl::QueryWorkspacePool workspaces;
    explicit cs2tl_scene(tl::Scene value) : data(std::move(value)) {}
};
namespace {
void write_error(cs2tl_error *error, cs2tl_status code, const char *message,
                 uint64_t offset = UINT64_MAX) noexcept {
    if (!error) {
        return;
    }
    error->code = code;
    error->offset = offset;
    const size_t size = std::min(std::strlen(message), sizeof(error->message) - 1);
    std::memcpy(error->message, message, size);
    error->message[size] = '\0';
}
template <class F> cs2tl_status guarded(cs2tl_error *error, F &&function) noexcept {
    if (error && error->struct_size < sizeof(cs2tl_error)) {
        return CS2TL_INVALID_ARGUMENT;
    }
    write_error(error, CS2TL_OK, "");
    try {
        function();
        return CS2TL_OK;
    } catch (const tl::Error &e) {
        write_error(error, e.code, e.what(), e.offset);
        return e.code;
    } catch (const std::bad_alloc &) {
        write_error(error, CS2TL_OUT_OF_MEMORY, "Memory allocation failed.");
        return CS2TL_OUT_OF_MEMORY;
    } catch (const std::filesystem::filesystem_error &) {
        write_error(error, CS2TL_IO_ERROR, "Filesystem operation failed.");
        return CS2TL_IO_ERROR;
    } catch (const std::exception &e) {
        write_error(error, CS2TL_INTERNAL_ERROR, e.what());
        return CS2TL_INTERNAL_ERROR;
    } catch (...) {
        write_error(error, CS2TL_INTERNAL_ERROR, "Unknown internal exception.");
        return CS2TL_INTERNAL_ERROR;
    }
}
template <class T> void sized(const T *value) {
    tl::require(value && value->struct_size >= sizeof(T), CS2TL_INVALID_ARGUMENT,
                "Missing structure or incompatible struct_size.");
}
const tl::Scene &checked(const cs2tl_scene *scene) {
    tl::require(scene != nullptr, CS2TL_INVALID_ARGUMENT, "Scene handle is required.");
    return scene->data;
}
tl::BuildOptions build_options(const cs2tl_build_options *value) {
    if (!value) {
        return {};
    }
    sized(value);
    tl::require((value->flags & ~(CS2TL_BUILD_ALL_COLLISION_GROUPS | CS2TL_BUILD_APPROXIMATE_HULLS |
                                  CS2TL_BUILD_IGNORE_BREAKABLES)) == 0 &&
                    value->reserved == 0,
                CS2TL_INVALID_ARGUMENT, "Invalid build options.");
    return {value->flags, value->leaf_size == 0 ? 4u : value->leaf_size};
}
tl::Ray ray(const cs2tl_segment *value) {
    sized(value);
    return tl::make_ray(*value);
}
const char *path(const char *value) {
    tl::require(value && *value, CS2TL_INVALID_ARGUMENT, "Path is required.");
    return value;
}
std::vector<tl::PhysicsSource> physics_sources(const cs2tl_physics_source *sources, uint32_t count) {
    tl::require(sources && count, CS2TL_INVALID_ARGUMENT, "Explicit physics sources are required.");
    std::vector<tl::PhysicsSource> result;
    result.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        const auto &source = sources[i];
        sized(&source);
        tl::require((source.flags & ~CS2TL_SOURCE_BREAKABLE) == 0, CS2TL_INVALID_ARGUMENT,
                    "Unknown physics source flags.");
        tl::PhysicsSource item;
        item.path = tl::utf8_path(path(source.path));
        item.flags = source.flags;
        for (size_t j = 0; j < 12; ++j) {
            tl::require(std::isfinite(source.transform[j]), CS2TL_INVALID_ARGUMENT,
                        "Nonfinite source transform.");
            item.transform[j] = source.transform[j];
        }
        const auto &m = item.transform;
        const double determinant = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8]) +
                                   m[2] * (m[4] * m[9] - m[5] * m[8]);
        tl::require(determinant != 0, CS2TL_INVALID_ARGUMENT, "Singular source transform.");
        result.push_back(std::move(item));
    }
    return result;
}
template <class T>
void copy_result(std::span<const T> values, T *buffer, uint32_t capacity, uint32_t *count) {
    tl::require(count != nullptr, CS2TL_INVALID_ARGUMENT, "Result count is required.");
    tl::require(buffer != nullptr || capacity == 0, CS2TL_INVALID_ARGUMENT,
                "Null buffer with nonzero capacity.");
    *count = static_cast<uint32_t>(values.size());
    if (!buffer && capacity == 0) {
        return;
    }
    tl::require(capacity >= values.size(), CS2TL_BUFFER_TOO_SMALL,
                "Result buffer is too small; required count was returned.");
    std::copy(values.begin(), values.end(), buffer);
}
} // namespace
uint32_t CS2TL_CALL cs2tl_get_abi_version(void) {
    return CS2TL_ABI_VERSION;
}
void CS2TL_CALL cs2tl_set_warning_callback(cs2tl_warning_callback callback, void *user) {
    tl::warning_callback = callback;
    tl::warning_user = user;
}
cs2tl_status CS2TL_CALL cs2tl_import_sources(const cs2tl_physics_source *sources, uint32_t count,
                                             const cs2tl_build_options *options, cs2tl_scene **out,
                                             cs2tl_error *error) {
    if (out) {
        *out = nullptr;
    }
    return guarded(error, [&] {
        tl::require(out != nullptr, CS2TL_INVALID_ARGUMENT, "Output scene pointer is required.");
        const auto inputs = physics_sources(sources, count);
        auto scene =
            std::make_unique<cs2tl_scene>(tl::import_physics_sources(inputs, build_options(options)));
        *out = scene.release();
    });
}
cs2tl_status CS2TL_CALL cs2tl_build_cache_sources(const cs2tl_physics_source *sources, uint32_t count,
                                                  const char *output, const cs2tl_build_options *options,
                                                  cs2tl_error *error) {
    return guarded(error, [&] {
        const auto inputs = physics_sources(sources, count);
        const auto out = tl::utf8_path(path(output));
        for (const auto &input : inputs) {
            std::error_code ec;
            tl::require(!std::filesystem::equivalent(input.path, out, ec) &&
                            std::filesystem::weakly_canonical(input.path) !=
                                std::filesystem::weakly_canonical(out),
                        CS2TL_INVALID_ARGUMENT, "Input and output paths must differ.");
        }
        const auto scene = tl::import_physics_sources(inputs, build_options(options));
        tl::save_cache(scene, out);
    });
}
cs2tl_status CS2TL_CALL cs2tl_build_cache(const char *input, const char *output,
                                          const cs2tl_build_options *options, cs2tl_error *error) {
    return guarded(error, [&] {
        const auto in = tl::utf8_path(path(input)), out = tl::utf8_path(path(output));
        std::error_code equivalent_error;
        const bool equivalent = std::filesystem::equivalent(in, out, equivalent_error);
        tl::require(!equivalent &&
                        std::filesystem::weakly_canonical(in) != std::filesystem::weakly_canonical(out),
                    CS2TL_INVALID_ARGUMENT, "Input and output paths must differ.");
        const auto scene = tl::import_physics(in, build_options(options));
        tl::save_cache(scene, out);
    });
}
cs2tl_status CS2TL_CALL cs2tl_load_scene(const char *input, cs2tl_scene **out, cs2tl_error *error) {
    if (out) {
        *out = nullptr;
    }
    return guarded(error, [&] {
        tl::require(out != nullptr, CS2TL_INVALID_ARGUMENT, "Output scene pointer is required.");
        auto scene = std::make_unique<cs2tl_scene>(tl::load_cache(tl::utf8_path(path(input))));
        *out = scene.release();
    });
}
cs2tl_status CS2TL_CALL cs2tl_import_scene(const char *input, const cs2tl_build_options *options,
                                           cs2tl_scene **out, cs2tl_error *error) {
    if (out) {
        *out = nullptr;
    }
    return guarded(error, [&] {
        tl::require(out != nullptr, CS2TL_INVALID_ARGUMENT, "Output scene pointer is required.");
        auto scene = std::make_unique<cs2tl_scene>(
            tl::import_physics(tl::utf8_path(path(input)), build_options(options)));
        *out = scene.release();
    });
}
cs2tl_status CS2TL_CALL cs2tl_create_scene(const cs2tl_triangle *triangles, uint32_t count,
                                           const cs2tl_build_options *options, cs2tl_scene **out,
                                           cs2tl_error *error) {
    if (out) {
        *out = nullptr;
    }
    return guarded(error, [&] {
        tl::require(out && (triangles || count == 0), CS2TL_INVALID_ARGUMENT,
                    "Invalid triangle input or scene output.");
        const auto config = build_options(options);
        std::vector<tl::Triangle> decoded;
        decoded.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            const auto &t = triangles[i];
            decoded.push_back({tl::Vec3(t.a), tl::Vec3(t.b), tl::Vec3(t.c), t.surface_id, t.object_id});
        }
        auto scene = std::make_unique<cs2tl_scene>(tl::build_scene(std::move(decoded), config));
        *out = scene.release();
    });
}
cs2tl_status CS2TL_CALL cs2tl_save_scene(const cs2tl_scene *scene, const char *output, cs2tl_error *error) {
    return guarded(error, [&] { tl::save_cache(checked(scene), tl::utf8_path(path(output))); });
}
void CS2TL_CALL cs2tl_destroy_scene(cs2tl_scene *scene) {
    delete scene;
}
cs2tl_status CS2TL_CALL cs2tl_get_scene_info(const cs2tl_scene *scene, cs2tl_scene_info *info,
                                             cs2tl_error *error) {
    return guarded(error, [&] {
        const auto &data = checked(scene);
        sized(info);
        const auto size = info->struct_size;
        *info = {size,
                 data.flags,
                 static_cast<uint32_t>(data.triangles.size()),
                 static_cast<uint32_t>(data.solids.size()),
                 static_cast<uint32_t>(std::count_if(data.solids.begin(), data.solids.end(),
                                                     [](const auto &s) { return s.closed; })),
                 static_cast<uint32_t>(data.nodes.size()),
                 data.fingerprint};
    });
}
cs2tl_status CS2TL_CALL cs2tl_get_surface_hash(const cs2tl_scene *scene, int32_t id, uint32_t *hash,
                                               cs2tl_error *error) {
    if (hash) {
        *hash = 0;
    }
    return guarded(error, [&] {
        const auto &data = checked(scene);
        tl::require(hash && id >= 0 && static_cast<size_t>(id) < data.surface_hashes.size(),
                    CS2TL_INVALID_ARGUMENT, "Surface hash is unavailable or index is invalid.");
        *hash = data.surface_hashes[static_cast<size_t>(id)];
    });
}
cs2tl_status CS2TL_CALL cs2tl_is_occluded(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                          uint32_t *out, cs2tl_error *error) {
    if (out) {
        *out = 0;
    }
    return guarded(error, [&] {
        const auto &data = checked(scene);
        const auto query = ray(segment);
        tl::require(out != nullptr, CS2TL_INVALID_ARGUMENT, "Occlusion output is required.");
        *out = tl::is_occluded(data, query, scene->workspaces) ? 1u : 0u;
    });
}
cs2tl_status CS2TL_CALL cs2tl_trace_closest(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                            cs2tl_hit *hit, uint32_t *found, cs2tl_error *error) {
    if (found) {
        *found = 0;
    }
    if (hit) {
        *hit = {};
    }
    return guarded(error, [&] {
        const auto &data = checked(scene);
        const auto query = ray(segment);
        tl::require(hit && found, CS2TL_INVALID_ARGUMENT, "Hit and found outputs are required.");
        if (const auto nearest = tl::trace_closest(data, query, scene->workspaces)) {
            *hit = tl::external_hit(data, query, *nearest);
            *found = 1;
        }
    });
}
cs2tl_status CS2TL_CALL cs2tl_trace_all(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                        cs2tl_hit *buffer, uint32_t capacity, uint32_t *count,
                                        cs2tl_error *error) {
    if (count) {
        *count = 0;
    }
    return guarded(error, [&] {
        const auto &data = checked(scene);
        const auto query = ray(segment);
        tl::require(count && (buffer || capacity == 0), CS2TL_INVALID_ARGUMENT,
                    "Invalid result buffer/count.");
        tl::QueryLease lease(scene->workspaces);
        const auto hits = tl::trace_all_into(data, query, lease.get());
        *count = static_cast<uint32_t>(hits.size());
        if (!buffer && capacity == 0) {
            return;
        }
        tl::require(capacity >= hits.size(), CS2TL_BUFFER_TOO_SMALL,
                    "Result buffer is too small; required count was returned.");
        if (data.primitives.empty()) {
            for (size_t i = 0; i < hits.size(); ++i) {
                buffer[i] = tl::external_hit<false>(data, query, hits[i]);
            }
        } else {
            for (size_t i = 0; i < hits.size(); ++i) {
                buffer[i] = tl::external_hit(data, query, hits[i]);
            }
        }
    });
}
cs2tl_status CS2TL_CALL cs2tl_query_intervals(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                              cs2tl_interval *buffer, uint32_t capacity, uint32_t *count,
                                              cs2tl_error *error) {
    if (count) {
        *count = 0;
    }
    return guarded(error, [&] {
        const auto &data = checked(scene);
        const auto query = ray(segment);
        tl::require(count && (buffer || capacity == 0), CS2TL_INVALID_ARGUMENT,
                    "Invalid interval buffer/count.");
        tl::QueryLease lease(scene->workspaces);
        copy_result(tl::query_intervals_into(data, query, lease.get()), buffer, capacity, count);
    });
}
cs2tl_status CS2TL_CALL cs2tl_trace_all_to(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                           cs2tl_allocate_hits_fn allocate, void *user, cs2tl_error *error) {
    return guarded(error, [&] {
        const auto &data = checked(scene);
        const auto query = ray(segment);
        tl::require(allocate != nullptr, CS2TL_INVALID_ARGUMENT, "Result allocator is required.");
        tl::QueryLease lease(scene->workspaces);
        const auto hits = tl::trace_all_into(data, query, lease.get());
        auto *buffer = allocate(static_cast<uint32_t>(hits.size()), user);
        tl::require(buffer || hits.empty(), CS2TL_OUT_OF_MEMORY, "Result allocation failed.");
        if (data.primitives.empty()) {
            for (size_t i = 0; i < hits.size(); ++i) {
                buffer[i] = tl::external_hit<false>(data, query, hits[i]);
            }
        } else {
            for (size_t i = 0; i < hits.size(); ++i) {
                buffer[i] = tl::external_hit(data, query, hits[i]);
            }
        }
    });
}
cs2tl_status CS2TL_CALL cs2tl_query_intervals_to(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                                 cs2tl_allocate_intervals_fn allocate, void *user,
                                                 cs2tl_error *error) {
    return guarded(error, [&] {
        const auto &data = checked(scene);
        const auto query = ray(segment);
        tl::require(allocate != nullptr, CS2TL_INVALID_ARGUMENT, "Result allocator is required.");
        tl::QueryLease lease(scene->workspaces);
        const auto intervals = tl::query_intervals_into(data, query, lease.get());
        auto *buffer = allocate(static_cast<uint32_t>(intervals.size()), user);
        tl::require(buffer || intervals.empty(), CS2TL_OUT_OF_MEMORY, "Result allocation failed.");
        if (!intervals.empty()) {
            std::copy(intervals.begin(), intervals.end(), buffer);
        }
    });
}
cs2tl_status CS2TL_CALL cs2tl_release_query_memory(cs2tl_scene *scene, cs2tl_error *error) {
    return guarded(error, [&] {
        (void)checked(scene);
        scene->workspaces.release_memory();
    });
}
cs2tl_status CS2TL_CALL cs2tl_simulate_shot(const cs2tl_scene *scene, const cs2tl_shot_request *request,
                                            cs2tl_shot_result *result, cs2tl_error *error) {
    return guarded(error, [&] {
        const auto &data = checked(scene);
        sized(request);
        sized(result);
        tl::simulate_shot(data, *request, *result);
    });
}
