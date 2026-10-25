#pragma once
#include "types.h"

#if defined(_WIN32)
#if defined(CS2TL_BUILD_DLL)
#define CS2TL_API __declspec(dllexport)
#else
#define CS2TL_API __declspec(dllimport)
#endif
#define CS2TL_CALL __cdecl
#else
#define CS2TL_API
#define CS2TL_CALL
#endif
#ifdef __cplusplus
extern "C" {
#endif

/* All strings are UTF-8. All optional error outputs must set struct_size first.
 * Output handles are NULL on failure; the DLL owns and destroys their storage.
 * Scenes support concurrent reads. Never destroy a scene during a query.
 * Options may be NULL for defaults. No API retains caller-owned input pointers.
 * Query scratch belongs to each scene and is reused across queries. Concurrent
 * and reentrant calls use separate workspaces. Destroying the scene releases
 * all of them. No query history or owning thread-local scene cache is kept. */
CS2TL_API uint32_t CS2TL_CALL cs2tl_get_abi_version(void);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_build_cache(const char *input_path, const char *output_path,
                                                    const cs2tl_build_options *options, cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_load_scene(const char *cache_path, cs2tl_scene **out_scene,
                                                   cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_import_scene(const char *input_path,
                                                     const cs2tl_build_options *options,
                                                     cs2tl_scene **out_scene, cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_create_scene(const cs2tl_triangle *triangles, uint32_t triangle_count,
                                                     const cs2tl_build_options *options,
                                                     cs2tl_scene **out_scene, cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_save_scene(const cs2tl_scene *scene, const char *cache_path,
                                                   cs2tl_error *error);
CS2TL_API void CS2TL_CALL cs2tl_destroy_scene(cs2tl_scene *scene);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_scene_info(const cs2tl_scene *scene, cs2tl_scene_info *info,
                                                       cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_surface_hash(const cs2tl_scene *scene, int32_t surface_id,
                                                         uint32_t *out_hash, cs2tl_error *error);

/* Invalid segments return a status, never a successful miss. Endpoints are open.
 * Closest/all report surface crossings; is_occluded also detects closed solids
 * containing the segment. Surface contact blocks; coplanar triangles are ignored.
 * Nested shells of one source object can return INCOMPLETE_GEOMETRY rather than
 * incorrectly treating an ambiguous cavity as filled. No weapon penetration is evaluated. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_is_occluded(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                                    uint32_t *out_occluded, cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_trace_closest(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                                      cs2tl_hit *hit, uint32_t *out_found,
                                                      cs2tl_error *error);

/* Two-call buffer protocol: buffer=NULL, capacity=0 obtains count and returns OK.
 * Insufficient capacity returns BUFFER_TOO_SMALL and required count, writes NO
 * elements. Counts are deterministic on an unchanged scene and query.
 * All hits are sorted and coincident hits within one solid are consolidated.
 * Different solids at the same distance remain distinct. merge_epsilon is an
 * upper bound on numerical coincidence; distinct thin layers are never merged
 * just because their distance is below this caller-supplied tolerance. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_trace_all(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                                  cs2tl_hit *hits, uint32_t capacity, uint32_t *out_count,
                                                  cs2tl_error *error);

/* Exact intervals require closed, orientable geometry; open/nonmanifold geometry
 * on the query path returns INCOMPLETE_GEOMETRY without fabricated thickness.
 * Same-object nested component bounds are conservatively rejected as ambiguous
 * cavities. Intervals of overlapping independent solids remain separate. Closed solids are assumed
 * simple (not self-intersecting); the importer validates edge topology only. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_query_intervals(const cs2tl_scene *scene,
                                                        const cs2tl_segment *segment,
                                                        cs2tl_interval *intervals, uint32_t capacity,
                                                        uint32_t *out_count, cs2tl_error *error);

/* Single-computation allocation. The allocator is called exactly once after a
 * successful complete query, including zero results. It must not throw and must
 * return storage for count elements, or NULL on allocation failure. NULL is valid
 * for zero results. Caller owns the storage; no input/output pointers are kept.
 * Reentrant queries are allowed. Never destroy/reset this scene in the callback.
 * Geometry errors do not call the allocator. */
typedef cs2tl_hit *(CS2TL_CALL *cs2tl_allocate_hits_fn)(uint32_t count, void *user);
typedef cs2tl_interval *(CS2TL_CALL *cs2tl_allocate_intervals_fn)(uint32_t count, void *user);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_trace_all_to(const cs2tl_scene *scene, const cs2tl_segment *segment,
                                                     cs2tl_allocate_hits_fn allocate, void *user,
                                                     cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_query_intervals_to(const cs2tl_scene *scene,
                                                           const cs2tl_segment *segment,
                                                           cs2tl_allocate_intervals_fn allocate, void *user,
                                                           cs2tl_error *error);

/* Optional release after a large batch. Finish ALL scene queries first, as for
 * destroy_scene. Geometry remains loaded and subsequent queries remain valid. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_release_query_memory(cs2tl_scene *scene, cs2tl_error *error);

/* Reserved: validates the ABI and returns NOT_IMPLEMENTED, result outcome stays
 * NOT_COMPUTED. remaining_damage/target_damage are NaN, never a damage estimate. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_simulate_shot(const cs2tl_scene *scene,
                                                      const cs2tl_shot_request *request,
                                                      cs2tl_shot_result *result, cs2tl_error *error);

#ifdef __cplusplus
}
#endif
