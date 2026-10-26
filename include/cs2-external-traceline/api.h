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
 * all of them. No query history or owning thread-local scene cache is kept.
 * Cache paths are explicit. Use CS2TL_SCENE_CACHE_EXTENSION for new collision
 * scene caches (geometry, surface identifiers and acceleration data). Loading
 * checks the container header/payload, not the extension; earlier .bvh caches
 * with a supported CS2TLBVH header remain compatible. */
CS2TL_API uint32_t CS2TL_CALL cs2tl_get_abi_version(void);
/* Per calling thread. NULL restores debugger output. The message is borrowed
 * for the callback duration; no warning history is retained by the DLL. */
typedef void(CS2TL_CALL *cs2tl_warning_callback)(void *user, const char *message, uint64_t offset);
CS2TL_API void CS2TL_CALL cs2tl_set_warning_callback(cs2tl_warning_callback callback, void *user);
/* No directory lookup or implicit companion files. All sources and transforms
 * are supplied by the caller. Default: breakables are intact and permanent. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_import_sources(const cs2tl_physics_source *sources, uint32_t count,
                                                       const cs2tl_build_options *options,
                                                       cs2tl_scene **out_scene, cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_build_cache_sources(const cs2tl_physics_source *sources,
                                                            uint32_t count, const char *output_path,
                                                            const cs2tl_build_options *options,
                                                            cs2tl_error *error);
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

/* Exact volume intervals require closed, orientable geometry. Physics imports
 * retain explicit native m_meshes provenance (cache v3). An open native surface
 * component without a surface hit contributes no interval; an actual hit still
 * returns INCOMPLETE_GEOMETRY without fabricated thickness. Raw triangles and
 * v1/v2 caches have no such provenance: overlapping untyped open-component bounds
 * remain incomplete. Rebuild old native caches from explicit physics-text paths.
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

/* Explicit-profile compatibility entry point for one bullet. ruleset_version=0
 * selects STATIC_V1. surface flags encode game_material in bits 0..15 and
 * NATIVE_FLAG13 in bit 16. target must be NULL: player health damage is not
 * evaluated; target_damage remains NaN. Prefer a bound ballistics context for
 * repeated queries. With a valid, correctly sized result, errors leave damage
 * NaN, never a successful zero. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_simulate_shot(const cs2tl_scene *scene,
                                                      const cs2tl_shot_request *request,
                                                      cs2tl_shot_result *result, cs2tl_error *error);

/* Explicit, immutable external parameter package; no directory discovery.
 * Profiles are loaded/validated once. Game-build differences produce warnings
 * and prediction flags. Binary/profile format errors remain errors. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_load_profiles(const char *path, cs2tl_profiles **out,
                                                      cs2tl_error *error);
CS2TL_API void CS2TL_CALL cs2tl_destroy_profiles(cs2tl_profiles *profiles);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_profiles_info(const cs2tl_profiles *profiles,
                                                          cs2tl_profiles_info *out, cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_weapon_profile(const cs2tl_profiles *profiles, uint32_t weapon_id,
                                                           uint32_t fire_mode, cs2tl_weapon_profile *out,
                                                           cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_weapon_profile_at(const cs2tl_profiles *profiles, uint32_t index,
                                                              cs2tl_weapon_profile *out, cs2tl_error *error);
/* Both name functions report bytes INCLUDING the UTF-8 NUL terminator. A
 * NULL buffer with capacity=0 queries the size; a short buffer is not written.
 * Names refer to the primary firing-mode record of the supplied weapon ID. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_weapon_name(const cs2tl_profiles *profiles, uint32_t weapon_id,
                                                        char *buffer, uint32_t capacity,
                                                        uint32_t *required_bytes, cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_weapon_ammo_name(const cs2tl_profiles *profiles,
                                                             uint32_t weapon_id, char *buffer,
                                                             uint32_t capacity, uint32_t *required_bytes,
                                                             cs2tl_error *error);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_material_profile(const cs2tl_profiles *profiles,
                                                             uint32_t surface_hash,
                                                             cs2tl_material_profile *out, cs2tl_error *error);
/* Resolves material indices once. The context retains scene and profile data,
 * so their public handles may be destroyed after binding. Finish all context
 * queries before destroying it. No per-query I/O occurs. Each reusable native
 * workspace may retain exactly one immutable flight result; memory does not
 * grow with query history, and release_ballistics_memory discards it. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_create_ballistics(const cs2tl_scene *scene,
                                                          const cs2tl_profiles *profiles,
                                                          cs2tl_ballistics **out, cs2tl_error *error);
CS2TL_API void CS2TL_CALL cs2tl_destroy_ballistics(cs2tl_ballistics *context);
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_get_ballistics_info(const cs2tl_ballistics *context,
                                                            cs2tl_ballistics_info *out, cs2tl_error *error);
/* Quiesce all queries on the retained scene before releasing reusable scratch. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_release_ballistics_memory(cs2tl_ballistics *context,
                                                                  cs2tl_error *error);
/* One supplied A-to-B ray, per bullet/pellet, with no player damage or spread.
 * v4 caches with complete raw PHYS tags use native static filters, original
 * mesh winding/trees and hull planes, then event/span/loss rules. B must be a
 * certified air endpoint. Full static range is queried before B is sampled;
 * no player collider or missing wall thickness is fabricated. Audited source
 * transforms are currently identity; unsupported traversed colliders and
 * conflicting coincident materials return an error. Older caches and untyped
 * triangles retain the closed-volume path and its conservative error behavior.
 * Missing required materials and uncertain boundaries return NOT_COMPUTED/NaN
 * when out is valid and correctly sized. Query scratch is reused, not history.
 * Observe prediction flags for validation scope.
 * Engine stopping rules (including four penetrations) are game behavior, not
 * resource/import limits. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_calculate_remaining_damage(
    const cs2tl_ballistics *context, const cs2tl_remaining_damage_request *request,
    cs2tl_shot_prediction *out, cs2tl_error *error);

/* Defaults come from the binary-audited build and carry no LIVE_SNAPSHOT flag.
 * Replace fields with actual match values and set LIVE_SNAPSHOT when verified.
 * NULL rules select these same defaults; predictions mark assumed rules. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_default_damage_rules(cs2tl_damage_rules *out, cs2tl_error *error);
/* One ordinary bullet/pellet hitting another player. incoming_damage already
 * includes distance/penetration loss; this entry point performs no geometry or
 * file I/O. Snapshot pointers are borrowed only for this call. Inputs describe
 * the actual first target impact, not a bone center or a hit probability.
 * Unknown fractional damage state produces health-loss bounds, never a guessed
 * exact loss. Flags describe this target stage; retain the incoming flight's
 * flags separately, or use calculate_shot_damage to propagate both stages.
 * Native damage filters/plugins and other live bodies along the ray
 * are not modeled. Read flags for version, snapshot and validation scope. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_calculate_target_damage(const cs2tl_profiles *profiles,
                                                                const cs2tl_target_damage_request *request,
                                                                cs2tl_target_damage_result *out,
                                                                cs2tl_error *error);
/* Performs exactly one existing static-world penetration query, then the same
 * target calculation. segment.end must be the first actual target hit position.
 * A stopped projectile produces zero target damage and preserves target state.
 * Geometry/profile errors leave both results NOT_COMPUTED. All old APIs and
 * structures keep their existing meaning, including simulate_shot's NULL target. */
CS2TL_API cs2tl_status CS2TL_CALL cs2tl_calculate_shot_damage(const cs2tl_ballistics *context,
                                                              const cs2tl_shot_damage_request *request,
                                                              cs2tl_shot_damage_result *out,
                                                              cs2tl_error *error);

#ifdef __cplusplus
}
#endif
