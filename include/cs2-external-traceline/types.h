#pragma once
#include <stdint.h>

#define CS2TL_ABI_VERSION 1u
/* Preferred collision scene cache suffix. Loading validates the file contents,
 * so existing caches remain readable regardless of their filename extension. */
#define CS2TL_SCENE_CACHE_EXTENSION ".cs2tl"
#define CS2TL_INVALID_ID UINT32_MAX
#define CS2TL_UNKNOWN_SURFACE (-1)

typedef int32_t cs2tl_status;
enum {
    CS2TL_OK = 0,
    CS2TL_INVALID_ARGUMENT = 1,
    CS2TL_IO_ERROR = 2,
    CS2TL_PARSE_ERROR = 3,
    CS2TL_INVALID_GEOMETRY = 4,
    CS2TL_UNSUPPORTED_FORMAT = 5,
    CS2TL_CORRUPT_CACHE = 6,
    CS2TL_BUFFER_TOO_SMALL = 7,
    CS2TL_OUT_OF_MEMORY = 8,
    CS2TL_NOT_IMPLEMENTED = 9,
    CS2TL_INTERNAL_ERROR = 10,
    CS2TL_INCOMPLETE_GEOMETRY = 11
};

typedef struct cs2tl_scene cs2tl_scene;
typedef struct cs2tl_vec3 {
    float x, y, z;
} cs2tl_vec3;

typedef struct cs2tl_error {
    uint32_t struct_size;
    cs2tl_status code;
    uint64_t offset;   /* Byte offset in input, or UINT64_MAX when unavailable. */
    char message[384]; /* UTF-8; copied into caller-owned memory. */
} cs2tl_error;

enum {
    CS2TL_BUILD_ALL_COLLISION_GROUPS = 1u << 0,
    CS2TL_BUILD_APPROXIMATE_HULLS = 1u << 1,
    CS2TL_BUILD_IGNORE_BREAKABLES = 1u << 2
};
typedef struct cs2tl_build_options {
    uint32_t struct_size;
    uint32_t flags;
    uint32_t leaf_size; /* 0 selects 4; values above 64 produce a performance warning. */
    uint32_t reserved;
} cs2tl_build_options;
enum { CS2TL_SOURCE_BREAKABLE = 1u << 0 };
/* Explicit independent PHYS text input. Classify BREAKABLE only from reliable
 * entity/model metadata. Unknown sources remain present. The transform is
 * row-major affine 3x4; identity is {1,0,0,0,0,1,0,0,0,0,1,0}. */
typedef struct cs2tl_physics_source {
    uint32_t struct_size;
    uint32_t flags;
    const char *path;
    float transform[12];
} cs2tl_physics_source;

typedef struct cs2tl_triangle {
    cs2tl_vec3 a, b, c;
    int32_t surface_id;
    uint32_t object_id; /* Triangles of one source object share this identifier. */
} cs2tl_triangle;

enum { CS2TL_SCENE_APPROXIMATE = 1u << 0, CS2TL_SCENE_HAS_OPEN_GEOMETRY = 1u << 1 };
typedef struct cs2tl_scene_info {
    uint32_t struct_size;
    uint32_t flags;
    uint32_t triangle_count;
    uint32_t solid_count;
    uint32_t closed_solid_count;
    uint32_t bvh_node_count;
    uint64_t source_fingerprint;
} cs2tl_scene_info;

typedef struct cs2tl_segment {
    uint32_t struct_size;
    uint32_t flags; /* Reserved, must be zero. */
    cs2tl_vec3 start, end;
    float endpoint_epsilon; /* World units, excludes this distance at BOTH endpoints. */
    float merge_epsilon;    /* World units, only merges coincident hits of one solid. */
} cs2tl_segment;

enum { CS2TL_HIT_ENTER = 1u, CS2TL_HIT_EXIT = 2u, CS2TL_HIT_TANGENT = 3u };
typedef struct cs2tl_hit {
    float distance;
    float fraction;
    cs2tl_vec3 position;
    cs2tl_vec3 normal;
    int32_t surface_id;
    uint32_t object_id;
    uint32_t solid_id;
    uint32_t triangle_id; /* CS2TL_INVALID_ID for analytic sphere/capsule hits. */
    uint32_t kind;
} cs2tl_hit;

enum { CS2TL_INTERVAL_START_INSIDE = 1u << 0, CS2TL_INTERVAL_END_INSIDE = 1u << 1 };
typedef struct cs2tl_interval {
    float entry_distance, exit_distance, thickness;
    uint32_t flags;
    cs2tl_vec3 entry_position, exit_position;
    int32_t entry_surface_id, exit_surface_id;
    uint32_t object_id, solid_id;
} cs2tl_interval;

/* Ballistics ABI reservation. No damage formula is implemented in ABI v1. */
typedef struct cs2tl_weapon_profile {
    uint32_t struct_size;
    uint32_t weapon_id;
    uint32_t ammo_id;
    uint32_t fire_mode;
    float base_damage;
    float max_range;
    float range_modifier;
    float penetration_power;
    float armor_ratio;
    float headshot_multiplier;
    uint32_t pellets_per_shot;
    uint32_t data_version;
} cs2tl_weapon_profile;

typedef struct cs2tl_surface_profile {
    uint32_t struct_size;
    int32_t surface_id;
    float penetration_modifier;
    float damage_modifier;
    uint32_t flags;
    uint32_t data_version;
} cs2tl_surface_profile;

typedef struct cs2tl_target_profile {
    uint32_t struct_size;
    uint32_t hit_group;
    uint32_t armor;
    uint32_t has_helmet;
} cs2tl_target_profile;

typedef struct cs2tl_shot_request {
    uint32_t struct_size;
    uint32_t ruleset_version;
    cs2tl_segment segment;
    const cs2tl_weapon_profile *weapon;
    const cs2tl_surface_profile *surfaces;
    uint32_t surface_count;
    uint32_t reserved;
    const cs2tl_target_profile *target; /* Optional. */
} cs2tl_shot_request;

enum { CS2TL_SHOT_NOT_COMPUTED = 0u, CS2TL_SHOT_REACHED = 1u, CS2TL_SHOT_STOPPED = 2u };
typedef struct cs2tl_shot_result {
    uint32_t struct_size;
    uint32_t outcome;
    float remaining_damage;
    float target_damage;
    uint32_t penetration_count;
    uint32_t stop_reason;
    uint32_t ruleset_version;
    uint32_t reserved;
} cs2tl_shot_result;
