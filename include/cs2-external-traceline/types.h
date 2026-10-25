#pragma once
#include <stdint.h>

#define CS2TL_ABI_VERSION 1u
/* Preferred collision scene cache suffix. Loading validates the file contents,
 * so existing caches remain readable regardless of their filename extension. */
#define CS2TL_SCENE_CACHE_EXTENSION ".cs2tl"
#define CS2TL_PROFILES_EXTENSION ".cs2tlp"
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
    CS2TL_INCOMPLETE_GEOMETRY = 11,
    CS2TL_DATA_NOT_FOUND = 12,
    CS2TL_CORRUPT_PROFILES = 13
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

/* One bullet/pellet. weapon_id is the game item definition index (7 = AK-47).
 * ammo_id is the case-insensitive CUtlStringToken of the resource ammo name,
 * not the ordinal of an in-process engine enum. */
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

typedef struct cs2tl_profiles cs2tl_profiles;
typedef struct cs2tl_ballistics cs2tl_ballistics;
enum { CS2TL_RULESET_CS2_STATIC_V1 = 1u };
enum { CS2TL_FIRE_PRIMARY = 0u, CS2TL_FIRE_ALTERNATE = 1u };
enum { CS2TL_PROFILES_NOT_LIVE_VALIDATED = 1u, CS2TL_PROFILES_BUILD_MISMATCH = 2u };
typedef struct cs2tl_profiles_info {
    uint32_t struct_size, format_version, client_version, ruleset_version;
    uint64_t steam_build_id, source_fingerprint;
    uint32_t weapon_profile_count, surface_count, flags, reserved;
} cs2tl_profiles_info;
/* Flags describe engine interaction metadata supplied by the caller. */
enum { CS2TL_MATERIAL_NATIVE_FLAG13 = 1u };
typedef struct cs2tl_material_profile {
    uint32_t struct_size, surface_hash, game_material, flags;
    float penetration_distance_modifier, penetration_damage_modifier;
    uint32_t data_version, reserved;
} cs2tl_material_profile;
typedef struct cs2tl_ballistics_info {
    uint32_t struct_size, flags, scene_surface_count, resolved_surface_count;
    uint32_t unknown_surface_count, reserved;
    uint64_t scene_fingerprint;
} cs2tl_ballistics_info;
typedef struct cs2tl_remaining_damage_request {
    uint32_t struct_size, weapon_id, fire_mode, reserved;
    cs2tl_segment segment;
} cs2tl_remaining_damage_request;
enum {
    CS2TL_STOP_NONE = 0u,
    CS2TL_STOP_RANGE = 1u,
    CS2TL_STOP_PENETRATION_POWER = 2u,
    CS2TL_STOP_MATERIAL = 3u,
    CS2TL_STOP_DAMAGE = 4u,
    CS2TL_STOP_PENETRATION_COUNT = 5u,
    CS2TL_STOP_PENETRATION_DISTANCE = 6u
};
enum {
    CS2TL_PREDICTION_STATIC_SCENE = 1u,
    CS2TL_PREDICTION_BINARY_RULES_AUDITED = 2u,
    CS2TL_PREDICTION_NOT_LIVE_VALIDATED = 4u,
    CS2TL_PREDICTION_BUILD_MISMATCH = 8u,
    /* Scene v1/v2 do not preserve native per-hit interaction bit 13. Its
     * special override is unavailable under the default intact-static policy. */
    CS2TL_PREDICTION_INTERACTION_FLAGS_UNAVAILABLE = 16u
};
typedef struct cs2tl_shot_prediction {
    uint32_t struct_size, outcome;
    float remaining_damage;
    uint32_t penetration_count, stop_reason, ruleset_version, flags, reserved;
} cs2tl_shot_prediction;

enum {
    CS2TL_HITGROUP_GENERIC = 0u,
    CS2TL_HITGROUP_HEAD = 1u,
    CS2TL_HITGROUP_CHEST = 2u,
    CS2TL_HITGROUP_STOMACH = 3u,
    CS2TL_HITGROUP_LEFT_ARM = 4u,
    CS2TL_HITGROUP_RIGHT_ARM = 5u,
    CS2TL_HITGROUP_LEFT_LEG = 6u,
    CS2TL_HITGROUP_RIGHT_LEG = 7u,
    CS2TL_HITGROUP_NECK = 8u,
    CS2TL_HITGROUP_UNUSED = 9u,
    CS2TL_HITGROUP_GEAR = 10u,
    CS2TL_HITGROUP_SPECIAL = 11u
};
enum { CS2TL_TEAM_T = 2u, CS2TL_TEAM_CT = 3u };
enum { CS2TL_TARGET_HAS_HELMET = 1u, CS2TL_TARGET_IMMUNE = 2u, CS2TL_TARGET_ACCUMULATOR_KNOWN = 4u };
/* A snapshot of another player at the supplied impact. The caller determines
 * hit_group and immunity. Fractional damage is server state, not derivable from
 * integer health. With ACCUMULATOR_KNOWN, damage_accumulator is in [0,1).
 * Otherwise it is ignored and the result reports conservative health bounds. */
typedef struct cs2tl_target_state {
    uint32_t struct_size, hit_group;
    int32_t health, armor;
    uint32_t team, flags;
    float damage_accumulator;
    uint32_t reserved;
} cs2tl_target_state;
enum { CS2TL_DAMAGE_RULES_LIVE_SNAPSHOT = 1u };
typedef struct cs2tl_damage_rules {
    uint32_t struct_size, flags;
    uint64_t steam_build_id;
    float ct_head_scale, ct_body_scale, t_head_scale, t_body_scale;
    float friendly_bullet_scale;
    uint32_t friendly_fire, teammates_are_enemies, headshots_only;
} cs2tl_damage_rules;
typedef struct cs2tl_target_damage_request {
    uint32_t struct_size, weapon_id, fire_mode, attacker_team;
    float incoming_damage;
    uint32_t reserved;
    const cs2tl_target_state *target;
    const cs2tl_damage_rules *rules; /* NULL selects explicitly flagged defaults. */
} cs2tl_target_damage_request;
enum { CS2TL_DAMAGE_NOT_COMPUTED = 0u, CS2TL_DAMAGE_EVALUATED = 1u, CS2TL_DAMAGE_BLOCKED = 2u };
enum {
    CS2TL_DAMAGE_BLOCK_NONE = 0u,
    CS2TL_DAMAGE_BLOCK_DEAD = 1u,
    CS2TL_DAMAGE_BLOCK_IMMUNE = 2u,
    CS2TL_DAMAGE_BLOCK_FRIENDLY_FIRE = 3u,
    CS2TL_DAMAGE_BLOCK_HEADSHOTS_ONLY = 4u,
    CS2TL_DAMAGE_BLOCK_PROJECTILE_STOPPED = 5u
};
enum { CS2TL_PREDICTION_ASSUMED_DAMAGE_RULES = 32u, CS2TL_PREDICTION_UNKNOWN_DAMAGE_ACCUMULATOR = 64u };
/* health_damage/armor_damage preserve float32 damage before integer accounting.
 * Health bounds are equal for an exact result. lethal is guaranteed; a boundary
 * depending on unknown fractional state sets possible_lethal instead. Accumulator
 * after is NaN when its input is unknown. Errors use NaN, UINT32_MAX loss bounds
 * and -1 remaining values; always check status/outcome before reading fields. */
typedef struct cs2tl_target_damage_result {
    uint32_t struct_size, outcome;
    float incoming_damage, scaled_damage, health_damage, armor_damage;
    float damage_accumulator_after;
    uint32_t armor_loss;
    int32_t armor_after;
    uint32_t health_loss_min, health_loss_max;
    int32_t health_after_min, health_after_max;
    uint32_t lethal, possible_lethal, block_reason, ruleset_version, flags;
} cs2tl_target_damage_result;
typedef struct cs2tl_shot_damage_request {
    uint32_t struct_size, weapon_id, fire_mode, attacker_team;
    cs2tl_segment segment;
    const cs2tl_target_state *target;
    const cs2tl_damage_rules *rules;
} cs2tl_shot_damage_request;
typedef struct cs2tl_shot_damage_result {
    uint32_t struct_size, reserved;
    cs2tl_shot_prediction flight;
    cs2tl_target_damage_result target;
} cs2tl_shot_damage_result;
