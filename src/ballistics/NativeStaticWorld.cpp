#include "NativeStaticWorld.h"
#include "NativeMath.h"
#include "ShotSimulator.h"
#include "bvh/Bvh.h"
#include "diagnostics/Error.h"
#include "profiles/Ruleset.h"
#include <algorithm>
#include <bit>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <numeric>

namespace cs2::traceline {
namespace {
namespace f32 = native_f32;
bool identity(const PhysicsCollider &collider) {
    constexpr std::array<float, 12> unit{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    return collider.pose == unit;
}
Vec3 transform(const PhysicsCollider &collider, Vec3 point) {
    const auto &p = collider.pose;
    return {p[0] * point.x + p[1] * point.y + p[2] * point.z + p[3],
            p[4] * point.x + p[5] * point.y + p[6] * point.z + p[7],
            p[8] * point.x + p[9] * point.y + p[10] * point.z + p[11]};
}
Aabb bounds(const PhysicsCollider &collider) {
    Aabb local;
    if ((collider.flags & physics_bounds_available) != 0) {
        local = {Vec3(collider.bounds_min), Vec3(collider.bounds_max)};
    } else if (!collider.triangles.empty()) {
        for (const auto &triangle : collider.triangles) {
            local.extend(Vec3(triangle.a));
            local.extend(Vec3(triangle.b));
            local.extend(Vec3(triangle.c));
        }
    } else if (collider.shape == PhysicsShape::sphere || collider.shape == PhysicsShape::capsule) {
        local.extend(Vec3(collider.a));
        local.extend(Vec3(collider.b));
        const Vec3 radius{collider.radius, collider.radius, collider.radius};
        local.lo = local.lo - radius;
        local.hi = local.hi + radius;
    } else {
        // Missing bounds must not make an unsupported collider disappear.
        return {{-DBL_MAX, -DBL_MAX, -DBL_MAX}, {DBL_MAX, DBL_MAX, DBL_MAX}};
    }
    if (identity(collider)) {
        return local;
    }
    Aabb world;
    for (unsigned corner = 0; corner < 8; ++corner) {
        world.extend(
            transform(collider, {corner & 1 ? local.hi.x : local.lo.x, corner & 2 ? local.hi.y : local.lo.y,
                                 corner & 4 ? local.hi.z : local.lo.z}));
    }
    return world;
}
class IndexBuilder {
    NativeStaticWorld &world;
    uint32_t node(uint32_t first, uint32_t count) {
        require(world.nodes.size() < UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
                "Native outer index exceeds uint32 representation.");
        const auto index = static_cast<uint32_t>(world.nodes.size());
        world.nodes.emplace_back();
        Aabb box, centers;
        for (uint32_t i = 0; i < count; ++i) {
            const auto &b = world.colliders[world.order[first + i]].bounds;
            box.extend(b);
            centers.extend(b.center());
        }
        world.nodes[index].bounds = box;
        if (count <= 4) {
            world.nodes[index].first = first;
            world.nodes[index].count = count;
            return index;
        }
        const auto extent = centers.hi - centers.lo;
        const unsigned axis = extent.x >= extent.y && extent.x >= extent.z ? 0u
                              : extent.y >= extent.z                       ? 1u
                                                                           : 2u;
        const auto middle = first + count / 2;
        std::nth_element(world.order.begin() + first, world.order.begin() + middle,
                         world.order.begin() + first + count, [&](uint32_t a, uint32_t b) {
                             const auto ca = world.colliders[a].bounds.center()[axis];
                             const auto cb = world.colliders[b].bounds.center()[axis];
                             return ca != cb ? ca < cb : a < b;
                         });
        const auto left = node(first, count / 2), right = node(middle, count - count / 2);
        world.nodes[index].left = left;
        world.nodes[index].right = right;
        return index;
    }

  public:
    explicit IndexBuilder(NativeStaticWorld &value) : world(value) {
        require(world.colliders.size() <= UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
                "Native collider indices exceed uint32 representation.");
        world.order.resize(world.colliders.size());
        if (world.order.size() > 4) {
            const auto count = static_cast<uint32_t>(world.order.size());
            const auto width = std::bit_floor(count / 4);
            const uint64_t leaves =
                count / width == 4 ? uint64_t(width) + count % width : uint64_t(width) * 2;
            world.nodes.reserve(static_cast<size_t>(leaves * 2 - 1));
        }
        std::iota(world.order.begin(), world.order.end(), 0u);
        if (!world.order.empty()) {
            node(0, static_cast<uint32_t>(world.order.size()));
        }
    }
};
void append(NativeQueryContext &query, const NativeGeometryEvent &hit, uint32_t collider, uint64_t feature,
            int32_t surface) {
    BallisticEvent event;
    std::memcpy(event.bytes.data(), &hit.normal, sizeof(hit.normal));
    event.fraction(hit.fraction);
    const uint64_t source = collider;
    std::memcpy(event.bytes.data() + 24, &source, sizeof(source));
    std::memcpy(event.bytes.data() + 32, &feature, sizeof(feature));
    // Private wide surface slot; the span builder treats the payload opaquely.
    std::memcpy(event.bytes.data() + 40, &surface, sizeof(surface));
    event.kind(static_cast<uint8_t>(hit.kind));
    query.events.push_back(event);
}
struct Provider {
    const NativeStaticWorld &world;
    const MaterialLookup &materials;
    static ResolvedMaterial get(void *user, const BallisticEvent &event) {
        const auto &provider = *static_cast<Provider *>(user);
        uint64_t source;
        int32_t surface;
        std::memcpy(&source, event.bytes.data() + 24, sizeof(source));
        std::memcpy(&surface, event.bytes.data() + 40, sizeof(surface));
        require(source < provider.world.colliders.size(), CS2TL_INVALID_GEOMETRY,
                "Native event has no source collider.");
        const auto &collider =
            provider.world.source->colliders[provider.world.colliders[static_cast<size_t>(source)].source];
        auto material = provider.materials.get(surface);
        const auto as = provider.world.attributes[collider.attribute].filter.interact_as;
        material.flags = (as & (uint64_t{1} << 13)) ? CS2TL_MATERIAL_NATIVE_FLAG13 : 0;
        return material;
    }
};
void check_material_ties(const NativeQueryContext &query, float point_fraction,
                         NativeMaterialProvider provider) {
    for (size_t first = 0; first < query.events.size();) {
        const auto &event = query.events[first];
        if (event.fraction() > point_fraction) {
            break;
        }
        size_t next = first + 1;
        while (next < query.events.size() && query.events[next].fraction() == event.fraction() &&
               query.events[next].kind() == event.kind()) {
            ++next;
        }
        if (next > first + 1) {
            const auto reference = provider.get(provider.user, event);
            for (size_t i = first + 1; i < next; ++i) {
                const auto other = provider.get(provider.user, query.events[i]);
                require(reference.material == other.material && reference.flags == other.flags &&
                            std::bit_cast<uint32_t>(reference.penetration) ==
                                std::bit_cast<uint32_t>(other.penetration),
                        CS2TL_INCOMPLETE_GEOMETRY,
                        "Coincident native boundaries disagree about ballistic material; world instance "
                        "order is unavailable.");
            }
        }
        first = next;
    }
}
} // namespace
std::unique_ptr<NativeStaticWorld> build_native_static_world(const Scene &scene) {
    if (scene.physics.colliders.empty() ||
        std::none_of(scene.physics.attributes.begin(), scene.physics.attributes.end(),
                     [](const auto &a) { return (a.flags & physics_attribute_complete) != 0; })) {
        return {};
    }
    auto world = std::make_unique<NativeStaticWorld>();
    world->source = &scene.physics;
    world->attributes.resize(scene.physics.attributes.size());
    NativeQueryFilter query;
    query.include = 0x1c3009;
    query.group = 3;
    query.flags = 0x4b;
    for (size_t i = 0; i < world->attributes.size(); ++i) {
        try {
            world->attributes[i].filter = compile_native_attribute(scene.physics.attributes[i]);
        } catch (const Error &error) {
            world->attributes[i].status = error.code;
            world->complete_attributes = false;
        }
    }
    size_t triangle_count = 0;
    for (const auto &source : scene.physics.colliders) {
        if (source.shape == PhysicsShape::mesh && identity(source) && source.resource_flags == 0 &&
            !source.mesh_nodes.empty() && source.attribute < world->attributes.size() &&
            world->attributes[source.attribute].status == CS2TL_OK &&
            accept_native_collision(native_collision_rules(), query,
                                    world->attributes[source.attribute].filter)) {
            require(source.triangles.size() <= SIZE_MAX - triangle_count, CS2TL_UNSUPPORTED_FORMAT,
                    "Native normal count exceeds address-space representation.");
            triangle_count += source.triangles.size();
        }
    }
    world->normals.reserve(triangle_count);
    world->colliders.reserve(scene.physics.colliders.size());
    for (size_t i = 0; i < scene.physics.colliders.size(); ++i) {
        const auto &source = scene.physics.colliders[i];
        NativeColliderView collider;
        collider.source = static_cast<uint32_t>(i);
        if (source.attribute >= world->attributes.size()) {
            collider.status = CS2TL_INCOMPLETE_GEOMETRY;
        } else {
            collider.status = world->attributes[source.attribute].status;
        }
        if (collider.status == CS2TL_OK &&
            !accept_native_collision(native_collision_rules(), query,
                                     world->attributes[source.attribute].filter)) {
            continue;
        }
        collider.bounds = bounds(source);
        const bool mesh = source.shape == PhysicsShape::mesh;
        const bool hull = source.shape == PhysicsShape::hull;
        if (collider.status == CS2TL_OK &&
            (!identity(source) || (mesh && (source.resource_flags != 0 || source.mesh_nodes.empty())) ||
             (hull && source.planes.empty()) || (!mesh && !hull))) {
            collider.status = CS2TL_INCOMPLETE_GEOMETRY;
        }
        collider.normal_first = world->normals.size();
        if (collider.status == CS2TL_OK && mesh) {
            for (const auto &triangle : source.triangles) {
                world->normals.push_back(native_triangle_normal(triangle));
            }
        }
        world->colliders.push_back(collider);
    }
    IndexBuilder{*world};
    return world;
}
void query_native_static_world(const NativeStaticWorld &world, cs2tl_vec3 origin, cs2tl_vec3 delta,
                               NativeWorkspace &workspace, float max_event_fraction) {
    auto &query = workspace.query;
    query.events.clear();
    query.spans.clear();
    query.delta = delta;
    query.max_fraction = 1;
    query.started_inside = false;
    const Ray ray{Vec3(origin), Vec3(delta), 1, 0, 1, 0};
    double high = 1;
    traverse_nodes<BvhNode>(world.nodes, world.order, 0, ray, 0, high, [&](uint32_t index) {
        const auto &view = world.colliders[index];
        double entry;
        if (!intersects(view.bounds, ray, 0, 1, entry)) {
            return true;
        }
        require(
            view.status == CS2TL_OK, view.status,
            "A traversed source collider lacks audited attributes, original geometry, or a unit transform.");
        const auto &collider = world.source->colliders[view.source];
        if (collider.shape == PhysicsShape::mesh) {
            trace_native_mesh(collider, origin, delta, 1, 1, workspace.mesh, max_event_fraction);
            for (const auto &hit : workspace.mesh.hits) {
                if (hit.fraction > max_event_fraction) {
                    break;
                }
                append(query, {world.normals[view.normal_first + hit.triangle], hit.fraction, hit.kind},
                       index, (uint64_t(hit.triangle) << 32) | UINT32_MAX,
                       collider.triangles[hit.triangle].surface);
            }
        } else {
            const auto hits = trace_native_hull(collider.planes, origin, delta);
            for (uint32_t i = 0; i < hits.count; ++i) {
                if (hits.events[i].fraction > max_event_fraction) {
                    continue;
                }
                append(query, hits.events[i], index, UINT64_MAX, collider.surface);
                query.started_inside |= hits.events[i].kind == 0 && hits.events[i].fraction == 0;
            }
        }
        return true;
    });
    sort_native_events(query.events);
}
void simulate_native_remaining_damage(const NativeStaticWorld &world, const Ray &ray,
                                      const WeaponProfile &weapon, const MaterialLookup &materials,
                                      NativeWorkspace &workspace, cs2tl_shot_prediction &out) {
    if (ray.length > weapon.max_range) {
        out.outcome = CS2TL_SHOT_STOPPED;
        out.remaining_damage = 0;
        out.stop_reason = CS2TL_STOP_RANGE;
        return;
    }
    const auto origin = ray.origin.external();
    const auto supplied_delta = (ray.direction * ray.length).external();
    const auto distance = native_ray_length(supplied_delta);
    require(std::isfinite(distance) && distance > 0, CS2TL_INVALID_ARGUMENT,
            "Bullet direction exceeds the finite float32 range.");
    const auto direction = f32::mul(supplied_delta, f32::div(1, distance));
    const auto delta = f32::mul(direction, weapon.max_range);
    const auto length = native_ray_length(delta);
    require(std::isfinite(length) && length > 0, CS2TL_INVALID_ARGUMENT,
            "Weapon query extent exceeds the finite float32 range.");
    const auto point_fraction = f32::div(distance, length);
    if (point_fraction > 1) {
        out.outcome = CS2TL_SHOT_STOPPED;
        out.remaining_damage = 0;
        out.stop_reason = CS2TL_STOP_RANGE;
        return;
    }
    // The geometric traversal remains complete. Only event records strictly
    // after B are omitted; real contacts at B remain for the boundary guard.
    query_native_static_world(world, origin, delta, workspace, point_fraction);
    const auto point_status =
        plan_native_point_energy(workspace.query, point_fraction, distance, weapon.max_range);
    require(point_status == NativePointStatus::ready, CS2TL_INCOMPLETE_GEOMETRY,
            "B is not a certified air endpoint, or A starts inside an unresolved native collider.");
    Provider data{world, materials};
    const NativeMaterialProvider provider{&data, Provider::get};
    // Check only the consumed prefix. The private point marker is its sole
    // event at point_fraction, and was rejected above if a real face coincides.
    check_material_ties(workspace.query, std::nextafter(point_fraction, -INFINITY), provider);
    NativeDamageState state{
        weapon.base_damage, weapon.penetration_power, weapon.range_modifier, length, game_penetrations, 0};
    const auto status = consume_native_point_energy(workspace.query, origin, provider, state);
    out.outcome = status == NativePointStatus::reached ? CS2TL_SHOT_REACHED : CS2TL_SHOT_STOPPED;
    out.remaining_damage = state.damage;
    // Native eligibility can zero the remaining budget without completing a
    // penetration. Count consumed, successful material spans independently.
    out.penetration_count = static_cast<uint32_t>(
        std::count_if(workspace.query.spans.begin(), workspace.query.spans.end(),
                      [](const auto &span) { return span.solid() && span.initial_damage > 0; }));
    out.stop_reason = state.stop_reason;
}
} // namespace cs2::traceline
