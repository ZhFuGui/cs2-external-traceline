#include "PhysicsMetadata.h"
#include "ballistics/NativeMesh.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <bit>
#include <cmath>

namespace cs2::traceline {
namespace {
class Output {
  public:
    std::vector<uint8_t> bytes;
    void u32(uint32_t value) {
        for (unsigned i = 0; i < 4; ++i) {
            bytes.push_back(static_cast<uint8_t>(value >> (i * 8)));
        }
    }
    void f32(float value) { u32(std::bit_cast<uint32_t>(value)); }
    void u64(uint64_t value) {
        u32(static_cast<uint32_t>(value));
        u32(static_cast<uint32_t>(value >> 32));
    }
    void vector(cs2tl_vec3 value) {
        f32(value.x);
        f32(value.y);
        f32(value.z);
    }
    void count(size_t value) {
        require(value <= UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
                "Physics metadata indices exceed uint32 representation.");
        u32(static_cast<uint32_t>(value));
    }
    void list(const std::vector<uint32_t> &values) {
        count(values.size());
        for (auto value : values) {
            u32(value);
        }
    }
};
class Input {
    std::span<const uint8_t> bytes;
    size_t position{};

  public:
    explicit Input(std::span<const uint8_t> data) : bytes(data) {}
    size_t remaining() const { return bytes.size() - position; }
    uint32_t u32() {
        require(remaining() >= 4, CS2TL_CORRUPT_CACHE, "Truncated physics metadata.");
        uint32_t value = 0;
        for (unsigned i = 0; i < 4; ++i) {
            value |= uint32_t(bytes[position++]) << (i * 8);
        }
        return value;
    }
    float f32() {
        const auto value = std::bit_cast<float>(u32());
        require(std::isfinite(value), CS2TL_CORRUPT_CACHE, "Nonfinite physics metadata value.");
        return value;
    }
    uint64_t u64() {
        const uint64_t low = u32();
        return low | (uint64_t(u32()) << 32);
    }
    std::vector<uint8_t> block(uint64_t size) {
        require(size <= remaining(), CS2TL_CORRUPT_CACHE, "Truncated source mesh hierarchy.");
        const auto data = bytes.subspan(position, static_cast<size_t>(size));
        position += static_cast<size_t>(size);
        return {data.begin(), data.end()};
    }
    cs2tl_vec3 vector() {
        const auto x = f32(), y = f32(), z = f32();
        return {x, y, z};
    }
    uint32_t count(size_t record_size) {
        const auto value = u32();
        require(value <= remaining() / record_size, CS2TL_CORRUPT_CACHE,
                "Physics metadata count exceeds its payload.");
        return value;
    }
    std::vector<uint32_t> list() {
        const auto size = count(4);
        std::vector<uint32_t> result;
        result.reserve(size);
        for (uint32_t i = 0; i < size; ++i) {
            result.push_back(u32());
        }
        return result;
    }
};
bool finite(cs2tl_vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}
} // namespace
void validate_physics_metadata(const PhysicsData &physics) {
    for (const auto &attribute : physics.attributes) {
        require((attribute.flags & ~physics_attribute_complete) == 0, CS2TL_INVALID_GEOMETRY,
                "Invalid source collision attribute flags.");
    }
    uint32_t previous = CS2TL_INVALID_ID;
    for (const auto &collider : physics.colliders) {
        require(
            collider.object != CS2TL_INVALID_ID &&
                (previous == CS2TL_INVALID_ID || collider.object > previous) &&
                (collider.attribute == CS2TL_INVALID_ID || collider.attribute < physics.attributes.size()) &&
                (collider.flags & ~(physics_legacy_cover | physics_approximate | physics_bounds_available)) ==
                    0 &&
                static_cast<uint32_t>(collider.shape) <= static_cast<uint32_t>(PhysicsShape::capsule) &&
                collider.surface >= -1 && finite(collider.a) && finite(collider.b) &&
                std::all_of(collider.pose.begin(), collider.pose.end(),
                            [](float value) { return std::isfinite(value); }),
            CS2TL_INVALID_GEOMETRY, "Invalid native source collider metadata.");
        previous = collider.object;
        if ((collider.flags & physics_bounds_available) != 0) {
            require(finite(collider.bounds_min) && finite(collider.bounds_max) &&
                        collider.bounds_min.x <= collider.bounds_max.x &&
                        collider.bounds_min.y <= collider.bounds_max.y &&
                        collider.bounds_min.z <= collider.bounds_max.z,
                    CS2TL_INVALID_GEOMETRY, "Invalid source collider bounds.");
        }
        require(collider.shape == PhysicsShape::mesh || collider.mesh_nodes.empty(), CS2TL_INVALID_GEOMETRY,
                "Only a mesh can supply a source mesh hierarchy.");
        if (!collider.mesh_nodes.empty()) {
            validate_native_mesh_nodes(collider);
        }
        if (collider.shape == PhysicsShape::sphere || collider.shape == PhysicsShape::capsule) {
            require(std::isfinite(collider.radius) && collider.radius > 0 && collider.triangles.empty() &&
                        collider.planes.empty(),
                    CS2TL_INVALID_GEOMETRY, "Invalid native curved shape metadata.");
        } else {
            require(collider.radius == 0, CS2TL_INVALID_GEOMETRY, "Invalid native polyhedron radius.");
        }
        for (const auto &triangle : collider.triangles) {
            require(finite(triangle.a) && finite(triangle.b) && finite(triangle.c) && triangle.surface >= -1,
                    CS2TL_INVALID_GEOMETRY, "Invalid native source triangle.");
        }
        require(collider.shape == PhysicsShape::hull || collider.planes.empty(), CS2TL_INVALID_GEOMETRY,
                "Only a convex hull can supply source planes.");
        for (const auto &plane : collider.planes) {
            require(finite(plane.normal) && std::isfinite(plane.distance) &&
                        (plane.normal.x != 0 || plane.normal.y != 0 || plane.normal.z != 0),
                    CS2TL_INVALID_GEOMETRY, "Invalid source hull plane.");
        }
    }
}
std::vector<uint8_t> encode_physics_metadata(const PhysicsData &physics) {
    validate_physics_metadata(physics);
    Output output;
    output.u32(2); // Schema 2 also preserves source bounds and the mesh hierarchy.
    output.count(physics.attributes.size());
    output.count(physics.colliders.size());
    for (const auto &attribute : physics.attributes) {
        output.u32(attribute.group_hash);
        output.u32(attribute.include_detail_count);
        output.u32(attribute.flags);
        output.list(attribute.interact_as);
        output.list(attribute.interact_with);
        output.list(attribute.exclude);
        output.list(attribute.detail_layers);
    }
    for (const auto &collider : physics.colliders) {
        output.u32(static_cast<uint32_t>(collider.shape));
        output.u32(collider.object);
        output.u32(collider.attribute);
        output.u32(collider.flags);
        output.u32(collider.resource_flags);
        output.u32(std::bit_cast<uint32_t>(collider.surface));
        for (auto value : collider.pose) {
            output.f32(value);
        }
        output.vector(collider.a);
        output.vector(collider.b);
        output.f32(collider.radius);
        output.count(collider.triangles.size());
        output.count(collider.planes.size());
        output.vector(collider.bounds_min);
        output.vector(collider.bounds_max);
        output.u64(collider.mesh_nodes.size());
        for (const auto &triangle : collider.triangles) {
            output.vector(triangle.a);
            output.vector(triangle.b);
            output.vector(triangle.c);
            output.u32(std::bit_cast<uint32_t>(triangle.surface));
        }
        for (const auto &plane : collider.planes) {
            output.vector(plane.normal);
            output.f32(plane.distance);
        }
        output.bytes.insert(output.bytes.end(), collider.mesh_nodes.begin(), collider.mesh_nodes.end());
    }
    return std::move(output.bytes);
}
PhysicsData decode_physics_metadata(std::span<const uint8_t> bytes) {
    Input input(bytes);
    const auto schema = input.u32();
    require(schema == 1 || schema == 2, CS2TL_UNSUPPORTED_FORMAT, "Unsupported physics metadata schema.");
    const size_t collider_size = schema == 1 ? 108 : 140;
    const auto attributes = input.count(28), colliders = input.count(collider_size);
    require(uint64_t(attributes) * 28 + uint64_t(colliders) * collider_size <= input.remaining(),
            CS2TL_CORRUPT_CACHE, "Physics source counts exceed their payload.");
    PhysicsData physics;
    physics.attributes.reserve(attributes);
    for (uint32_t i = 0; i < attributes; ++i) {
        PhysicsAttribute attribute;
        attribute.group_hash = input.u32();
        attribute.include_detail_count = input.u32();
        attribute.flags = input.u32();
        attribute.interact_as = input.list();
        attribute.interact_with = input.list();
        attribute.exclude = input.list();
        attribute.detail_layers = input.list();
        physics.attributes.push_back(std::move(attribute));
    }
    physics.colliders.reserve(colliders);
    for (uint32_t i = 0; i < colliders; ++i) {
        PhysicsCollider collider;
        collider.shape = static_cast<PhysicsShape>(input.u32());
        collider.object = input.u32();
        collider.attribute = input.u32();
        collider.flags = input.u32();
        collider.resource_flags = input.u32();
        collider.surface = std::bit_cast<int32_t>(input.u32());
        for (auto &value : collider.pose) {
            value = input.f32();
        }
        collider.a = input.vector();
        collider.b = input.vector();
        collider.radius = input.f32();
        const auto triangles = input.count(40), planes = input.count(16);
        uint64_t nodes = 0;
        if (schema == 2) {
            collider.bounds_min = input.vector();
            collider.bounds_max = input.vector();
            nodes = input.u64();
        }
        require(uint64_t(triangles) * 40 + uint64_t(planes) * 16 <= input.remaining(), CS2TL_CORRUPT_CACHE,
                "Physics collider counts exceed their payload.");
        require(nodes <= input.remaining() - uint64_t(triangles) * 40 - uint64_t(planes) * 16,
                CS2TL_CORRUPT_CACHE, "Source mesh hierarchy exceeds its payload.");
        collider.triangles.reserve(triangles);
        for (uint32_t j = 0; j < triangles; ++j) {
            PhysicsTriangle triangle;
            triangle.a = input.vector();
            triangle.b = input.vector();
            triangle.c = input.vector();
            triangle.surface = std::bit_cast<int32_t>(input.u32());
            collider.triangles.push_back(triangle);
        }
        collider.planes.reserve(planes);
        for (uint32_t j = 0; j < planes; ++j) {
            PhysicsPlane plane;
            plane.normal = input.vector();
            plane.distance = input.f32();
            collider.planes.push_back(plane);
        }
        collider.mesh_nodes = input.block(nodes);
        physics.colliders.push_back(std::move(collider));
    }
    require(input.remaining() == 0, CS2TL_CORRUPT_CACHE, "Unexpected physics metadata tail.");
    try {
        validate_physics_metadata(physics);
    } catch (const Error &error) {
        throw Error(CS2TL_CORRUPT_CACHE, error.what());
    }
    return physics;
}
} // namespace cs2::traceline
