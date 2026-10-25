#include "PhysicsTextReader.h"
#include "diagnostics/Error.h"
#include <array>
#include <bit>
#include <limits>

namespace cs2::traceline::text {
namespace {
uint32_t le32(const uint8_t *p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
Vec3 vector(const Value &value) {
    if (value.kind == Kind::array) {
        require(value.children.size() == 3, CS2TL_PARSE_ERROR, "Vertex must have exactly 3 coordinates.",
                value.offset);
        return {number(value.children[0]), number(value.children[1]), number(value.children[2])};
    }
    return {number(value.need("x")), number(value.need("y")), number(value.need("z"))};
}
} // namespace
std::vector<uint32_t> uints(const Value &value) {
    std::vector<uint32_t> result;
    if (value.kind == Kind::blob) {
        const auto data = bytes(value);
        require(data.size() % 4 == 0, CS2TL_PARSE_ERROR, "Integer blob length must be a multiple of 4.",
                value.offset);
        result.reserve(data.size() / 4);
        for (size_t i = 0; i < data.size(); i += 4) {
            result.push_back(le32(data.data() + i));
        }
    } else {
        require(value.kind == Kind::array, CS2TL_PARSE_ERROR, "Expected integer array.", value.offset);
        for (const auto &v : value.children) {
            const auto n = integer(v);
            require(n >= 0 && n <= UINT32_MAX, CS2TL_PARSE_ERROR, "Unsigned integer out of range.", v.offset);
            result.push_back(static_cast<uint32_t>(n));
        }
    }
    return result;
}
std::vector<Vec3> vertices(const Value &value) {
    std::vector<Vec3> result;
    if (value.kind == Kind::blob) {
        const auto data = bytes(value);
        require(data.size() % 12 == 0, CS2TL_PARSE_ERROR, "Vertex blob length must be a multiple of 12.",
                value.offset);
        result.reserve(data.size() / 12);
        for (size_t i = 0; i < data.size(); i += 12) {
            Vec3 v{std::bit_cast<float>(le32(data.data() + i)),
                   std::bit_cast<float>(le32(data.data() + i + 4)),
                   std::bit_cast<float>(le32(data.data() + i + 8))};
            require(v.finite(), CS2TL_INVALID_GEOMETRY, "Nonfinite vertex in blob.", value.offset + i);
            result.push_back(v);
        }
    } else {
        require(value.kind == Kind::array, CS2TL_PARSE_ERROR, "Expected vertex array.", value.offset);
        if (!value.children.empty() && value.children.front().kind == Kind::atom) {
            require(value.children.size() % 3 == 0, CS2TL_PARSE_ERROR,
                    "Flat vertex array length must be divisible by 3.", value.offset);
            for (size_t i = 0; i < value.children.size(); i += 3) {
                result.emplace_back(number(value.children[i]), number(value.children[i + 1]),
                                    number(value.children[i + 2]));
            }
        } else {
            for (const auto &child : value.children) {
                result.push_back(vector(child));
            }
        }
    }
    require(result.size() <= UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
            "Vertex indices exceed uint32 representation.", value.offset);
    return result;
}
std::vector<Triangle> decode_mesh(const Value &descriptor, uint32_t object) {
    const auto &mesh = descriptor.need("m_Mesh");
    const auto *positions = mesh.get("m_VertexPositions");
    const auto verts = vertices(positions ? *positions : mesh.need("m_Vertices"));
    const auto &source = mesh.need("m_Triangles");
    std::vector<std::array<uint32_t, 3>> indices;
    if (source.kind == Kind::blob ||
        (source.kind == Kind::array && (source.children.empty() || source.children[0].kind == Kind::atom))) {
        const auto flat = uints(source);
        require(flat.size() % 3 == 0, CS2TL_PARSE_ERROR, "Triangle index count must be divisible by 3.",
                source.offset);
        for (size_t i = 0; i < flat.size(); i += 3) {
            indices.push_back({flat[i], flat[i + 1], flat[i + 2]});
        }
    } else {
        require(source.kind == Kind::array, CS2TL_PARSE_ERROR, "Expected triangle array.", source.offset);
        for (const auto &value : source.children) {
            const auto tri = uints(value.kind == Kind::object ? value.need("m_nIndex") : value);
            require(tri.size() == 3, CS2TL_PARSE_ERROR, "Triangle must have 3 indices.", value.offset);
            indices.push_back({tri[0], tri[1], tri[2]});
        }
    }
    require(indices.size() <= UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
            "Triangle indices exceed uint32 representation.", source.offset);
    int32_t surface = CS2TL_UNKNOWN_SURFACE;
    if (const auto *field = descriptor.get("m_nSurfacePropertyIndex")) {
        const auto n = integer(*field);
        require(n >= -1 && n <= INT32_MAX, CS2TL_PARSE_ERROR, "Surface identifier out of range.",
                field->offset);
        surface = static_cast<int32_t>(n);
    }
    std::vector<uint32_t> materials;
    if (const auto *field = mesh.get("m_Materials")) {
        materials = uints(*field);
    }
    require(materials.empty() || materials.size() == indices.size(), CS2TL_INVALID_GEOMETRY,
            "Material count must be zero or match triangle count.", mesh.offset);
    std::vector<Triangle> result;
    result.reserve(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        const auto &t = indices[i];
        require(t[0] < verts.size() && t[1] < verts.size() && t[2] < verts.size(), CS2TL_INVALID_GEOMETRY,
                "Triangle references an invalid/negative vertex index.", source.offset);
        if (!materials.empty()) {
            require(materials[i] <= INT32_MAX, CS2TL_INVALID_GEOMETRY,
                    "Invalid per-triangle surface identifier.", mesh.offset);
            surface = static_cast<int32_t>(materials[i]);
        }
        result.push_back({verts[t[0]], verts[t[1]], verts[t[2]], surface, object});
    }
    return result;
}
} // namespace cs2::traceline::text
