#include "PhysicsTextReader.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <array>
#include <unordered_set>

namespace cs2::traceline::text {
namespace {
struct HalfEdge {
    uint32_t next{}, twin{}, origin{}, face{};
};
uint32_t index(const Value &value) {
    const auto n = integer(value);
    require(n >= 0 && n <= 255, CS2TL_INVALID_GEOMETRY, "Hull topology index exceeds uint8 range.",
            value.offset);
    return static_cast<uint32_t>(n);
}
std::vector<Triangle> box(const Value &hull, int32_t surface, uint32_t object) {
    const auto &bounds = hull.need("m_Bounds");
    auto coordinate = [](const Value &v) {
        if (v.kind == Kind::array) {
            require(v.children.size() == 3, CS2TL_PARSE_ERROR, "Bounds need 3 coordinates.", v.offset);
            return Vec3(number(v.children[0]), number(v.children[1]), number(v.children[2]));
        }
        return Vec3(number(v.need("x")), number(v.need("y")), number(v.need("z")));
    };
    const auto lo = coordinate(bounds.need("m_vMinBounds")), hi = coordinate(bounds.need("m_vMaxBounds"));
    require(lo.x < hi.x && lo.y < hi.y && lo.z < hi.z, CS2TL_INVALID_GEOMETRY,
            "Hull approximation bounds must have positive volume.", bounds.offset);
    const std::array<Vec3, 8> v{{lo,
                                 {hi.x, lo.y, lo.z},
                                 {hi.x, hi.y, lo.z},
                                 {lo.x, hi.y, lo.z},
                                 {lo.x, lo.y, hi.z},
                                 {hi.x, lo.y, hi.z},
                                 hi,
                                 {lo.x, hi.y, hi.z}}};
    const std::array<std::array<unsigned, 3>, 12> faces{{{0, 2, 1},
                                                         {0, 3, 2},
                                                         {4, 5, 6},
                                                         {4, 6, 7},
                                                         {0, 1, 5},
                                                         {0, 5, 4},
                                                         {3, 6, 2},
                                                         {3, 7, 6},
                                                         {0, 7, 3},
                                                         {0, 4, 7},
                                                         {1, 2, 6},
                                                         {1, 6, 5}}};
    std::vector<Triangle> result;
    for (const auto &f : faces) {
        result.push_back({v[f[0]], v[f[1]], v[f[2]], surface, object});
    }
    return result;
}
} // namespace
std::vector<Triangle> decode_hull(const Value &descriptor, uint32_t object, bool allow_approximation,
                                  bool &approximate) {
    const auto &hull = descriptor.need("m_Hull");
    int32_t surface = CS2TL_UNKNOWN_SURFACE;
    if (const auto *field = descriptor.get("m_nSurfacePropertyIndex")) {
        const auto n = integer(*field);
        require(n >= -1 && n <= INT32_MAX, CS2TL_PARSE_ERROR, "Hull surface identifier out of range.",
                field->offset);
        surface = static_cast<int32_t>(n);
    }
    const auto *positions = hull.get("m_VertexPositions");
    if (!positions) {
        positions = hull.get("m_Vertices");
    }
    const auto *edges_value = hull.get("m_Edges");
    const auto *faces_value = hull.get("m_Faces");
    if (!positions || !edges_value || !faces_value) {
        require(allow_approximation, CS2TL_UNSUPPORTED_FORMAT,
                "Hull topology is missing; bounds-only approximation requires explicit opt-in.", hull.offset);
        approximate = true;
        return box(hull, surface, object);
    }
    const auto verts = vertices(*positions);
    require(!verts.empty() && verts.size() <= 256, CS2TL_INVALID_GEOMETRY, "Hull vertex count out of range.",
            positions->offset);
    std::vector<HalfEdge> edges;
    if (edges_value->kind == Kind::blob) {
        const auto data = bytes(*edges_value);
        require(data.size() % 4 == 0, CS2TL_PARSE_ERROR, "Hull half-edge blob length must be divisible by 4.",
                edges_value->offset);
        for (size_t i = 0; i < data.size(); i += 4) {
            edges.push_back({data[i], data[i + 1], data[i + 2], data[i + 3]});
        }
    } else {
        require(edges_value->kind == Kind::array, CS2TL_PARSE_ERROR, "Expected hull half-edge array.",
                edges_value->offset);
        for (const auto &e : edges_value->children) {
            edges.push_back({index(e.need("m_nNext")), index(e.need("m_nTwin")), index(e.need("m_nOrigin")),
                             index(e.need("m_nFace"))});
        }
    }
    std::vector<uint32_t> faces;
    if (faces_value->kind == Kind::blob) {
        for (auto b : bytes(*faces_value)) {
            faces.push_back(b);
        }
    } else {
        require(faces_value->kind == Kind::array, CS2TL_PARSE_ERROR, "Expected hull face array.",
                faces_value->offset);
        for (const auto &f : faces_value->children) {
            faces.push_back(index(f.kind == Kind::object ? f.need("m_nEdge") : f));
        }
    }
    require(!edges.empty() && edges.size() <= 256 && faces.size() >= 4 && faces.size() <= 256,
            CS2TL_INVALID_GEOMETRY, "Hull edge/face count out of range.", hull.offset);
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto &e = edges[i];
        require(e.next < edges.size() && e.twin < edges.size() && e.origin < verts.size() &&
                    e.face < faces.size(),
                CS2TL_INVALID_GEOMETRY, "Hull edge references an invalid index.", edges_value->offset);
        require(e.twin != i && edges[e.twin].twin == i, CS2TL_INVALID_GEOMETRY,
                "Hull twin relationship is invalid.", edges_value->offset);
        require(edges[e.twin].origin == edges[e.next].origin, CS2TL_INVALID_GEOMETRY,
                "Hull twin endpoint mismatch.", edges_value->offset);
    }
    std::vector<bool> visited(edges.size());
    std::vector<Triangle> result;
    for (uint32_t face = 0; face < faces.size(); ++face) {
        const uint32_t start = faces[face];
        require(start < edges.size(), CS2TL_INVALID_GEOMETRY, "Invalid hull face edge.", faces_value->offset);
        std::vector<uint32_t> polygon;
        std::unordered_set<uint32_t> face_vertices;
        uint32_t current = start;
        do {
            require(!visited[current] && edges[current].face == face, CS2TL_INVALID_GEOMETRY,
                    "Hull face loop is broken or reuses an edge.", faces_value->offset);
            visited[current] = true;
            require(face_vertices.insert(edges[current].origin).second, CS2TL_INVALID_GEOMETRY,
                    "Hull face repeats a vertex.", faces_value->offset);
            polygon.push_back(edges[current].origin);
            current = edges[current].next;
        } while (current != start);
        require(polygon.size() >= 3, CS2TL_INVALID_GEOMETRY, "Hull face has fewer than 3 vertices.",
                faces_value->offset);
        for (size_t i = 1; i + 1 < polygon.size(); ++i) {
            result.push_back({verts[polygon[0]], verts[polygon[i]], verts[polygon[i + 1]], surface, object});
        }
    }
    require(std::all_of(visited.begin(), visited.end(), [](bool v) { return v; }), CS2TL_INVALID_GEOMETRY,
            "Hull contains unreferenced half edges.", edges_value->offset);
    return result;
}
} // namespace cs2::traceline::text
