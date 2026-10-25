#include "PhysicsTextReader.h"
#include "diagnostics/Error.h"
#include "storage/FileIO.h"
#include <algorithm>
#include <array>
#include <cctype>

namespace cs2::traceline {
namespace {
using text::Kind;
using text::Value;
struct Transform {
    std::array<double, 12> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    Vec3 apply(Vec3 p) const {
        return {m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
                m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
    }
};
Transform multiply(const Transform &a, const Transform &b) {
    Transform result;
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            result.m[r * 4 + c] = c == 3 ? a.m[r * 4 + 3] : 0;
            for (size_t k = 0; k < 3; ++k) {
                result.m[r * 4 + c] += a.m[r * 4 + k] * b.m[k * 4 + c];
            }
        }
    }
    return result;
}
void flatten(const Value &value, std::vector<double> &out) {
    if (value.kind == Kind::array) {
        for (const auto &child : value.children) {
            flatten(child, out);
        }
    } else {
        out.push_back(text::number(value));
    }
}
Transform transform(const Value &value) {
    std::vector<double> values;
    flatten(value, values);
    require(values.size() == 12 || values.size() == 16, CS2TL_UNSUPPORTED_FORMAT,
            "Bind pose must be a row-major 3x4 or affine 4x4 array.", value.offset);
    if (values.size() == 16) {
        require(values[12] == 0 && values[13] == 0 && values[14] == 0 && values[15] == 1,
                CS2TL_UNSUPPORTED_FORMAT, "Non-affine bind pose.", value.offset);
    }
    Transform out;
    std::copy_n(values.begin(), 12, out.m.begin());
    return out;
}
class Importer {
    BuildOptions options;
    std::vector<Triangle> triangles;
    std::vector<uint32_t> hashes;
    uint32_t next_object{};
    bool approximate{}, found_shapes{};
    bool accept(const Value &descriptor, const Value *attributes) const {
        if ((options.flags & CS2TL_BUILD_ALL_COLLISION_GROUPS) != 0 || !attributes) {
            return true;
        }
        require(attributes->kind == Kind::array, CS2TL_PARSE_ERROR, "Collision attributes must be an array.",
                attributes->offset);
        const auto &index = descriptor.need("m_nCollisionAttributeIndex");
        const auto n = text::integer(index);
        require(n >= 0 && static_cast<uint64_t>(n) < attributes->children.size(), CS2TL_INVALID_GEOMETRY,
                "Collision attribute index out of range.", index.offset);
        const auto &group = attributes->children[static_cast<size_t>(n)].need("m_CollisionGroupString");
        require(group.kind == Kind::atom, CS2TL_PARSE_ERROR, "Collision group must be a string.",
                group.offset);
        std::string name = group.scalar;
        for (char &c : name) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        return name == "default";
    }
    void shapes(const Value &array, bool hull, const Value *attributes, const Transform &pose) {
        found_shapes = true;
        require(array.kind == Kind::array, CS2TL_PARSE_ERROR, "Shapes must be an array.", array.offset);
        for (const auto &descriptor : array.children) {
            require(next_object != CS2TL_INVALID_ID, CS2TL_INVALID_GEOMETRY, "Too many objects.");
            const uint32_t object = next_object++;
            if (!accept(descriptor, attributes)) {
                continue;
            }
            auto decoded =
                hull ? text::decode_hull(descriptor, object,
                                         (options.flags & CS2TL_BUILD_APPROXIMATE_HULLS) != 0, approximate)
                     : text::decode_mesh(descriptor, object);
            require(triangles.size() + decoded.size() <= max_triangles, CS2TL_INVALID_GEOMETRY,
                    "Scene triangle limit exceeded.", descriptor.offset);
            for (auto &t : decoded) {
                t.a = pose.apply(t.a);
                t.b = pose.apply(t.b);
                t.c = pose.apply(t.c);
                triangles.push_back(t);
            }
        }
    }

  public:
    explicit Importer(BuildOptions value) : options(value) {}
    void walk(const Value &node, const Value *attributes = nullptr, const Transform &pose = {}) {
        if (node.kind == Kind::array) {
            for (const auto &child : node.children) {
                walk(child, attributes, pose);
            }
            return;
        }
        if (node.kind != Kind::object) {
            return;
        }
        if (const auto *local = node.get("m_collisionAttributes")) {
            attributes = local;
        }
        if (const auto *table = node.get("m_surfacePropertyHashes")) {
            auto local = text::uints(*table);
            require(local.size() <= 1'000'000, CS2TL_UNSUPPORTED_FORMAT, "Surface table exceeds size limit.",
                    table->offset);
            require(hashes.empty() || hashes == local, CS2TL_UNSUPPORTED_FORMAT,
                    "Conflicting surface property tables.", table->offset);
            hashes = std::move(local);
        }
        const auto *parts = node.get("m_parts");
        const auto *bind = node.get("m_bindPose");
        if (bind) {
            require(bind->kind == Kind::array, CS2TL_UNSUPPORTED_FORMAT,
                    "Bind pose must be an array of matrices.", bind->offset);
        }
        if (bind && !bind->children.empty()) {
            require(parts != nullptr, CS2TL_UNSUPPORTED_FORMAT, "Bind pose without parts.", bind->offset);
        }
        for (size_t i = 0; i < node.keys.size(); ++i) {
            const auto &key = node.keys[i];
            const auto &value = node.children[i];
            if (key == "m_meshes") {
                shapes(value, false, attributes, pose);
            } else if (key == "m_hulls") {
                shapes(value, true, attributes, pose);
            } else if (key == "m_spheres" || key == "m_capsules" || key == "m_compounds") {
                require(value.kind == Kind::array, CS2TL_PARSE_ERROR, "Shape field must be an array: " + key,
                        value.offset);
                require(value.children.empty(), CS2TL_UNSUPPORTED_FORMAT,
                        "Unsupported collision shape array: " + key, value.offset);
            } else if (key == "m_parts") {
                require(value.kind == Kind::array, CS2TL_PARSE_ERROR, "Parts must be an array.",
                        value.offset);
                require(!bind || (bind->kind == Kind::array &&
                                  (bind->children.empty() || bind->children.size() == value.children.size())),
                        CS2TL_UNSUPPORTED_FORMAT, "Bind pose count must match parts.", value.offset);
                for (size_t p = 0; p < value.children.size(); ++p) {
                    const Transform part_pose =
                        bind && !bind->children.empty() ? multiply(pose, transform(bind->children[p])) : pose;
                    walk(value.children[p], attributes, part_pose);
                }
            } else if (key != "m_collisionAttributes" && key != "m_surfacePropertyHashes" &&
                       key != "m_bindPose") {
                walk(value, attributes, pose);
            }
        }
    }
    Scene finish() {
        require(found_shapes, CS2TL_UNSUPPORTED_FORMAT, "No m_meshes/m_hulls arrays found in physics text.");
        if (!hashes.empty()) {
            for (const auto &t : triangles) {
                require(t.surface < 0 || static_cast<size_t>(t.surface) < hashes.size(),
                        CS2TL_INVALID_GEOMETRY, "Surface identifier exceeds surface property table.");
            }
        }
        auto scene = build_scene(std::move(triangles), options);
        scene.surface_hashes = std::move(hashes);
        if (approximate) {
            scene.flags |= CS2TL_SCENE_APPROXIMATE;
        }
        return scene;
    }
};
} // namespace
Scene import_physics(const std::filesystem::path &path, BuildOptions options) {
    const auto data = read_file(path);
    require(!data.empty(), CS2TL_PARSE_ERROR, "Physics text is empty.", 0);
    const auto root = text::parse(std::string_view(reinterpret_cast<const char *>(data.data()), data.size()));
    Importer importer(options);
    importer.walk(root);
    auto scene = importer.finish();
    scene.fingerprint = fingerprint(data);
    return scene;
}
} // namespace cs2::traceline
