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
    std::vector<const Value *> pending{&value};
    while (!pending.empty()) {
        const auto *current = pending.back();
        pending.pop_back();
        if (current->kind == Kind::array) {
            for (auto it = current->children.rbegin(); it != current->children.rend(); ++it) {
                pending.push_back(&*it);
            }
        } else {
            out.push_back(text::number(*current));
        }
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
        const auto &attribute = attributes->children[static_cast<size_t>(n)];
        auto tagged = [&](std::string_view field, std::string_view tag) {
            const auto *values = attribute.get(field);
            if (!values) {
                return false;
            }
            require(values->kind == Kind::array, CS2TL_PARSE_ERROR, "Collision tags must be an array.",
                    values->offset);
            return std::any_of(values->children.begin(), values->children.end(), [&](const auto &value) {
                std::string label = value.scalar;
                for (char &c : label) {
                    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                }
                return label == tag;
            });
        };
        if (tagged("m_InteractAsStrings", "passbullets") || tagged("m_InteractExcludeStrings", "bullet") ||
            tagged("m_InteractExcludeStrings", "bullets")) {
            return false;
        }
        if (name == "default") {
            return true;
        }
        // Source game scripts/collision_properties.txt explicitly describes
        // window and these solid tags as solid. Player/nav/grenade-only clips,
        // trigger volumes and sound-only blockers must not become cover.
        return name == "conditionallysolid" &&
               (tagged("m_InteractAsStrings", "window") || tagged("m_InteractAsStrings", "contents_solid") ||
                tagged("m_InteractAsStrings", "contents_solid_no_block_los"));
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
            require(triangles.size() + decoded.size() <= UINT32_MAX, CS2TL_UNSUPPORTED_FORMAT,
                    "Scene indices exceed uint32 representation.", descriptor.offset);
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
    void walk(const Value &root, const Value *attributes = nullptr, const Transform &pose = {}) {
        struct Frame {
            const Value *node;
            const Value *attributes;
            Transform pose;
            size_t next{};
            const Value *part_bind{};
            bool entered{};
        };
        std::vector<Frame> pending{{&root, attributes, pose}};
        while (!pending.empty()) {
            auto &frame = pending.back();
            const auto &node = *frame.node;
            if (node.kind != Kind::array && node.kind != Kind::object) {
                pending.pop_back();
                continue;
            }
            if (!frame.entered) {
                frame.entered = true;
                if (node.kind == Kind::object) {
                    if (const auto *local = node.get("m_collisionAttributes")) {
                        frame.attributes = local;
                    }
                    if (const auto *table = node.get("m_surfacePropertyHashes")) {
                        auto local = text::uints(*table);
                        require(local.size() <= uint64_t(INT32_MAX) + 1, CS2TL_UNSUPPORTED_FORMAT,
                                "Surface indices exceed int32 representation.", table->offset);
                        if (local.size() > 1'000'000) {
                            warn("Large surface property table (>1000000); processing continues.",
                                 table->offset);
                        }
                        require(hashes.empty() || hashes == local, CS2TL_UNSUPPORTED_FORMAT,
                                "Conflicting surface property tables.", table->offset);
                        hashes = std::move(local);
                    }
                    const auto *bind = node.get("m_bindPose");
                    if (bind) {
                        require(bind->kind == Kind::array, CS2TL_UNSUPPORTED_FORMAT,
                                "Bind pose must be an array of matrices.", bind->offset);
                        if (!bind->children.empty()) {
                            require(node.get("m_parts") != nullptr, CS2TL_UNSUPPORTED_FORMAT,
                                    "Bind pose without parts.", bind->offset);
                        }
                    }
                }
            }
            if (frame.next == node.children.size()) {
                pending.pop_back();
                continue;
            }
            const auto index = frame.next++;
            const auto &value = node.children[index];
            if (node.kind == Kind::array) {
                auto child_pose = frame.pose;
                if (frame.part_bind && !frame.part_bind->children.empty()) {
                    child_pose = multiply(child_pose, transform(frame.part_bind->children[index]));
                }
                pending.push_back({&value, frame.attributes, child_pose});
                continue;
            }
            const auto &key = node.keys[index];
            if (key == "m_meshes") {
                shapes(value, false, frame.attributes, frame.pose);
            } else if (key == "m_hulls") {
                shapes(value, true, frame.attributes, frame.pose);
            } else if (key == "m_spheres" || key == "m_capsules" || key == "m_compounds") {
                require_lazy(value.kind == Kind::array, CS2TL_PARSE_ERROR,
                             [&] { return "Shape field must be an array: " + key; }, value.offset);
                require_lazy(value.children.empty(), CS2TL_UNSUPPORTED_FORMAT,
                             [&] { return "Unsupported collision shape array: " + key; }, value.offset);
            } else if (key == "m_parts") {
                const auto *bind = node.get("m_bindPose");
                require(value.kind == Kind::array, CS2TL_PARSE_ERROR, "Parts must be an array.",
                        value.offset);
                require(!bind || bind->children.empty() || bind->children.size() == value.children.size(),
                        CS2TL_UNSUPPORTED_FORMAT, "Bind pose count must match parts.", value.offset);
                pending.push_back({&value, frame.attributes, frame.pose, 0, bind});
            } else if (key != "m_collisionAttributes" && key != "m_surfacePropertyHashes" &&
                       key != "m_bindPose") {
                pending.push_back({&value, frame.attributes, frame.pose});
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
    Importer importer(options);
    uint64_t source_fingerprint{};
    {
        const auto data = read_file(path);
        require(!data.empty(), CS2TL_PARSE_ERROR, "Physics text is empty.", 0);
        const auto root =
            text::parse(std::string_view(reinterpret_cast<const char *>(data.data()), data.size()));
        importer.walk(root);
        source_fingerprint = fingerprint(data);
    } // Release the text tree and input buffer before building topology/BVH.
    auto scene = importer.finish();
    scene.fingerprint = source_fingerprint;
    return scene;
}
} // namespace cs2::traceline
