#pragma once
#include "scene/Scene.h"
#include <filesystem>
#include <span>
#include <string>
#include <string_view>

namespace cs2::traceline::text {
enum class Kind { object, array, atom, blob };
struct Value {
    Value() = default;
    Value(Value &&) noexcept = default;
    Value &operator=(Value &&) noexcept = default;
    Value(const Value &) = delete;
    Value &operator=(const Value &) = delete;
    ~Value() {
        // Walk owned children bottom-up without recursion or allocations. A
        // deeply nested input must not overflow the stack during cleanup.
        Value *current = this;
        for (;;) {
            if (!current->children.empty()) {
                auto *child = &current->children.back();
                child->cleanup_parent = current;
                current = child;
            } else {
                if (current == this) {
                    break;
                }
                auto *parent = current->cleanup_parent;
                parent->children.pop_back();
                current = parent;
            }
        }
    }
    Kind kind{Kind::atom};
    uint64_t offset{};
    std::string scalar;
    std::string_view blob;
    std::vector<std::string> keys;
    std::vector<Value> children;
    const Value *get(std::string_view key) const;
    const Value &need(std::string_view key) const;

  private:
    Value *cleanup_parent{};
};
Value parse(std::string_view input);
int64_t integer(const Value &value);
double number(const Value &value);
std::vector<uint8_t> bytes(const Value &value);
std::vector<uint32_t> uints(const Value &value);
std::vector<Vec3> vertices(const Value &value);
std::vector<Triangle> decode_mesh(const Value &descriptor, uint32_t object);
std::vector<Triangle> decode_hull(const Value &descriptor, uint32_t object, bool allow_approximation,
                                  bool &approximate);
} // namespace cs2::traceline::text
namespace cs2::traceline {
struct PhysicsSource {
    std::filesystem::path path;
    uint32_t flags{};
    std::array<double, 12> transform{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
};
Scene import_physics(const std::filesystem::path &path, BuildOptions options);
Scene import_physics_sources(std::span<const PhysicsSource> sources, BuildOptions options);
} // namespace cs2::traceline
