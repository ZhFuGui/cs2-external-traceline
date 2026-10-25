#pragma once
#include "scene/Scene.h"
#include <filesystem>
#include <string>
#include <string_view>

namespace cs2::traceline::text {
enum class Kind { object, array, atom, blob };
struct Value {
    Kind kind{Kind::atom};
    uint64_t offset{};
    std::string scalar;
    std::string_view blob;
    std::vector<std::string> keys;
    std::vector<Value> children;
    const Value *get(std::string_view key) const;
    const Value &need(std::string_view key) const;
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
Scene import_physics(const std::filesystem::path &path, BuildOptions options);
}
