#include "PhysicsTextReader.h"
#include "diagnostics/Error.h"
#include <cctype>
#include <charconv>
#include <limits>
#include <unordered_set>

namespace cs2::traceline::text {
const Value *Value::get(std::string_view key) const {
    if (kind != Kind::object) {
        return nullptr;
    }
    for (size_t i = 0; i < keys.size(); ++i) {
        if (keys[i] == key) {
            return &children[i];
        }
    }
    return nullptr;
}
const Value &Value::need(std::string_view key) const {
    const auto *value = get(key);
    require_lazy(
        value != nullptr, CS2TL_PARSE_ERROR, [&] { return "Missing field: " + std::string(key); }, offset);
    return *value;
}
namespace {
class Reader {
    std::string_view input;
    size_t pos{};
    size_t count{};
    bool starts(std::string_view value) const { return input.substr(pos, value.size()) == value; }
    [[noreturn]] void fail(const std::string &message) const { throw Error(CS2TL_PARSE_ERROR, message, pos); }
    void skip() {
        for (;;) {
            while (pos < input.size() && std::isspace(static_cast<unsigned char>(input[pos]))) {
                ++pos;
            }
            if (starts("//")) {
                while (pos < input.size() && input[pos] != '\n') {
                    ++pos;
                }
            } else if (starts("/*") || starts("<!--")) {
                const bool header = starts("<!--");
                const auto end = input.find(header ? "-->" : "*/", pos + (header ? 4 : 2));
                if (end == std::string_view::npos) {
                    fail("Unterminated comment/header.");
                }
                pos = end + (header ? 3 : 2);
            } else {
                return;
            }
        }
    }
    uint32_t hex4() {
        uint32_t code = 0;
        for (unsigned i = 0; i < 4; ++i) {
            if (pos >= input.size()) {
                fail("Incomplete Unicode escape.");
            }
            const char c = input[pos++];
            const int digit =
                c >= '0' && c <= '9'
                    ? c - '0'
                    : (c >= 'a' && c <= 'f' ? c - 'a' + 10 : (c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1));
            if (digit < 0) {
                fail("Invalid Unicode escape.");
            }
            code = (code << 4) | static_cast<uint32_t>(digit);
        }
        return code;
    }
    static void utf8(std::string &out, uint32_t code) {
        if (code <= 0x7f) {
            out.push_back(static_cast<char>(code));
        } else if (code <= 0x7ff) {
            out.push_back(static_cast<char>(0xc0 | (code >> 6)));
            out.push_back(static_cast<char>(0x80 | (code & 63)));
        } else if (code <= 0xffff) {
            out.push_back(static_cast<char>(0xe0 | (code >> 12)));
            out.push_back(static_cast<char>(0x80 | ((code >> 6) & 63)));
            out.push_back(static_cast<char>(0x80 | (code & 63)));
        } else {
            out.push_back(static_cast<char>(0xf0 | (code >> 18)));
            out.push_back(static_cast<char>(0x80 | ((code >> 12) & 63)));
            out.push_back(static_cast<char>(0x80 | ((code >> 6) & 63)));
            out.push_back(static_cast<char>(0x80 | (code & 63)));
        }
    }
    std::string token() {
        skip();
        if (pos >= input.size()) {
            fail("Unexpected end of input.");
        }
        std::string result;
        if (input[pos] == '\"') {
            ++pos;
            while (pos < input.size()) {
                char c = input[pos++];
                if (c == '\"') {
                    return result;
                }
                if (static_cast<unsigned char>(c) < 32) {
                    fail("Control character in string.");
                }
                if (c == '\\') {
                    if (pos >= input.size()) {
                        fail("Incomplete string escape.");
                    }
                    c = input[pos++];
                    switch (c) {
                    case '\"':
                    case '\\':
                    case '/':
                        result.push_back(c);
                        break;
                    case 'n':
                        result.push_back('\n');
                        break;
                    case 'r':
                        result.push_back('\r');
                        break;
                    case 't':
                        result.push_back('\t');
                        break;
                    case 'b':
                        result.push_back('\b');
                        break;
                    case 'f':
                        result.push_back('\f');
                        break;
                    case 'u': {
                        uint32_t code = hex4();
                        if (code >= 0xd800 && code <= 0xdbff) {
                            if (!starts("\\u")) {
                                fail("Unpaired Unicode surrogate.");
                            }
                            pos += 2;
                            const auto low = hex4();
                            if (low < 0xdc00 || low > 0xdfff) {
                                fail("Invalid Unicode surrogate.");
                            }
                            code = 0x10000 + ((code - 0xd800) << 10) + (low - 0xdc00);
                        } else if (code >= 0xdc00 && code <= 0xdfff) {
                            fail("Unpaired Unicode surrogate.");
                        }
                        utf8(result, code);
                        break;
                    }
                    default:
                        fail("Unsupported string escape.");
                    }
                } else {
                    result.push_back(c);
                }
            }
            fail("Unterminated string.");
        }
        const size_t begin = pos;
        while (pos < input.size()) {
            const char c = input[pos];
            if (std::isspace(static_cast<unsigned char>(c)) || c == '=' || c == ':' || c == ',' || c == '{' ||
                c == '}' || c == '[' || c == ']' || starts("//") || starts("/*")) {
                break;
            }
            ++pos;
        }
        if (begin == pos) {
            fail("Expected identifier or scalar.");
        }
        return std::string(input.substr(begin, pos - begin));
    }
    Value value(unsigned depth) {
        if (depth > 64 || ++count > 2'000'000) {
            fail("Text nesting/node limit exceeded.");
        }
        skip();
        if (pos >= input.size()) {
            fail("Expected value.");
        }
        Value out;
        out.offset = pos;
        if (input[pos] == '{' || input[pos] == '[') {
            const char close = input[pos] == '{' ? '}' : ']';
            out.kind = close == '}' ? Kind::object : Kind::array;
            ++pos;
            skip();
            std::unordered_set<std::string> keys;
            while (pos < input.size() && input[pos] != close) {
                if (out.kind == Kind::object) {
                    auto key = token();
                    skip();
                    if (pos >= input.size() || input[pos++] != '=') {
                        fail("Expected '=' after field name.");
                    }
                    if (!keys.insert(key).second) {
                        fail("Duplicate field: " + key);
                    }
                    out.keys.push_back(std::move(key));
                }
                out.children.push_back(value(depth + 1));
                skip();
                if (pos < input.size() && input[pos] == ',') {
                    ++pos;
                    skip();
                }
            }
            if (pos >= input.size()) {
                fail("Unterminated object/array.");
            }
            ++pos;
            return out;
        }
        if (starts("#[")) {
            out.kind = Kind::blob;
            pos += 2;
            const auto end = input.find(']', pos);
            if (end == std::string_view::npos) {
                fail("Unterminated hexadecimal blob.");
            }
            out.blob = input.substr(pos, end - pos);
            pos = end + 1;
            return out;
        }
        out.scalar = token();
        skip();
        if (pos < input.size() && input[pos] == ':') {
            ++pos;
            return value(depth + 1);
        }
        return out;
    }

  public:
    explicit Reader(std::string_view source) : input(source) {
        if (input.substr(0, 3) == "\xef\xbb\xbf") {
            pos = 3;
        }
    }
    Value read() {
        auto root = value(0);
        skip();
        if (pos != input.size()) {
            fail("Trailing text after root value.");
        }
        if (root.kind != Kind::object) {
            fail("Physics text root must be an object.");
        }
        return root;
    }
};
} // namespace
Value parse(std::string_view input) {
    return Reader(input).read();
}
int64_t integer(const Value &value) {
    require(value.kind == Kind::atom, CS2TL_PARSE_ERROR, "Expected integer.", value.offset);
    std::string_view text = value.scalar;
    if (!text.empty() && text.front() == '+') {
        text.remove_prefix(1);
    }
    int base = 10;
    if (text.starts_with("0x") || text.starts_with("0X")) {
        text.remove_prefix(2);
        base = 16;
    }
    int64_t out{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), out, base);
    require(result.ec == std::errc{} && result.ptr == text.data() + text.size(), CS2TL_PARSE_ERROR,
            "Invalid/out-of-range integer.", value.offset);
    return out;
}
double number(const Value &value) {
    require(value.kind == Kind::atom, CS2TL_PARSE_ERROR, "Expected numeric value.", value.offset);
    std::string_view text = value.scalar;
    if (!text.empty() && text.front() == '+') {
        text.remove_prefix(1);
    }
    double out{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), out);
    require(result.ec == std::errc{} && result.ptr == text.data() + text.size() && std::isfinite(out) &&
                std::abs(out) <= std::numeric_limits<float>::max(),
            CS2TL_PARSE_ERROR, "Invalid/nonfinite/out-of-range numeric value.", value.offset);
    return out;
}
std::vector<uint8_t> bytes(const Value &value) {
    require(value.kind == Kind::blob, CS2TL_PARSE_ERROR, "Expected hexadecimal blob.", value.offset);
    std::vector<uint8_t> out;
    int high = -1;
    for (size_t i = 0; i < value.blob.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(value.blob[i]);
        if (std::isspace(c)) {
            continue;
        }
        const int digit =
            c >= '0' && c <= '9'
                ? c - '0'
                : (c >= 'a' && c <= 'f' ? c - 'a' + 10 : (c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1));
        require(digit >= 0, CS2TL_PARSE_ERROR, "Invalid hexadecimal digit.", value.offset + 2 + i);
        if (high < 0) {
            high = digit;
        } else {
            out.push_back(static_cast<uint8_t>((high << 4) | digit));
            high = -1;
        }
    }
    require(high < 0, CS2TL_PARSE_ERROR, "Odd hexadecimal digit count.", value.offset);
    return out;
}
} // namespace cs2::traceline::text
