#include "FileIO.h"
#include "diagnostics/Error.h"
#include <algorithm>
#include <atomic>
#include <climits>
#include <fstream>
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace cs2::traceline {
std::filesystem::path utf8_path(std::string_view value) {
    require(!value.empty() && value.find('\0') == std::string_view::npos, CS2TL_INVALID_ARGUMENT,
            "Empty or invalid path.");
#if defined(_WIN32)
    require(value.size() <= INT_MAX, CS2TL_INVALID_ARGUMENT, "Path is too long.");
    const int size = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
                                         static_cast<int>(value.size()), nullptr, 0);
    require(size > 0, CS2TL_INVALID_ARGUMENT, "Path must be valid UTF-8.");
    std::wstring wide(static_cast<size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(), static_cast<int>(value.size()),
                        wide.data(), size);
    return std::filesystem::path(wide);
#else
    return std::filesystem::u8path(value);
#endif
}
std::vector<uint8_t> read_file(const std::filesystem::path &path) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    require(stream.is_open(), CS2TL_IO_ERROR, "Cannot open input file.");
    const auto end = stream.tellg();
    require(end >= 0, CS2TL_IO_ERROR, "Cannot determine input size.");
    const auto size = static_cast<uint64_t>(end);
    require(size <= max_file_bytes, CS2TL_UNSUPPORTED_FORMAT, "Input exceeds the 512 MiB limit.");
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    stream.seekg(0);
    if (size) {
        stream.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(size));
    }
    require(!stream.fail() && stream.peek() == std::char_traits<char>::eof() && !stream.bad(), CS2TL_IO_ERROR,
            "Input read failed or file changed while reading.");
    return bytes;
}
uint64_t fingerprint(std::span<const uint8_t> bytes) {
    uint64_t hash = 14695981039346656037ull;
    for (uint8_t b : bytes) {
        hash ^= b;
        hash *= 1099511628211ull;
    }
    return hash;
}
void atomic_write(const std::filesystem::path &path, std::span<const uint8_t> bytes) {
    require(!path.empty(), CS2TL_INVALID_ARGUMENT, "Output path is empty.");
#if defined(_WIN32)
    static std::atomic<uint64_t> sequence{0};
    std::filesystem::path temporary;
    HANDLE handle = INVALID_HANDLE_VALUE;
    for (unsigned attempt = 0; attempt < 32; ++attempt) {
        temporary = path;
        temporary +=
            L".tmp." + std::to_wstring(GetCurrentProcessId()) + L"." + std::to_wstring(sequence.fetch_add(1));
        handle = CreateFileW(temporary.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW, FILE_ATTRIBUTE_NORMAL,
                             nullptr);
        if (handle != INVALID_HANDLE_VALUE) {
            break;
        }
        require(GetLastError() == ERROR_FILE_EXISTS || GetLastError() == ERROR_ALREADY_EXISTS, CS2TL_IO_ERROR,
                "Cannot create temporary output file.");
    }
    require(handle != INVALID_HANDLE_VALUE, CS2TL_IO_ERROR, "Cannot reserve temporary output file.");
    struct Cleanup {
        HANDLE handle;
        std::filesystem::path path;
        ~Cleanup() {
            if (handle != INVALID_HANDLE_VALUE) {
                CloseHandle(handle);
            }
            if (!path.empty()) {
                DeleteFileW(path.c_str());
            }
        }
    } cleanup{handle, temporary};
    size_t offset = 0;
    while (offset < bytes.size()) {
        const DWORD amount = static_cast<DWORD>(std::min<size_t>(bytes.size() - offset, 1u << 20));
        DWORD written = 0;
        require(WriteFile(handle, bytes.data() + offset, amount, &written, nullptr) != 0 && written == amount,
                CS2TL_IO_ERROR, "Output write failed.");
        offset += written;
    }
    require(FlushFileBuffers(handle) != 0, CS2TL_IO_ERROR, "Cannot flush output file.");
    const BOOL closed = CloseHandle(handle);
    cleanup.handle = INVALID_HANDLE_VALUE;
    require(closed != 0, CS2TL_IO_ERROR, "Cannot close output file.");
    require(
        MoveFileExW(temporary.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0,
        CS2TL_IO_ERROR, "Cannot commit output file.");
    cleanup.path.clear();
#else
    // Non-Windows builds are useful for the core tests; Windows is the DLL target.
    auto temporary = path;
    temporary += ".tmp";
    try {
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        require(out.is_open(), CS2TL_IO_ERROR, "Cannot create output file.");
        out.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        out.close();
        require(!out.fail(), CS2TL_IO_ERROR, "Cannot write output file.");
        std::filesystem::rename(temporary, path);
    } catch (...) {
        std::error_code ec;
        std::filesystem::remove(temporary, ec);
        throw;
    }
#endif
}
} // namespace cs2::traceline
