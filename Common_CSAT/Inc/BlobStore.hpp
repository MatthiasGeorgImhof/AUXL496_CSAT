#ifndef __BLOBSTORE_HPP__
#define __BLOBSTORE_HPP__

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string_view>
#include <type_traits>
#include <bit>
#include <concepts>
#include <array>
#include <iostream>
#include <span>

// --------------------
// 📐 BlobStoreAccess Concept
// --------------------
template <typename T>
concept BlobStoreAccess = requires(T access, size_t offset, uint8_t* buffer, size_t buffer_size, const uint8_t* const_buffer, size_t const_buffer_size) {
    { access.read(offset, buffer, buffer_size) } -> std::same_as<bool>;
    { access.write(offset, const_buffer, const_buffer_size) } -> std::same_as<bool>;
    { access.get_flash_size() } -> std::convertible_to<size_t>;
};

// ---------------------------
// 💾 File-based Flash Backend
// ---------------------------
class FileBlobStoreAccess {
public:
    FileBlobStoreAccess(const std::string_view& filename, size_t flash_size)
        : filename_(std::string(filename)), flash_size_(flash_size), is_valid_(initialize_flash()) {}

    bool read(size_t offset, uint8_t* buffer, size_t size) const {
        if (!is_valid_ || offset + size > flash_size_) return false;
        std::ifstream file(filename_, std::ios::binary);
        if (!file) return false;
        file.seekg(static_cast<std::streamoff>(offset));
        file.read(reinterpret_cast<char*>(buffer), static_cast<std::streamsize>(size));
        return file.gcount() == static_cast<std::streamsize>(size);
    }

    bool write(size_t offset, const uint8_t* data, size_t size) {
        if (!is_valid_ || offset + size > flash_size_) return false;
        std::fstream file(filename_, std::ios::binary | std::ios::in | std::ios::out);
        if (!file) return false;
        file.seekp(static_cast<std::streamoff>(offset));
        file.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
        return !file.fail();
    }

    size_t get_flash_size() const { return flash_size_; }
    bool isValid() const { return is_valid_; }

private:
    bool initialize_flash() {
        std::ofstream file(filename_, std::ios::binary);
        if (!file) return false;
        file.seekp(static_cast<std::streamoff>(flash_size_ - 1));
        file.write("\0", 1);
        return !file.fail();
    }

    std::string filename_;
    size_t flash_size_;
    bool is_valid_;
};

// ----------------------------------
// 🔌 RAM-Emulated SPI Flash Backend
// ----------------------------------
class SPIBlobStoreAccess {
public:
    SPIBlobStoreAccess(size_t flash_size, uint8_t* memory)
        : flash_size_(flash_size), spi_memory_(memory), is_valid_(memory != nullptr) {}

    bool read(size_t offset, uint8_t* buffer, size_t size) const {
        if (!is_valid_ || offset + size > flash_size_) return false;
        std::memcpy(buffer, spi_memory_ + offset, size);
        return true;
    }

    bool write(size_t offset, const uint8_t* data, size_t size) {
        if (!is_valid_ || offset + size > flash_size_) return false;
        std::memcpy(spi_memory_ + offset, data, size);
        return true;
    }

    size_t get_flash_size() const { return flash_size_; }
    bool isValid() const { return is_valid_; }

private:
    size_t flash_size_;
    uint8_t* spi_memory_;
    bool is_valid_;
};

// ------------------------------
// 📦 Generic BlobStore Interface
// ------------------------------
template <BlobStoreAccess AccessType, typename BlobStruct>
class BlobStore {
public:
    explicit BlobStore(AccessType access) : access_(access) {}

    bool write_blob(const uint8_t* data, size_t data_size, size_t offset, size_t array_size) {
        if (data_size > array_size) return false;
        return access_.write(offset, data, data_size);
    }

    bool read_blob(uint8_t* buffer, size_t buffer_size, size_t offset, size_t array_size) {
        if (buffer_size < array_size) return false;
        return access_.read(offset, buffer, array_size);
    }

protected:
    AccessType access_;
};

// -----------------------------------
// 🏷️ Named BlobStore 
// -----------------------------------
struct BlobMemberInfo {
    std::string_view name;
    size_t offset;
    size_t size;
};

template <BlobStoreAccess AccessType, typename BlobStruct, typename MemberInfo, size_t MapSize>
class NamedBlobStore : public BlobStore<AccessType, BlobStruct> {
public:
    using BlobStore<AccessType, BlobStruct>::BlobStore;

    NamedBlobStore(AccessType access, const std::array<MemberInfo, MapSize>& blob_map)
        : BlobStore<AccessType, BlobStruct>(access), blob_map_(blob_map) {}

    bool write_blob_by_name(const std::string_view& name, const uint8_t* data, size_t data_size) {
        for (const auto& entry : blob_map_) {
            if (entry.name == name) {
                return this->write_blob(data, data_size, entry.offset, entry.size);
            }
        }
        return false;
    }

    std::span<uint8_t> read_blob_by_name(const std::string_view& name, uint8_t* buffer, size_t buffer_size) {
        for (const auto& entry : blob_map_) {
            if (entry.name == name) {
                if (this->read_blob(buffer, buffer_size, entry.offset, entry.size)) {
                    return std::span<uint8_t>(buffer, entry.size);
                } else {
                    return std::span<uint8_t>(buffer, 0);
                }
            }
        }
        return std::span<uint8_t>(buffer, 0);
    }

    bool direct_read_blob(uint8_t* buffer, size_t buffer_size, size_t offset, size_t array_size) {
        return this->read_blob(buffer, buffer_size, offset, array_size);
    }

    bool direct_write_blob(const uint8_t* data, size_t data_size, size_t offset, size_t array_size) {
        return this->write_blob(data, data_size, offset, array_size);
    }

private:
    const std::array<MemberInfo, MapSize>& blob_map_;
};

#endif // __BLOBSTORE_HPP__