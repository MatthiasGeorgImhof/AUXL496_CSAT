#ifndef CONFIGURABLE_MEMORY_ACCESSOR_HPP
#define CONFIGURABLE_MEMORY_ACCESSOR_HPP

#include <vector>
#include <cstdint>
#include <cstring>
#include "imagebuffer/accessor.hpp"

class ConfigurableMemoryAccessor
{
public:
    ConfigurableMemoryAccessor(size_t flash_start,
                               size_t flash_size,
                               size_t erase_block_size)
        : flash_start_(flash_start),
          flash_size_(flash_size),
          erase_block_size_(erase_block_size),
          mem_(flash_size, 0xFF) // erased state
    {
    }

    // --- Accessor concept API --------------------------------
    AccessorError write(size_t address, const uint8_t *data, size_t size)
    {
        if (!in_range(address, size))
            return AccessorError::OUT_OF_BOUNDS;

        const size_t offset = address - flash_start_;
        std::memcpy(&mem_[offset], data, size);
        return AccessorError::NO_ERROR;
    }

    AccessorError read(size_t address, uint8_t *data, size_t size)
    {
        if (!in_range(address, size))
            return AccessorError::OUT_OF_BOUNDS;

        const size_t offset = address - flash_start_;
        std::memcpy(data, &mem_[offset], size);
        return AccessorError::NO_ERROR;
    }

    AccessorError erase(size_t address)
    {
        if (address < flash_start_)
            return AccessorError::OUT_OF_BOUNDS;

        size_t offset = address - flash_start_;
        if (offset >= flash_size_)
            return AccessorError::OUT_OF_BOUNDS;

        // Align to erase block
        const size_t block_index = offset / erase_block_size_;
        const size_t block_start = block_index * erase_block_size_;
        const size_t block_end = block_start + erase_block_size_;

        if (block_end > flash_size_)
            return AccessorError::OUT_OF_BOUNDS;

        using diff_t = std::vector<uint8_t>::iterator::difference_type;

        auto begin_it = mem_.begin() + static_cast<diff_t>(block_start);
        auto end_it = mem_.begin() + static_cast<diff_t>(block_end);

        std::fill(begin_it, end_it, 0xFF);

        return AccessorError::NO_ERROR;
    }

    size_t getAlignment() const { return erase_block_size_; }
    size_t getFlashMemorySize() const { return flash_size_; }
    size_t getFlashStartAddress() const { return flash_start_; }
    size_t getEraseBlockSize() const { return erase_block_size_; }

    // For tests to inspect memory
    std::vector<uint8_t> &raw() { return mem_; }
    const std::vector<uint8_t> &raw() const { return mem_; }

private:
    bool in_range(size_t address, size_t size) const
    {
        if (address < flash_start_)
            return false;
        const size_t offset = address - flash_start_;
        return offset + size <= flash_size_;
    }

    size_t flash_start_;
    size_t flash_size_;
    size_t erase_block_size_;
    std::vector<uint8_t> mem_;
};

static_assert(Accessor<ConfigurableMemoryAccessor>,
              "ConfigurableMemoryAccessor does not satisfy Accessor concept");

#endif // CONFIGURABLE_MEMORY_ACCESSOR_HPP
