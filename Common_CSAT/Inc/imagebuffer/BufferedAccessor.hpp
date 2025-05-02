#ifndef BUFFERED_ACCESSOR_H
#define BUFFERED_ACCESSOR_H

#include "imagebuffer/accessor.hpp"
#include <algorithm>

template <typename BaseAccessor, size_t BLOCK_SIZE>
class BufferedAccessor
{
public:
    BufferedAccessor(BaseAccessor &base_access)
        : base_access_(base_access), cache_dirty_(false), cache_address_(0) {}
    
    ~BufferedAccessor()
    {
        if(cache_dirty_)
        {
            flush_cache();
        }
    }

    AccessorError write(uint32_t address, const uint8_t *data, size_t size);
    AccessorError read(uint32_t address, uint8_t *data, size_t size);
    AccessorError erase(uint32_t address);

    size_t getAlignment() const { return BLOCK_SIZE; };
    size_t getFlashMemorySize() const { return base_access_.getFlashMemorySize(); };
    size_t getFlashStartAddress() const { return base_access_.getFlashStartAddress(); };
    AccessorError flush_cache();

private:
    AccessorError fill_cache(uint32_t address);

private:
    BaseAccessor &base_access_;
    uint8_t cache_[BLOCK_SIZE];
    bool cache_dirty_;       // Indicates if the cache has been modified
    uint32_t cache_address_; // Starting address of the cached block
};

template <typename BaseAccessor, size_t BLOCK_SIZE>
AccessorError BufferedAccessor<BaseAccessor, BLOCK_SIZE>::flush_cache()
{
    if (cache_dirty_)
    {
        AccessorError result = base_access_.write(cache_address_, cache_, BLOCK_SIZE);
        if (result != AccessorError::NO_ERROR)
        {
            return result;
        }
        cache_dirty_ = false;
    }
    return AccessorError::NO_ERROR;
}

template <typename BaseAccessor, size_t BLOCK_SIZE>
AccessorError BufferedAccessor<BaseAccessor, BLOCK_SIZE>::fill_cache(uint32_t address)
{
    AccessorError result = flush_cache();
    if (result != AccessorError::NO_ERROR)
    {
        return result;
    }

    cache_address_ = address - (address % BLOCK_SIZE); // Block align the address
    result = base_access_.read(cache_address_, cache_, BLOCK_SIZE);
    if (result != AccessorError::NO_ERROR)
    {
        return result;
    }
    return AccessorError::NO_ERROR;
}

template <typename BaseAccessor, size_t BLOCK_SIZE>
AccessorError BufferedAccessor<BaseAccessor, BLOCK_SIZE>::write(uint32_t address, const uint8_t *data, size_t size)
{
    size_t data_offset = 0;
    while (size > 0)
    {
        size_t block_offset = address % BLOCK_SIZE;
        uint32_t block_start_address = address - block_offset;
        uint32_t block_end_address = block_start_address + BLOCK_SIZE - 1;
        uint32_t block_remaining_size = block_end_address - address + 1;
        size_t done = std::min(size, static_cast<size_t>(block_remaining_size));

        if (block_start_address != cache_address_)
        {
            AccessorError flush_result = flush_cache();
            if (flush_result != AccessorError::NO_ERROR)
            {
                return flush_result;
            }
            AccessorError fill_result = fill_cache(address);
            if (fill_result != AccessorError::NO_ERROR)
            {
                return fill_result;
            }
        }

        std::memcpy(cache_ + block_offset, data + data_offset, done);
        cache_dirty_ = true;

        data_offset += done;
        address += done;
        size -= done;
    }
    return AccessorError::NO_ERROR;
}

template <typename BaseAccessor, size_t BLOCK_SIZE>
AccessorError BufferedAccessor<BaseAccessor, BLOCK_SIZE>::read(uint32_t address, uint8_t *data, size_t size)
{
    size_t data_offset = 0;
    while (size > 0)
    {
        size_t block_offset = address % BLOCK_SIZE;
        uint32_t block_start_address = address - block_offset;
        uint32_t block_end_address = block_start_address + BLOCK_SIZE - 1;
        uint32_t block_remaining_size = block_end_address - address + 1;
        size_t done = std::min(size, static_cast<size_t>(block_remaining_size));

        if (block_start_address != cache_address_)
        {
            AccessorError fill_result = fill_cache(address);
            if (fill_result != AccessorError::NO_ERROR)
            {
                return fill_result;
            }
        }

        std::memcpy(data + data_offset, cache_ + block_offset, done);
        data_offset += done;
        address += done;
        size -= done;
    }
    return AccessorError::NO_ERROR;
}

template <typename BaseAccessor, size_t BLOCK_SIZE>
AccessorError BufferedAccessor<BaseAccessor, BLOCK_SIZE>::erase(uint32_t address)
{
    // Easiest implementation: Flush cache and then erase directly.
    AccessorError flush_result = flush_cache();
    if (flush_result != AccessorError::NO_ERROR)
    {
        return flush_result;
    }
    return base_access_.erase(address);
}

namespace BUFFERED_ACCESSOR_H
{
    class __MockAccessor__
    {
    public:
        __MockAccessor__() {}

        AccessorError write(uint32_t /*address*/, const uint8_t */*data*/, size_t /*size*/) { return AccessorError::NO_ERROR; };
        AccessorError read(uint32_t /*address*/, uint8_t */*data*/, size_t /*size*/) { return AccessorError::NO_ERROR; };
        AccessorError erase(uint32_t /*address*/) { return AccessorError::NO_ERROR; };

        size_t getAlignment() const { return 0; }
        size_t getFlashMemorySize() const { return 0; }
        size_t getFlashStartAddress() const { return 0; }
    };

    static_assert(Accessor<__MockAccessor__>, "__MockAccessor__ does not satisfy the Accessor concept!");
    static_assert(Accessor<BufferedAccessor<__MockAccessor__, 512>>, "BufferedAccessor does not satisfy the Accessor concept!");
}
#endif // BUFFERED_ACCESSOR_H
