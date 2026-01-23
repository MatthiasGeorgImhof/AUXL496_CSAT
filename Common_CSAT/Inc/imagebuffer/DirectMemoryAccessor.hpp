// DirectMemoryAccessor.hpp
#ifndef DIRECT_MEMORY_ACCESSOR_H
#define DIRECT_MEMORY_ACCESSOR_H

#include <cstdint>
#include <cstddef>
#include <iostream>
#include <cstring>
#include <vector>
#include "imagebuffer/accessor.hpp"

class DirectMemoryAccessor
{
public:
    DirectMemoryAccessor(size_t flash_start, size_t total_size)
        : FLASH_START_ADDRESS(flash_start), TOTAL_BUFFER_SIZE(total_size)
    {
        flash_memory.resize(TOTAL_BUFFER_SIZE, 0); // Allocate and zero-initialize the buffer
    }

    AccessorError write(size_t address, const uint8_t *data, size_t size);
    AccessorError read(size_t address, uint8_t *data, size_t size);
    AccessorError erase(size_t address);
    void format(); 

    size_t getAlignment() const { return 1; };
    size_t getFlashMemorySize() const { return TOTAL_BUFFER_SIZE; };
    size_t getFlashStartAddress() const { return FLASH_START_ADDRESS; };
    size_t getEraseBlockSize() const { return 1; } 

    std::vector<uint8_t> &getFlashMemory() { return flash_memory; }

private:
    AccessorError checkBounds(size_t address, size_t size);

private:
    const size_t FLASH_START_ADDRESS;  // Make it a class member
    const size_t TOTAL_BUFFER_SIZE;    // Make it a class member
    std::vector<uint8_t> flash_memory; // Use a vector for dynamic allocation
};

AccessorError DirectMemoryAccessor::write(size_t address, const uint8_t *data, size_t size)
{
    // Check if the access is within bounds using base class method
    if (checkBounds(address, size) != AccessorError::NO_ERROR)
    {
        return AccessorError::OUT_OF_BOUNDS;
    }

    size_t offset = address - FLASH_START_ADDRESS;

    // Perform the memory copy
    std::memcpy(flash_memory.data() + offset, data, size);
    return AccessorError::NO_ERROR;
}

AccessorError DirectMemoryAccessor::read(size_t address, uint8_t *data, size_t size)
{
    // Check if the access is within bounds using base class method
    if (checkBounds(address, size) != AccessorError::NO_ERROR)
    {
        return AccessorError::OUT_OF_BOUNDS; // Return error if out of bounds
    }

    size_t offset = address - FLASH_START_ADDRESS;
    // Perform the memory copy
    std::memcpy(data, flash_memory.data() + offset, size);
    return AccessorError::NO_ERROR; // Return success
}

AccessorError DirectMemoryAccessor::erase(size_t address)
{
    const size_t block_size = getEraseBlockSize();
    const size_t offset = address - FLASH_START_ADDRESS;

    if (offset >= TOTAL_BUFFER_SIZE)
        return AccessorError::OUT_OF_BOUNDS;

    const size_t end = std::min(offset + block_size, TOTAL_BUFFER_SIZE);

    using diff_t = std::vector<uint8_t>::iterator::difference_type;

    auto begin_it = flash_memory.begin() + static_cast<diff_t>(offset);
    auto end_it   = flash_memory.begin() + static_cast<diff_t>(end);

    std::fill(begin_it, end_it, 0xFF);

    return AccessorError::NO_ERROR;
}

void DirectMemoryAccessor::format() {
    std::fill(flash_memory.begin(), flash_memory.end(), 0xFF);
}

AccessorError DirectMemoryAccessor::checkBounds(size_t address, size_t size)
{
    const size_t start = FLASH_START_ADDRESS;
    const size_t end   = FLASH_START_ADDRESS + TOTAL_BUFFER_SIZE;

    if (address < start || address + size > end)
    {
        return AccessorError::OUT_OF_BOUNDS;
    }
    return AccessorError::NO_ERROR;
}

static_assert(Accessor<DirectMemoryAccessor>, "DirectMemoryAccessor does not satisfy the Accessor concept!");

#endif /* DIRECT_MEMORY_ACCESSOR_H */