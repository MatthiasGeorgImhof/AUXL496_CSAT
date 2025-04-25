// DirectMemoryAccess.hpp
#ifndef DIRECT_MEMORY_ACCESS_H
#define DIRECT_MEMORY_ACCESS_H

#include <cstdint>
#include <cstddef>
#include <iostream>
#include <cstring>
#include <vector>
#include "imagebuffer/access.hpp"

class DirectMemoryAccess
{
public:
    DirectMemoryAccess(size_t flash_start, size_t total_size)
        : FLASH_START_ADDRESS(flash_start), TOTAL_BUFFER_SIZE(total_size)
    {
        flash_memory.resize(TOTAL_BUFFER_SIZE, 0); // Allocate and zero-initialize the buffer
    }

    AccessError write(uint32_t address, const uint8_t *data, size_t size);
    AccessError read(uint32_t address, uint8_t *data, size_t size);
    AccessError erase(uint32_t address);
    
    std::vector<uint8_t> &getFlashMemory() { return flash_memory; }
    size_t getFlashMemorySize() const { return TOTAL_BUFFER_SIZE; };
    size_t getFlashStartAddress() const { return FLASH_START_ADDRESS; };

private:
    AccessError checkBounds(uint32_t address, size_t size);

private:
    const size_t FLASH_START_ADDRESS; // Make it a class member
    const size_t TOTAL_BUFFER_SIZE;   // Make it a class member
    std::vector<uint8_t> flash_memory;  // Use a vector for dynamic allocation
};

AccessError DirectMemoryAccess::write(uint32_t address, const uint8_t *data, size_t size)
{
    // Check if the access is within bounds using base class method
    if (checkBounds(address, size) != AccessError::NO_ERROR)
    {
        return AccessError::OUT_OF_BOUNDS;
    }

    size_t offset = address - FLASH_START_ADDRESS;

    // Perform the memory copy
    std::memcpy(flash_memory.data() + offset, data, size);
    return AccessError::NO_ERROR;
}

AccessError DirectMemoryAccess::read(uint32_t address, uint8_t *data, size_t size)
{
    // Check if the access is within bounds using base class method
    if (checkBounds(address, size) != AccessError::NO_ERROR)
    {
        return AccessError::OUT_OF_BOUNDS; // Return error if out of bounds
    }

    size_t offset = address - FLASH_START_ADDRESS;
    // Perform the memory copy
    std::memcpy(data, flash_memory.data() + offset, size);
    return AccessError::NO_ERROR; // Return success
}

AccessError DirectMemoryAccess::erase(uint32_t /*address*/)
{
    // Simulate erasing a sector (e.g., by setting all bytes in the sector to 0xFF)
    // Implement sector size and erase logic here
    std::fill(flash_memory.begin(), flash_memory.end(), 0xFF); // Simulate erasing by filling with 0xFF
    return AccessError::NO_ERROR;                             // Success
}

AccessError DirectMemoryAccess::checkBounds(uint32_t address, size_t size)
{
    if (address < FLASH_START_ADDRESS || address + size > FLASH_START_ADDRESS + TOTAL_BUFFER_SIZE)
    {
        std::cerr << "Error: Access out of bounds. Address: 0x" << std::hex << address
                  << ", Size: " << size << std::dec << std::endl; // Added address and size for debugging
        return AccessError::OUT_OF_BOUNDS;
    }
    return AccessError::NO_ERROR; // Return success
}

#endif