//direct_memory_access.cpp
#include "imagebuffer/DirectMemoryAccess.hpp"
#include <iostream>
#include <cstring>

//Define the static memory region so it can be written or read
uint8_t DirectMemoryAccess::static_flash_memory[STATIC_BUFFER_SIZE];

DirectMemoryAccess::DirectMemoryAccess() {}

int32_t DirectMemoryAccess::write(uint32_t address, const uint8_t* data, size_t size) {
    // Check if the access is within bounds using base class method
    if (checkBounds(address, size) != 0) {
        return -1; // Return error if out of bounds
    }
    // Perform the memory copy
    std::memcpy(static_flash_memory + address, data, size);
    return 0; // Return success
}

int32_t DirectMemoryAccess::read(uint32_t address, uint8_t* data, size_t size) {
    // Check if the access is within bounds using base class method
    if (checkBounds(address, size) != 0) {
        return -1; // Return error if out of bounds
    }
    // Perform the memory copy
    std::memcpy(data, static_flash_memory + address, size);
    return 0; // Return success
}

int32_t DirectMemoryAccess::erase(uint32_t address) {
    // Simulate erasing a sector (e.g., by setting all bytes in the sector to 0xFF)
    // Implement sector size and erase logic here
    std::cout << "Erasing sector at address: " << address << std::endl;
    return 0; // Return success
}

int32_t DirectMemoryAccess::checkBounds(uint32_t address, size_t size) {
    if (address + size > STATIC_BUFFER_SIZE) {
        std::cerr << "Error: Access out of bounds." << std::endl;
        return -1; // Return error
    }
    return 0; // Return success
}