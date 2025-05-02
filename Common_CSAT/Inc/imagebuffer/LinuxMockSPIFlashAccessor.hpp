#ifndef LINUX_MOCK_SPI_FLASH_ACCESSOR_H
#define LINUX_MOCK_SPI_FLASH_ACCESSOR_H

#include <cstdint>
#include <cstddef>
#include <iostream>
#include <cstring>
#include <vector>
#include "mock_hal.h"
#include "imagebuffer/accessor.hpp"

class LinuxMockSPIFlashAccessor
{
public:
    LinuxMockSPIFlashAccessor() = delete;

    LinuxMockSPIFlashAccessor(SPI_HandleTypeDef *hspi, size_t flash_start, size_t total_size)
        : hspi_(hspi), FLASH_START_ADDRESS(flash_start), TOTAL_BUFFER_SIZE(total_size)
    {
        flash_memory.resize(TOTAL_BUFFER_SIZE, 0xFF); // Initialize with 0xFF (erased state)
    }

    AccessorError write(uint32_t address, const uint8_t *data, size_t size);
    AccessorError read(uint32_t address, uint8_t *data, size_t size);
    AccessorError erase(uint32_t address); // Erase a sector
    AccessorError full_erase();            // Erase entire flash memory

    size_t getAlignment() const { return 1; };
    size_t getFlashMemorySize() const { return TOTAL_BUFFER_SIZE; };
    size_t getFlashStartAddress() const { return FLASH_START_ADDRESS; };

    std::vector<uint8_t> &getFlashMemory() { return flash_memory; }

private:
    AccessorError checkBounds(uint32_t address, size_t size);

private:
    SPI_HandleTypeDef *hspi_;
    const size_t FLASH_START_ADDRESS;
    const size_t TOTAL_BUFFER_SIZE;
    std::vector<uint8_t> flash_memory;
};

AccessorError LinuxMockSPIFlashAccessor::write(uint32_t address, const uint8_t *data, size_t size)
{
    // Check bounds
    if (checkBounds(address, size) != AccessorError::NO_ERROR)
    {
        return AccessorError::OUT_OF_BOUNDS; // Return error if out of bounds
    }

    size_t offset = address - FLASH_START_ADDRESS;

    // Call the mocked HAL_SPI_Transmit function to simulate the SPI write
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi_, (uint8_t *)data, static_cast<uint16_t>(size), 100); // Cast away const for the mock
    if (status != HAL_OK)
    {
        std::cerr << "SPI Write Failed (Mock HAL)" << std::endl;
        return AccessorError::WRITE_ERROR; // Return error if write fails
    }

    // Perform the memory copy to the simulated flash
    std::memcpy(flash_memory.data() + offset, data, size);

    return AccessorError::NO_ERROR; // Return success
}

AccessorError LinuxMockSPIFlashAccessor::read(uint32_t address, uint8_t *data, size_t size)
{
    // Check bounds
    if (checkBounds(address, size) != AccessorError::NO_ERROR)
    {
        return AccessorError::OUT_OF_BOUNDS; // Return error if out of bounds
    }

    size_t offset = address - FLASH_START_ADDRESS;

    // Call the mocked HAL_SPI_Receive function to simulate the SPI read
    HAL_StatusTypeDef status = HAL_SPI_Receive(hspi_, data, static_cast<uint16_t>(size), 100);
    if (status != HAL_OK)
    {
        std::cerr << "SPI Read Failed (Mock HAL)" << std::endl;
        return AccessorError::READ_ERROR; // Return error if read fails
    }

    // Perform the memory copy from the simulated flash
    std::memcpy(data, flash_memory.data() + offset, size);

    return AccessorError::NO_ERROR; // Return success
}

AccessorError LinuxMockSPIFlashAccessor::erase(uint32_t address)
{
    if (checkBounds(address, 1) != AccessorError::NO_ERROR)
    {
        return AccessorError::OUT_OF_BOUNDS;
    }

    size_t offset = address - FLASH_START_ADDRESS;
    // Simulate Sector Erase (usually 4KB)
    size_t start_of_sector = offset - (offset % 4096); // Assuming 4KB sectors
    for (size_t i = start_of_sector; i < start_of_sector + 4096 && i < TOTAL_BUFFER_SIZE; ++i)
    {
        flash_memory[i] = 0xFF; // Erase to 0xFF
    }
    return AccessorError::NO_ERROR;
}

AccessorError LinuxMockSPIFlashAccessor::full_erase()
{
    std::fill(flash_memory.begin(), flash_memory.end(), 0xFF);
    return AccessorError::NO_ERROR;
}

AccessorError LinuxMockSPIFlashAccessor::checkBounds(uint32_t address, size_t size)
{
    if (address < FLASH_START_ADDRESS || address + size > FLASH_START_ADDRESS + TOTAL_BUFFER_SIZE)
    {
        std::cerr << "Error: Access out of bounds. Address: 0x" << std::hex << address
                  << ", Size: " << size << std::dec << std::endl; // Added address and size for debugging
        return AccessorError::OUT_OF_BOUNDS;                        // Return error
    }
    return AccessorError::NO_ERROR; // Return success
}

static_assert(Accessor<LinuxMockSPIFlashAccessor>, "LinuxMockSPIFlashAccessor does not satisfy the Accessor concept!");

#endif /* LINUX_MOCK_SPI_FLASH_ACCESSOR_H */