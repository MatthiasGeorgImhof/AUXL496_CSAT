#ifndef LINUX_MOCK_I2C_FLASH_ACCESSOR_H
#define LINUX_MOCK_I2C_FLASH_ACCESSOR_H

#include <cstdint>
#include <cstddef>
#include <iostream>
#include <cstring>
#include "mock_hal.h"
#include "imagebuffer/accessor.hpp"

class LinuxMockI2CFlashAccessor
{
public:
    LinuxMockI2CFlashAccessor() = delete;

    LinuxMockI2CFlashAccessor(I2C_HandleTypeDef *hi2c, size_t flash_start, size_t total_size)
        : hi2c_(hi2c), FLASH_START_ADDRESS(flash_start), TOTAL_BUFFER_SIZE(total_size)
    {
        flash_memory.resize(TOTAL_BUFFER_SIZE, 0xff);
    }

    AccessorError write(size_t address, const uint8_t *data, size_t size);
    AccessorError read(size_t address, uint8_t *data, size_t size);
    AccessorError erase(size_t address);

    size_t getAlignment() const { return 1; };
    size_t getFlashMemorySize() const { return TOTAL_BUFFER_SIZE; };
    size_t getFlashStartAddress() const { return FLASH_START_ADDRESS; };

    std::vector<uint8_t> &getFlashMemory() { return flash_memory; }

private:
    AccessorError checkBounds(size_t address, size_t size);

private:
    I2C_HandleTypeDef *hi2c_;
    const size_t FLASH_START_ADDRESS;
    const size_t TOTAL_BUFFER_SIZE;
    std::vector<uint8_t> flash_memory;
};

AccessorError LinuxMockI2CFlashAccessor::write(size_t address, const uint8_t *data, size_t size)
{
    // Check bounds
    if (checkBounds(address, size) != AccessorError::NO_ERROR)
    {
        return AccessorError::OUT_OF_BOUNDS; // Return error if out of bounds
    }

    uint16_t offset = static_cast<uint16_t>(address - FLASH_START_ADDRESS);
    // Now, call the mocked HAL_I2C_Mem_Write function to simulate the I2C write
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c_, 0xA0, offset, 2, (uint8_t *)data, static_cast<uint16_t>(size), 100);
    if (status != HAL_OK)
    {
        std::cerr << "I2C Write Failed (Mock HAL)" << std::endl;
        return AccessorError::WRITE_ERROR; // Return error if write fails
    }

    // Perform the memory copy to the simulated flash
    std::memcpy(flash_memory.data() + offset, data, size);

    return AccessorError::NO_ERROR; // Return success
}

AccessorError LinuxMockI2CFlashAccessor::read(size_t address, uint8_t *data, size_t size)
{
    // Check bounds
    if (checkBounds(address, size) != AccessorError::NO_ERROR)
    {
        return AccessorError::OUT_OF_BOUNDS; // Return error if out of bounds
    }

    uint16_t offset = static_cast<uint16_t>(address - FLASH_START_ADDRESS);
    // Now, call the mocked HAL_I2C_Mem_Read function to simulate the I2C read
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c_, 0xA0, offset, 2, (uint8_t *)data, static_cast<uint16_t>(size), 100);
    if (status != HAL_OK)
    {
        std::cerr << "I2C Read Failed (Mock HAL)" << std::endl;
        return AccessorError::OUT_OF_BOUNDS;
    }

    // Perform the memory copy from the simulated flash
    std::memcpy(data, flash_memory.data() + offset, size);

    return AccessorError::NO_ERROR; // Return success
}

AccessorError LinuxMockI2CFlashAccessor::erase(size_t /*address*/)
{
    // Implement the flash erase sequence using the HAL mocks
    // For now, just return success.  In a real implementation, you would need
    // to simulate the flash erase operation, possibly by writing 0xFF to the
    // affected memory region.
    std::fill(flash_memory.begin(), flash_memory.end(), 0xFF); // Simulate erasing by filling with 0xFF
    return AccessorError::NO_ERROR;                              // Success
}

AccessorError LinuxMockI2CFlashAccessor::checkBounds(size_t address, size_t size)
{
    if (address < FLASH_START_ADDRESS || address + size > FLASH_START_ADDRESS + TOTAL_BUFFER_SIZE)
    {
        std::cerr << "Error: Access out of bounds. Address: 0x" << std::hex << address
                  << ", Size: " << size << std::dec << std::endl; // Added address and size for debugging
        return AccessorError::OUT_OF_BOUNDS;                        // Return error
    }
    return AccessorError::NO_ERROR; // Return success
}

static_assert(Accessor<LinuxMockI2CFlashAccessor>, "LinuxMockI2CFlashAccessor does not satisfy the Accessor concept!");

#endif /* LinuxMockI2CFlashAccess */