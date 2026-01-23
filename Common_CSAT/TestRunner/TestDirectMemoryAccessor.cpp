#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "imagebuffer/accessor.hpp"
#include "imagebuffer/DirectMemoryAccessor.hpp"
#include "mock_hal.h"
#include <iostream>
#include <vector> // Include vector header

// Helper function to compare memory regions
bool compareMemory(const uint8_t *mem1, const uint8_t *mem2, size_t size)
{
    return std::memcmp(mem1, mem2, size) == 0;
}

// Test case for DirectMemoryAccessor
TEST_CASE("DirectMemoryAccessor")
{
    const size_t FLASH_START = 0x08000000;
    const size_t FLASH_SIZE = 1024;
    DirectMemoryAccessor dma(FLASH_START, FLASH_SIZE);

    SUBCASE("Write and Read within bounds")
    {
        size_t address = FLASH_START + 10;
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        size_t size = sizeof(data);
        std::vector<uint8_t> read_data(size); // Use std::vector

        REQUIRE(dma.write(address, data, size) == AccessorError::NO_ERROR);
        REQUIRE(dma.read(address, read_data.data(), size) == AccessorError::NO_ERROR); // Pass the data pointer
        REQUIRE(compareMemory(data, read_data.data(), size));                        // Pass the data pointer
    }

    SUBCASE("Write out of bounds")
    {
        size_t address = FLASH_START + FLASH_SIZE;
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        size_t size = sizeof(data);

        REQUIRE(dma.write(address, data, size) == AccessorError::OUT_OF_BOUNDS);
    }

    SUBCASE("Read out of bounds")
    {
        size_t address = FLASH_START + FLASH_SIZE;
        std::vector<uint8_t> data(4); // Fixed size
        size_t size = data.size();

        REQUIRE(dma.read(address, data.data(), size) == AccessorError::OUT_OF_BOUNDS);
    }

    SUBCASE("Erase (simulated)")
    {
        size_t address = FLASH_START + 10;
        REQUIRE(dma.erase(address) == AccessorError::NO_ERROR); // Simply checks that the function runs without error
    }
}
