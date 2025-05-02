#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "imagebuffer/accessor.hpp"
#include "imagebuffer/DirectMemoryAccessor.hpp"
#include "imagebuffer/LinuxMockI2CFlashAccessor.hpp"
#include "imagebuffer/LinuxMockSPIFlashAccessor.hpp"
#include "imagebuffer/BufferedAccessor.hpp"
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
    const uint32_t FLASH_START = 0x08000000;
    const uint32_t FLASH_SIZE = 1024;
    DirectMemoryAccessor dma(FLASH_START, FLASH_SIZE);

    SUBCASE("Write and Read within bounds")
    {
        uint32_t address = FLASH_START + 10;
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        size_t size = sizeof(data);
        std::vector<uint8_t> read_data(size); // Use std::vector

        REQUIRE(dma.write(address, data, size) == AccessorError::NO_ERROR);
        REQUIRE(dma.read(address, read_data.data(), size) == AccessorError::NO_ERROR); // Pass the data pointer
        REQUIRE(compareMemory(data, read_data.data(), size));                        // Pass the data pointer
    }

    SUBCASE("Write out of bounds")
    {
        uint32_t address = FLASH_START + FLASH_SIZE;
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        size_t size = sizeof(data);

        REQUIRE(dma.write(address, data, size) == AccessorError::OUT_OF_BOUNDS);
    }

    SUBCASE("Read out of bounds")
    {
        uint32_t address = FLASH_START + FLASH_SIZE;
        std::vector<uint8_t> data(4); // Fixed size
        size_t size = data.size();

        REQUIRE(dma.read(address, data.data(), size) == AccessorError::OUT_OF_BOUNDS);
    }

    SUBCASE("Erase (simulated)")
    {
        uint32_t address = FLASH_START + 10;
        REQUIRE(dma.erase(address) == AccessorError::NO_ERROR); // Simply checks that the function runs without error
    }
}

// Test case for LinuxMockI2CFlashAccessor
TEST_CASE("LinuxMockI2CFlashAccessor")
{
    const uint32_t FLASH_START = 0x08000000;
    const uint32_t FLASH_SIZE = 1024;
    I2C_HandleTypeDef hi2c;
    LinuxMockI2CFlashAccessor hal(&hi2c, FLASH_START, FLASH_SIZE);

    SUBCASE("Write and Read within bounds")
    {
        uint32_t address = FLASH_START + 10;
        uint8_t data[] = {0x05, 0x06, 0x07, 0x08};
        size_t size = sizeof(data);
        std::vector<uint8_t> read_data(size); // Use std::vector
        auto result = hal.write(address, data, size);
        REQUIRE(result == AccessorError::NO_ERROR);
        result = hal.read(address, read_data.data(), size);
        REQUIRE(result == AccessorError::NO_ERROR);
        bool result_bool = compareMemory(data, read_data.data(), size);
        REQUIRE(result_bool == true);
    }

    SUBCASE("Write out of bounds")
    {
        uint32_t address = FLASH_START + FLASH_SIZE;
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        size_t size = sizeof(data);

        REQUIRE(hal.write(address, data, size) == AccessorError::OUT_OF_BOUNDS);
    }

    SUBCASE("Read out of bounds")
    {
        uint32_t address = FLASH_START + FLASH_SIZE;
        std::vector<uint8_t> data(4); // Fixed size
        size_t size = data.size();

        REQUIRE(hal.read(address, data.data(), size) == AccessorError::OUT_OF_BOUNDS);
    }

    SUBCASE("Erase (simulated)")
    {
        uint32_t address = FLASH_START + 10;
        REQUIRE(hal.erase(address) == AccessorError::NO_ERROR); // Simply checks that the function runs without error
    }
}

// Test case for LinuxMockSPIFlashAccessor
TEST_CASE("LinuxMockSPIFlashAccessor")
{
    const uint32_t FLASH_START = 0x08000000;
    const uint32_t FLASH_SIZE = 1024;
    SPI_HandleTypeDef hspi; // Create an instance of the SPI_HandleTypeDef
    LinuxMockSPIFlashAccessor hal(&hspi, FLASH_START, FLASH_SIZE);

    SUBCASE("Write and Read within bounds")
    {
        uint32_t address = FLASH_START + 10;
        uint8_t data[] = {0x05, 0x06, 0x07, 0x08};
        size_t size = sizeof(data);
        std::vector<uint8_t> read_data(size);

        REQUIRE(hal.write(address, data, size) == AccessorError::NO_ERROR);
        copy_spi_tx_to_rx();
        REQUIRE(hal.read(address, read_data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(compareMemory(data, read_data.data(), size));
    }

    SUBCASE("Write out of bounds")
    {
        uint32_t address = FLASH_START + FLASH_SIZE;
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        size_t size = sizeof(data);

        REQUIRE(hal.write(address, data, size) == AccessorError::OUT_OF_BOUNDS);
    }

    SUBCASE("Read out of bounds")
    {
        uint32_t address = FLASH_START + FLASH_SIZE;
        std::vector<uint8_t> data(4);
        size_t size = data.size();

        REQUIRE(hal.read(address, data.data(), size) == AccessorError::OUT_OF_BOUNDS);
    }

    SUBCASE("Erase (simulated)")
    {
        uint32_t address = FLASH_START + 10;
        REQUIRE(hal.erase(address) == AccessorError::NO_ERROR);
    }
}

TEST_CASE("DirectMemoryAccessor and LinuxMockI2CFlashAccessor API consistency")
{
    const uint32_t FLASH_START = 0x08000000;
    const uint32_t FLASH_SIZE = 1024;
    I2C_HandleTypeDef hi2c;

    DirectMemoryAccessor dma(FLASH_START, FLASH_SIZE);
    LinuxMockI2CFlashAccessor hal(&hi2c, FLASH_START, FLASH_SIZE);

    uint32_t address = FLASH_START + 10;
    uint8_t data[] = {0x09, 0x0A, 0x0B, 0x0C};
    size_t size = sizeof(data);
    std::vector<uint8_t> read_data_dma(size); // Use std::vector
    std::vector<uint8_t> read_data_hal(size); // Use std::vector

    // Write using both APIs
    REQUIRE(dma.write(address, data, size) == AccessorError::NO_ERROR);
    REQUIRE(hal.write(address, data, size) == AccessorError::NO_ERROR);

    // Read using both APIs
    REQUIRE(dma.read(address, read_data_dma.data(), size) == AccessorError::NO_ERROR); // Pass the data pointer
    REQUIRE(hal.read(address, read_data_hal.data(), size) == AccessorError::NO_ERROR); // Pass the data pointer

    // Compare the read data
    REQUIRE(compareMemory(read_data_dma.data(), read_data_hal.data(), size)); // Pass the data pointer

    // Erase using both APIs
    REQUIRE(dma.erase(address) == AccessorError::NO_ERROR);
    REQUIRE(hal.erase(address) == AccessorError::NO_ERROR);
}

// Mock Accessor for testing
struct MockAccessor
{
    size_t start;
    size_t size;
    std::vector<uint8_t> data;
    bool force_write_error = false;
    bool force_read_error = false;
    bool is_flushed = false;
    uint32_t last_flushed_address = 0;
    std::vector<uint8_t> last_flushed_data;

    MockAccessor(size_t start, size_t size) : start(start), size(size), data(size, 0) {}

    std::vector<uint8_t> &getFlashMemory() { return data; }
    size_t getFlashMemorySize() const { return size; };
    size_t getFlashStartAddress() const { return start; };
    size_t getAlignment() const { return 1; }

    AccessorError write(uint32_t address, const uint8_t *buffer, size_t num_bytes)
    {
        if (force_write_error)
        {
            return AccessorError::WRITE_ERROR;
        }
        if (address + num_bytes - start > size)
        {
            std::cerr << "MockAccessor::write: out of bounds write. address=" << address
                      << ", num_bytes=" << num_bytes << ", size=" << size << std::endl;
            return AccessorError::OUT_OF_BOUNDS; // Simulate write error
        }
        std::memcpy(data.data() + address - start, buffer, num_bytes);

        last_flushed_address = address;
        last_flushed_data.assign(buffer, buffer + num_bytes);
        is_flushed = true;
        return AccessorError::NO_ERROR; // Simulate successful write
    }

    AccessorError read(uint32_t address, uint8_t *buffer, size_t num_bytes)
    {
        if (force_read_error)
        {
            return AccessorError::READ_ERROR;
        }
        if (address + num_bytes - start > size)
        {
            std::cerr << "MockAccessor::read: out of bounds read. address=" << address
                      << ", num_bytes=" << num_bytes << ", size=" << size << std::endl;
            return AccessorError::OUT_OF_BOUNDS; // Simulate read error
        }
        std::memcpy(buffer, data.data() + address - start, num_bytes);
        return AccessorError::NO_ERROR; // Simulate successful read
    }

    AccessorError erase(uint32_t /*address*/)
    {
        // Mock implementation - just return success
        return AccessorError::NO_ERROR;
    }

    void reset()
    {
        std::fill(data.begin(), data.end(), 0);
        force_write_error = false;
        force_read_error = false;
        is_flushed = false;
        last_flushed_address = 0;
        last_flushed_data.clear();
    }

    void setForceWriteError(bool error)
    {
        force_write_error = error;
    }

    void setForceReadError(bool error)
    {
        force_read_error = error;
    }

    std::pair<uint32_t, std::vector<uint8_t>> getLastFlushedData() const
    {
        return {last_flushed_address, last_flushed_data};
    }
};
static_assert(Accessor<MockAccessor>, "MockAccessor does not satisfy the Accessor concept!");

template <typename T, size_t S>
concept BufferedAccessorConcept = requires(T a) {
    requires Accessor<T>;
};
static_assert(BufferedAccessorConcept<MockAccessor, 512>, "BufferedAccessor does not satisfy the Accessor concept!");

TEST_CASE("Test BufferedAccessor")
{
    const size_t FLASH_START = 0x08000000;
    const size_t FLASH_SIZE = 4096;
    const size_t BLOCK_SIZE = 512; // A smaller block size for this test
    MockAccessor base_accessor(FLASH_START, FLASH_SIZE);
    BufferedAccessor<MockAccessor, BLOCK_SIZE> buffered_accessor(base_accessor);

    SUBCASE("Write and Read within a single block")
    {
        uint32_t address = FLASH_START + 10;
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        size_t size = sizeof(data);
        std::vector<uint8_t> read_data(size);

        REQUIRE(buffered_accessor.write(address, data, size) == AccessorError::NO_ERROR);
        REQUIRE(buffered_accessor.read(address, read_data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data, data + size, read_data.begin()));
    }

    SUBCASE("Write and Read spanning two blocks")
    {
        uint32_t address = FLASH_START + BLOCK_SIZE - 2;
        uint8_t data[] = {0x05, 0x06, 0x07, 0x08};
        size_t size = sizeof(data);
        std::vector<uint8_t> read_data(size);

        REQUIRE(buffered_accessor.write(address, data, size) == AccessorError::NO_ERROR);
        REQUIRE(buffered_accessor.read(address, read_data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data, data + size, read_data.begin()));
    }

    SUBCASE("Write and Read with a larger data size than the block size")
    {
        uint32_t address = FLASH_START + 10;
        std::vector<uint8_t> data(BLOCK_SIZE + 100); // Larger than a block
        for (size_t i = 0; i < data.size(); ++i)
        {
            data[i] = static_cast<uint8_t>(i % 256);
        }
        std::vector<uint8_t> read_data(data.size());
        REQUIRE(buffered_accessor.write(address, data.data(), data.size()) == AccessorError::NO_ERROR);
        REQUIRE(buffered_accessor.read(address, read_data.data(), data.size()) == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data.begin(), data.end(), read_data.begin()));
    }

    SUBCASE("Erase operation")
    {
        uint32_t address = FLASH_START + 10;
        REQUIRE(buffered_accessor.erase(address) == AccessorError::NO_ERROR);
    }

    SUBCASE("Aligned Write and Read")
    {
        uint32_t address = FLASH_START + 10;
        uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};
        size_t size = sizeof(data);

        REQUIRE(buffered_accessor.write(address, data, size) == AccessorError::NO_ERROR);

        std::vector<uint8_t> retrieved_data(size);
        REQUIRE(buffered_accessor.read(address, retrieved_data.data(), size) == AccessorError::NO_ERROR);

        REQUIRE(std::equal(data, data + size, retrieved_data.begin()));
    }

    SUBCASE("Write and Read at the end of Flash Memory")
    {
        uint32_t address = FLASH_START + FLASH_SIZE - 10;
        uint8_t data[] = {0x11, 0x22, 0x33, 0x44};
        size_t size = sizeof(data);

        REQUIRE(buffered_accessor.write(address, data, size) == AccessorError::NO_ERROR);

        std::vector<uint8_t> retrieved_data(size);
        REQUIRE(buffered_accessor.read(address, retrieved_data.data(), size) == AccessorError::NO_ERROR);

        REQUIRE(std::equal(data, data + size, retrieved_data.begin()));
    }

    SUBCASE("Stress test: Many small writes and reads")
    {
        const size_t NUM_OPS = 1000;
        const size_t MAX_WRITE_SIZE = 64;

        for (size_t i = 0; i < NUM_OPS; ++i)
        {
            uint32_t address = FLASH_START + (i * 17) % (FLASH_SIZE - MAX_WRITE_SIZE); // Varying address
            size_t write_size = (i * 31) % MAX_WRITE_SIZE + 1;                         // Varying write size
            std::vector<uint8_t> data(write_size);
            for (size_t j = 0; j < write_size; ++j)
            {
                data[j] = static_cast<uint8_t>((i + j) % 256);
            }

            REQUIRE(buffered_accessor.write(address, data.data(), write_size) == AccessorError::NO_ERROR);

            std::vector<uint8_t> read_data(write_size);
            REQUIRE(buffered_accessor.read(address, read_data.data(), write_size) == AccessorError::NO_ERROR);
            REQUIRE(std::equal(data.begin(), data.end(), read_data.begin()));
        }
    }

    SUBCASE("Write at FLASH_START")
    {
        uint32_t address = FLASH_START;
        uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
        size_t size = sizeof(data);
        std::vector<uint8_t> read_data(size);

        REQUIRE(buffered_accessor.write(address, data, size) == AccessorError::NO_ERROR);
        REQUIRE(buffered_accessor.read(address, read_data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data, data + size, read_data.begin()));
    }

    SUBCASE("Write ending at FLASH_SIZE - 1")
    {
        uint32_t address = FLASH_START + FLASH_SIZE - 4;
        uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
        size_t size = sizeof(data);
        std::vector<uint8_t> read_data(size);

        REQUIRE(buffered_accessor.write(address, data, size) == AccessorError::NO_ERROR);
        REQUIRE(buffered_accessor.read(address, read_data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data, data + size, read_data.begin()));
    }

    SUBCASE("Write zero bytes")
    {
        uint32_t address = FLASH_START + 10;
        std::vector<uint8_t> data; // Empty vector
        size_t size = 0;
        std::vector<uint8_t> read_data;

        REQUIRE(buffered_accessor.write(address, data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(buffered_accessor.read(address, read_data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(read_data.empty()); // Check that read_data is still empty
    }

    SUBCASE("Write to same address twice, with read in between")
    {
        uint32_t address = FLASH_START + 10;
        uint8_t data1[] = {0x11, 0x22, 0x33, 0x44};
        uint8_t data2[] = {0x55, 0x66, 0x77, 0x88};
        size_t size = sizeof(data1); // Both data arrays are the same size
        std::vector<uint8_t> read_data(size);

        // First write
        REQUIRE(buffered_accessor.write(address, data1, size) == AccessorError::NO_ERROR);

        // Read back - should be from cache
        REQUIRE(buffered_accessor.read(address, read_data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data1, data1 + size, read_data.begin()));

        // Second write, overwriting the same location
        REQUIRE(buffered_accessor.write(address, data2, size) == AccessorError::NO_ERROR);

        // Read back - should be the new data
        REQUIRE(buffered_accessor.read(address, read_data.data(), size) == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data2, data2 + size, read_data.begin()));
    }

    SUBCASE("Flush on Destruct")
    {
        const uint32_t address = FLASH_START + 10;
        uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};
        size_t size = sizeof(data);

        // Write to the buffered accessor (making it dirty)
        MockAccessor base_accessor_local(FLASH_START, FLASH_SIZE);
        {
            BufferedAccessor<MockAccessor, BLOCK_SIZE> local_accessor(base_accessor_local);
            auto status = local_accessor.write(address, data, size);
            REQUIRE(status == AccessorError::NO_ERROR);
        } // local_accessor goes out of scope and its destructor is called.

        // Check that data was flushed
        auto [flushed_address, flushed_data] = base_accessor_local.getLastFlushedData();
        REQUIRE(base_accessor_local.is_flushed == true);
        // REQUIRE(flushed_address == FLASH_START);
        REQUIRE(std::equal(data, data + size, flushed_data.begin() + 10));

        // Check that the underlying memory was actually written to.
        std::vector<uint8_t> written_data(size);
        auto read_status = base_accessor_local.read(address, written_data.data(), size);
        REQUIRE(read_status == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data, data + size, written_data.begin()));

        // Reset the base accessor for future tests.
        base_accessor_local.reset();
    }

    SUBCASE("getAlignment() test")
    {
        REQUIRE(buffered_accessor.getAlignment() == BLOCK_SIZE);
    }
}

TEST_CASE("Simplified BufferedAccessor Write Test")
{
    const uint32_t FLASH_START = 0x08000000;
    const uint32_t FLASH_SIZE = 4096;
    const size_t BLOCK_SIZE = 512;

    MockAccessor base_accessor_local(FLASH_START, FLASH_SIZE);

    uint32_t address = FLASH_START + 10;
    uint8_t data[] = {0x11, 0x22, 0x33, 0x44};
    size_t size = sizeof(data);

    SUBCASE("Write to FLASH_START + 10 flush")
    {
        BufferedAccessor<MockAccessor, BLOCK_SIZE> local_accessor(base_accessor_local);
        auto status = local_accessor.write(address, data, size);
        REQUIRE(status == AccessorError::NO_ERROR);
        local_accessor.flush_cache(); // Ensure cache is flushed after write

        // Check base_accessor after destructing
        auto [flushed_address, flushed_data] = base_accessor_local.getLastFlushedData();
        REQUIRE(flushed_address != 0);

        // ASSERT that it written to the mock accessor
        std::vector<uint8_t> written_data(size);
        auto read_status = base_accessor_local.read(address, written_data.data(), size);
        REQUIRE(read_status == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data, data + size, written_data.begin()));
    }

    SUBCASE("Write to FLASH_START + 10 destroy")
    {
        {
            BufferedAccessor<MockAccessor, BLOCK_SIZE> local_accessor(base_accessor_local);
            auto status = local_accessor.write(address, data, size);
            REQUIRE(status == AccessorError::NO_ERROR);
        }
        // Check base_accessor after destructing
        auto [flushed_address, flushed_data] = base_accessor_local.getLastFlushedData();
        REQUIRE(flushed_address != 0);

        // ASSERT that it written to the mock accessor
        std::vector<uint8_t> written_data(size);
        auto read_status = base_accessor_local.read(address, written_data.data(), size);
        REQUIRE(read_status == AccessorError::NO_ERROR);
        REQUIRE(std::equal(data, data + size, written_data.begin()));
    }
}