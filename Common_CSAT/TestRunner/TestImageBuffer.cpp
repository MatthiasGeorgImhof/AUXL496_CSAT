#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "ImageBuffer.hpp"

#include "imagebuffer/accessor.hpp"
#include "imagebuffer/DirectMemoryAccessor.hpp"
#include "imagebuffer/LinuxMockI2CFlashAccessor.hpp"
#include "mock_hal.h"
#include "Checksum.hpp"

#include <vector>
#include <iostream>

// Mock Accessor for testing
struct MockAccessor
{
    size_t start;
    size_t size;
    std::vector<uint8_t> data;

    MockAccessor(size_t start, size_t size) : start(start), size(size), data(size, 0) {}

    std::vector<uint8_t> &getFlashMemory() { return data; }
    size_t getFlashMemorySize() const { return size; };
    size_t getFlashStartAddress() const { return start; };
    size_t getAlignment() const { return 1; }

    AccessorError write(size_t address, const uint8_t *buffer, size_t num_bytes)
    {
        if (address + num_bytes - start > size)
        {
            std::cerr << "MockAccessor::write: out of bounds write. address=" << address << ", num_bytes=" << num_bytes << ", size=" << size << std::endl;
            return AccessorError::WRITE_ERROR; // Simulate write error
        }
        std::memcpy(data.data() + address - start, buffer, num_bytes);
        return AccessorError::NO_ERROR; // Simulate successful write
    }

    AccessorError read(size_t address, uint8_t *buffer, size_t num_bytes)
    {
        if (address + num_bytes - start > size)
        {
            std::cerr << "MockAccessor::read: out of bounds read. address=" << address << ", num_bytes=" << num_bytes << ", size=" << size << std::endl;
            return AccessorError::READ_ERROR; // Simulate read error
        }
        std::memcpy(buffer, data.data() + address - start, num_bytes);
        return AccessorError::NO_ERROR; // Simulate successful read
    }

    AccessorError erase(uint32_t /*address*/) {
        // Mock implementation - just return success
        return AccessorError::NO_ERROR;
    }

    void reset()
    {
        std::fill(data.begin(), data.end(), 0);
    }
};

static_assert(Accessor<MockAccessor>, "MockAccessor does not satisfy the Accessor concept!");

TEST_CASE("ImageBuffer Initialization")
{
    MockAccessor accessor(0, 1024);
    ImageBuffer<MockAccessor> buffer(accessor);

    REQUIRE(buffer.is_empty() == true);
    REQUIRE(buffer.size() == 0);
    REQUIRE(buffer.count() == 0);
    REQUIRE(buffer.available() == 1024);
    REQUIRE(buffer.capacity() == 1024);
    REQUIRE(buffer.get_head() == 0);
    REQUIRE(buffer.get_tail() == 0);
}

TEST_CASE("ImageBuffer Add and Get Image")
{
    MockAccessor accessor(1000, 1024);
    ImageBuffer<MockAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 12345;
    metadata.image_size = 256;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.camera_index = 1;

    std::vector<uint8_t> image_data(metadata.image_size);
    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    SUBCASE("Add Image Success")
    {
        REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
        for (size_t i = 0; i < image_data.size(); i++)
        {
            REQUIRE(buffer.add_data_chunk(&image_data[i], 1) == ImageBufferError::NO_ERROR);
        }
        REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);

        REQUIRE(buffer.is_empty() == false);
        REQUIRE(buffer.size() > 0);
        REQUIRE(buffer.count() == 1);
        REQUIRE(buffer.available() < 1024);
    }

    SUBCASE("Get Image Success")
    {
        REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
        for (size_t i = 0; i < image_data.size(); i++)
        {
            REQUIRE(buffer.add_data_chunk(&image_data[i], 1) == ImageBufferError::NO_ERROR);
        }
        REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);

        ImageMetadata retrieved_metadata;
        REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);

        REQUIRE(retrieved_metadata.magic == IMAGE_MAGIC);
        REQUIRE(retrieved_metadata.timestamp == metadata.timestamp);
        REQUIRE(retrieved_metadata.image_size == metadata.image_size);
        REQUIRE(retrieved_metadata.latitude == metadata.latitude);
        REQUIRE(retrieved_metadata.longitude == metadata.longitude);
        REQUIRE(retrieved_metadata.camera_index == metadata.camera_index);

        std::vector<uint8_t> retrieved_data(metadata.image_size);
        for (size_t i = 0; i < image_data.size(); i++)
        {
            size_t size = 1;
            REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
            REQUIRE(size == 1);
        }

        for (size_t i = 0; i < metadata.image_size; ++i)
        {
            REQUIRE(retrieved_data[i] == image_data[i]);
        }
    }

    SUBCASE("Add image exceeding capacity")
    {
        MockAccessor accessor(0, 100); // Small buffer
        ImageBuffer<MockAccessor> buffer(accessor);

        ImageMetadata large_metadata;
        large_metadata.timestamp = 12345;
        large_metadata.image_size = 256;
        large_metadata.latitude = 37.7749f;
        large_metadata.longitude = -122.4194f;
        large_metadata.camera_index = 1;

        REQUIRE(buffer.add_image(large_metadata) == ImageBufferError::FULL_BUFFER);
    }

    SUBCASE("Pop Image Success")
    {
        REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
        for (size_t i = 0; i < image_data.size(); i++)
        {
            REQUIRE(buffer.add_data_chunk(&image_data[i], 1) == ImageBufferError::NO_ERROR);
        }
        REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);

        ImageMetadata retrieved_metadata;
        REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);

        std::vector<uint8_t> retrieved_data(metadata.image_size);
        for (size_t i = 0; i < image_data.size(); i++)
        {
            size_t size = 1;
            REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
            REQUIRE(size == 1);
        }

        REQUIRE(buffer.pop_image() == ImageBufferError::NO_ERROR);
        REQUIRE(buffer.is_empty() == true);
        REQUIRE(buffer.size() == 0);
        REQUIRE(buffer.count() == 0);
    }

    SUBCASE("Get Image From Empty Buffer")
    {
        ImageMetadata retrieved_metadata;
        REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::EMPTY_BUFFER);
    }
}

TEST_CASE("ImageBuffer Wrap-Around")
{
    MockAccessor accessor(0, 512);
    ImageBuffer<MockAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 12345;
    metadata.image_size = 256;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.camera_index = 1;

    std::vector<uint8_t> image_data(metadata.image_size);
    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    // Fill most of the buffer
    REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    for (size_t i = 0; i < image_data.size(); i++)
    {
        REQUIRE(buffer.add_data_chunk(&image_data[i], 1) == ImageBufferError::NO_ERROR);
    }
    REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);

    // Add another image that will cause wrap-around
    ImageMetadata metadata2;
    metadata2.timestamp = 67890;
    metadata2.image_size = 128;
    metadata2.latitude = 34.0522f;
    metadata2.longitude = -118.2437f;
    metadata2.camera_index = 2;

    std::vector<uint8_t> image_data2(metadata2.image_size);
    for (size_t i = 0; i < metadata2.image_size; ++i)
    {
        image_data2[i] = static_cast<uint8_t>((i + 100) % 256);
    }

    auto result = buffer.add_image(metadata2);
    REQUIRE(result == ImageBufferError::NO_ERROR);
    for (size_t i = 0; i < image_data2.size(); i++)
    {
        REQUIRE(buffer.add_data_chunk(&image_data2[i], 1) == ImageBufferError::NO_ERROR);
    }
    REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);

    REQUIRE(buffer.count() == 2);

    // Retrieve the images to check for correct data after wrap-around
    ImageMetadata retrieved_metadata;
    REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);
    std::vector<uint8_t> retrieved_data(metadata.image_size);
    for (size_t i = 0; i < image_data.size(); i++)
    {
        size_t size = 1;
        REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
        REQUIRE(size == 1);
    }

    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        REQUIRE(retrieved_data[i] == image_data[i]);
    }

    REQUIRE(buffer.pop_image() == ImageBufferError::NO_ERROR);

    ImageMetadata retrieved_metadata2;
    REQUIRE(buffer.get_image(retrieved_metadata2) == ImageBufferError::NO_ERROR);

    std::vector<uint8_t> retrieved_data2(metadata2.image_size);
    for (size_t i = 0; i < image_data2.size(); i++)
    {
        size_t size = 1;
        REQUIRE(buffer.get_data_chunk(&retrieved_data2[i], size) == ImageBufferError::NO_ERROR);
        REQUIRE(size == 1);
    }

    for (size_t i = 0; i < metadata2.image_size; ++i)
    {
        REQUIRE(retrieved_data2[i] == image_data2[i]);
    }
}

TEST_CASE("ImageBuffer chunk read")
{
    MockAccessor accessor(0, 512);
    ImageBuffer<MockAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 12345;
    metadata.image_size = 255;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.camera_index = 1;

    std::vector<uint8_t> image_data(metadata.image_size);
    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    // Fill most of the buffer
    REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    REQUIRE(buffer.add_data_chunk(image_data.data(), metadata.image_size) == ImageBufferError::NO_ERROR);
    REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);
    REQUIRE(buffer.count() == 1);

    ImageMetadata retrieved_metadata;
    REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);
    std::vector<uint8_t> retrieved_data(metadata.image_size);
    
    size_t size = 100;
    REQUIRE(buffer.get_data_chunk(retrieved_data.data(), size) == ImageBufferError::NO_ERROR);
    REQUIRE(size == 100);

    size = 100;
    REQUIRE(buffer.get_data_chunk(retrieved_data.data(), size) == ImageBufferError::NO_ERROR);
    REQUIRE(size == 100);

    size = 100;
    REQUIRE(buffer.get_data_chunk(retrieved_data.data(), size) == ImageBufferError::NO_ERROR);
    REQUIRE(size == 55);

    REQUIRE(buffer.pop_image() == ImageBufferError::NO_ERROR);
    REQUIRE(buffer.count() == 0);
    REQUIRE(buffer.is_empty() == true);
    REQUIRE(buffer.size() == 0);
}


TEST_CASE("ImageBuffer with DirectMemoryAccessor")
{
    DirectMemoryAccessor accessor(0x8000000, 4096); // Flash starts at 0x8000000, size 4096
    ImageBuffer<DirectMemoryAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 98765;
    metadata.image_size = 1024;
    metadata.latitude = 33.0f;
    metadata.longitude = -97.0f;
    metadata.camera_index = 3;

    std::vector<uint8_t> image_data(metadata.image_size);
    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    for (size_t i = 0; i < image_data.size(); i++)
    {
        REQUIRE(buffer.add_data_chunk(&image_data[i], 1) == ImageBufferError::NO_ERROR);
    }
    REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);

    ImageMetadata retrieved_metadata;
    REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);

    REQUIRE(retrieved_metadata.timestamp == metadata.timestamp);
    REQUIRE(retrieved_metadata.image_size == metadata.image_size);
    REQUIRE(retrieved_metadata.latitude == metadata.latitude);
    REQUIRE(retrieved_metadata.longitude == metadata.longitude);
    REQUIRE(retrieved_metadata.camera_index == metadata.camera_index);

    std::vector<uint8_t> retrieved_data(metadata.image_size);
    for (size_t i = 0; i < image_data.size(); i++)
    {
        size_t size = 1;
        REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
        REQUIRE(size == 1);
    }

    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        REQUIRE(retrieved_data[i] == image_data[i]);
    }
}

TEST_CASE("ImageBuffer with LinuxMockI2CFlashAccessor")
{
    // Initialize the mocked HAL
    I2C_HandleTypeDef hi2c;
    LinuxMockI2CFlashAccessor accessor(&hi2c, 0x08000000, 4096);
    ImageBuffer<LinuxMockI2CFlashAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 98765;
    metadata.image_size = 1024;
    metadata.latitude = 33.0f;
    metadata.longitude = -97.0f;
    metadata.camera_index = 3;

    std::vector<uint8_t> image_data(metadata.image_size);
    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        REQUIRE(buffer.add_data_chunk(&image_data[i], 1) == ImageBufferError::NO_ERROR);
    }
    REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);

    ImageMetadata retrieved_metadata;
    REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);

    REQUIRE(retrieved_metadata.timestamp == metadata.timestamp);
    REQUIRE(retrieved_metadata.image_size == metadata.image_size);
    REQUIRE(retrieved_metadata.latitude == metadata.latitude);
    REQUIRE(retrieved_metadata.longitude == metadata.longitude);
    REQUIRE(retrieved_metadata.camera_index == metadata.camera_index);

    std::vector<uint8_t> retrieved_data(metadata.image_size);
    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        size_t size = 1;
        REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
        REQUIRE(size == 1);
    }

    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        REQUIRE(retrieved_data[i] == image_data[i]);
    }

    // Additional checks:  Verify the HAL calls

    // little endian
    std::vector<uint8_t> &flash_memory = accessor.getFlashMemory();
    CHECK(flash_memory[3] == static_cast<uint8_t>((IMAGE_MAGIC >> 24) & 0xFF));
    CHECK(flash_memory[2] == static_cast<uint8_t>((IMAGE_MAGIC >> 16) & 0xFF));
    CHECK(flash_memory[1] == static_cast<uint8_t>((IMAGE_MAGIC >> 8) & 0xFF));
    CHECK(flash_memory[0] == static_cast<uint8_t>(IMAGE_MAGIC & 0xFF));
}