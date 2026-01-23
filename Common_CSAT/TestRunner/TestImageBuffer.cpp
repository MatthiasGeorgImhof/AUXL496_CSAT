#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "ImageBuffer.hpp"
#include "NullImageBuffer.hpp"

#include "imagebuffer/accessor.hpp"
#include "imagebuffer/DirectMemoryAccessor.hpp"
#include "mock_hal.h"
#include "Checksum.hpp"

#include <vector>
#include <iostream>

template<typename Accessor>
using SimpleImageBuffer = ImageBuffer<Accessor>;

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
    size_t getEraseBlockSize() const { return 1; } 

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

    AccessorError erase(size_t /*address*/) {
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
    SimpleImageBuffer<MockAccessor> buffer(accessor);

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
    SimpleImageBuffer<MockAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 12345;
    metadata.payload_size = 256;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.producer = METADATA_PRODUCER::CAMERA_1;

    std::vector<uint8_t> image_data(metadata.payload_size);
    for (size_t i = 0; i < metadata.payload_size; ++i)
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

        REQUIRE(retrieved_metadata.timestamp == metadata.timestamp);
        REQUIRE(retrieved_metadata.payload_size == metadata.payload_size);
        REQUIRE(retrieved_metadata.latitude == metadata.latitude);
        REQUIRE(retrieved_metadata.longitude == metadata.longitude);
        REQUIRE(retrieved_metadata.producer == metadata.producer);

        std::vector<uint8_t> retrieved_data(metadata.payload_size);
        for (size_t i = 0; i < image_data.size(); i++)
        {
            size_t size = 1;
            REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
            REQUIRE(size == 1);
        }

        for (size_t i = 0; i < metadata.payload_size; ++i)
        {
            REQUIRE(retrieved_data[i] == image_data[i]);
        }
    }

    SUBCASE("Add image exceeding capacity")
    {
        MockAccessor accessor(0, 100); // Small buffer
        SimpleImageBuffer<MockAccessor> buffer(accessor);

        ImageMetadata large_metadata;
        large_metadata.timestamp = 12345;
        large_metadata.payload_size = 256;
        large_metadata.latitude = 37.7749f;
        large_metadata.longitude = -122.4194f;
        large_metadata.producer = METADATA_PRODUCER::CAMERA_1;

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

        std::vector<uint8_t> retrieved_data(metadata.payload_size);
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

TEST_CASE("ImageBuffer Wrap-Around fails due to size")
{
    MockAccessor accessor(0, 512);
    SimpleImageBuffer<MockAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 12345;
    metadata.payload_size = 256;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.producer = METADATA_PRODUCER::CAMERA_1;

    std::vector<uint8_t> image_data(metadata.payload_size);
    for (size_t i = 0; i < metadata.payload_size; ++i)
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
    metadata2.payload_size = 128;
    metadata2.latitude = 34.0522f;
    metadata2.longitude = -118.2437f;
    metadata2.producer = METADATA_PRODUCER::CAMERA_2;

    std::vector<uint8_t> image_data2(metadata2.payload_size);
    for (size_t i = 0; i < metadata2.payload_size; ++i)
    {
        image_data2[i] = static_cast<uint8_t>((i + 100) % 256);
    }

    auto result = buffer.add_image(metadata2);
    REQUIRE(result == ImageBufferError::FULL_BUFFER);
}

TEST_CASE("ImageBuffer Wrap-Around suceeds due pop despite size")
{
    MockAccessor accessor(0, 512);
    SimpleImageBuffer<MockAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 12345;
    metadata.payload_size = 256;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.producer = METADATA_PRODUCER::CAMERA_1;

    std::vector<uint8_t> image_data(metadata.payload_size);
    for (size_t i = 0; i < metadata.payload_size; ++i)
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

    // pop the image to free space
    ImageMetadata retrieved_metadata;
    REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);
    std::vector<uint8_t> retrieved_data(metadata.payload_size);
    for (size_t i = 0; i < image_data.size(); i++)
    {
        size_t size = 1;
        REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
        REQUIRE(size == 1);
    }

    for (size_t i = 0; i < metadata.payload_size; ++i)
    {
        REQUIRE(retrieved_data[i] == image_data[i]);
    }

    REQUIRE(buffer.pop_image() == ImageBufferError::NO_ERROR);

    // Add another image that will cause wrap-around
    ImageMetadata metadata2;
    metadata2.timestamp = 67890;
    metadata2.payload_size = 128;
    metadata2.latitude = 34.0522f;
    metadata2.longitude = -118.2437f;
    metadata2.producer = METADATA_PRODUCER::CAMERA_2;

    std::vector<uint8_t> image_data2(metadata2.payload_size);
    for (size_t i = 0; i < metadata2.payload_size; ++i)
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

    REQUIRE(buffer.count() == 1);

    // Retrieve the images to check for correct data after wrap-around
    ImageMetadata retrieved_metadata2;
    REQUIRE(buffer.get_image(retrieved_metadata2) == ImageBufferError::NO_ERROR);

    std::vector<uint8_t> retrieved_data2(metadata2.payload_size);
    for (size_t i = 0; i < image_data2.size(); i++)
    {
        size_t size = 1;
        REQUIRE(buffer.get_data_chunk(&retrieved_data2[i], size) == ImageBufferError::NO_ERROR);
        REQUIRE(size == 1);
    }

    for (size_t i = 0; i < metadata2.payload_size; ++i)
    {
        REQUIRE(retrieved_data2[i] == image_data2[i]);
    }
}

TEST_CASE("ImageBuffer chunk read")
{
    MockAccessor accessor(0, 512);
    SimpleImageBuffer<MockAccessor> buffer(accessor);

    ImageMetadata metadata;
    metadata.timestamp = 12345;
    metadata.payload_size = 255;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.producer = METADATA_PRODUCER::CAMERA_1;

    std::vector<uint8_t> image_data(metadata.payload_size);
    for (size_t i = 0; i < metadata.payload_size; ++i)
    {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    // Fill most of the buffer
    REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    REQUIRE(buffer.add_data_chunk(image_data.data(), metadata.payload_size) == ImageBufferError::NO_ERROR);
    REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);
    REQUIRE(buffer.count() == 1);

    ImageMetadata retrieved_metadata;
    REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);
    std::vector<uint8_t> retrieved_data(metadata.payload_size);
    
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
    metadata.payload_size = 1024;
    metadata.latitude = 33.0f;
    metadata.longitude = -97.0f;
    metadata.producer = METADATA_PRODUCER::CAMERA_3;

    std::vector<uint8_t> image_data(metadata.payload_size);
    for (size_t i = 0; i < metadata.payload_size; ++i)
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
    REQUIRE(retrieved_metadata.payload_size == metadata.payload_size);
    REQUIRE(retrieved_metadata.latitude == metadata.latitude);
    REQUIRE(retrieved_metadata.longitude == metadata.longitude);
    REQUIRE(retrieved_metadata.producer == metadata.producer);

    std::vector<uint8_t> retrieved_data(metadata.payload_size);
    for (size_t i = 0; i < image_data.size(); i++)
    {
        size_t size = 1;
        REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
        REQUIRE(size == 1);
    }

    for (size_t i = 0; i < metadata.payload_size; ++i)
    {
        REQUIRE(retrieved_data[i] == image_data[i]);
    }
}

TEST_CASE("NullImageBuffer basic behavior")
{
    NullImageBuffer buf;

    // Construct minimal metadata
    ImageMetadata meta{};
    meta.version       = 1;
    meta.metadata_size = sizeof(ImageMetadata);
    meta.timestamp     = 123456;
    meta.latitude      = 1.23f;
    meta.longitude     = 4.56f;
    meta.payload_size  = 16;
    meta.dimensions    = {4, 2, 2};
    meta.format        = METADATA_FORMAT::UNKN;
    meta.producer      = METADATA_PRODUCER::THERMAL;

    // add_image should succeed and log
    CHECK(buf.add_image(meta) == ImageBufferError::NO_ERROR);

    // add_data_chunk should accept and discard
    uint8_t dummy[16] = {0};
    size_t sz = sizeof(dummy);
    CHECK(buf.add_data_chunk(dummy, sz) == ImageBufferError::NO_ERROR);

    // push_image should succeed
    CHECK(buf.push_image() == ImageBufferError::NO_ERROR);

    // Buffer must always appear empty
    CHECK(buf.is_empty() == true);
    CHECK(buf.count() == 0);
    CHECK(buf.size() == 0);

    // Reads must return EMPTY_BUFFER
    ImageMetadata out{};
    CHECK(buf.get_image(out) == ImageBufferError::EMPTY_BUFFER);

    uint8_t outbuf[8];
    size_t outsz = sizeof(outbuf);
    CHECK(buf.get_data_chunk(outbuf, outsz) == ImageBufferError::EMPTY_BUFFER);
    CHECK(buf.pop_image() == ImageBufferError::EMPTY_BUFFER);
}

// -----------------------------------------------------------------------------
// Minimal test-only subclass to expose test_set_tail()
// -----------------------------------------------------------------------------
template <typename Accessor>
class TestImageBuffer : public ImageBuffer<Accessor>
{
public:
    using Base = ImageBuffer<Accessor>;
    using Base::Base;   // inherit constructors

    void set_tail_for_test(size_t t) {
        this->test_set_tail(t);
    }
};


TEST_CASE("Regression: DirectMemoryAccessor 512-byte temp buffer can write one full entry")
{
    // 1. Create a 512-byte temp flash
    std::vector<uint8_t> temp(512, 0xFF);
    DirectMemoryAccessor temp_acc(reinterpret_cast<uintptr_t>(temp.data()), temp.size());

    // 2. Construct a buffer on that accessor
    TestImageBuffer<DirectMemoryAccessor> buf(temp_acc);

    // 3. Place tail at 0 (as write_valid_entry does)
    buf.set_tail_for_test(0);

    // 4. Create metadata with a small payload
    ImageMetadata meta{};
    meta.timestamp     = 1234;
    meta.payload_size  = 32;
    meta.latitude      = 1.0f;
    meta.longitude     = 2.0f;
    meta.producer      = METADATA_PRODUCER::CAMERA_1;

    // 5. add_image must succeed
    REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

    // 6. Write payload
    std::vector<uint8_t> payload(meta.payload_size);
    for (size_t i = 0; i < payload.size(); i++)
        payload[i] = static_cast<uint8_t>(i);

    REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);

    // 7. push_image must succeed
    REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);

    // 8. Final sanity: buffer state is consistent
    CHECK(buf.size() > 0);
    CHECK(buf.count() == 1);
}
