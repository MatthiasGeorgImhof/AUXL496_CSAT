#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "ImageBuffer.hpp"
#include "InputOutputStream.hpp"

#include "imagebuffer/accessor.hpp"
#include "imagebuffer/DirectMemoryAccessor.hpp"
#include "mock_hal.h"
#include "Checksum.hpp"

#include <vector>
#include <iostream>
#include <fstream> // for file I/O in tests

template<typename Accessor>
using SimpleImageBuffer = ImageBuffer<Accessor>;

template <typename Accessor>
using CachedImageBuffer = ImageBuffer<Accessor>;


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

TEST_CASE("ImageInputStream with ImageBuffer") {
    MockAccessor accessor(0, 2048); // Larger buffer for this test
    ImageBuffer<MockAccessor> image_buffer(accessor);
    ImageInputStream<ImageBuffer<MockAccessor>> stream(image_buffer);

    ImageMetadata metadata;
    metadata.timestamp = 0x12345678;
    metadata.payload_size = 256;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.producer = METADATA_PRODUCER::THERMAL;

    std::vector<uint8_t> image_data(metadata.payload_size);
    for (size_t i = 0; i < metadata.payload_size; ++i) {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    REQUIRE(image_buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    REQUIRE(image_buffer.add_data_chunk(image_data.data(), metadata.payload_size) == ImageBufferError::NO_ERROR);
    REQUIRE(image_buffer.push_image() == ImageBufferError::NO_ERROR);

    SUBCASE("Test initialize") {
        size_t size = 2*sizeof(ImageMetadata);
        uint8_t meta[2*sizeof(ImageMetadata)];
        REQUIRE(stream.initialize(meta, size) == true);
        REQUIRE(size == sizeof(ImageMetadata));
        ImageMetadata *metadata_ = reinterpret_cast<ImageMetadata*>(meta);
        CHECK(metadata_->timestamp == metadata.timestamp);
        CHECK(metadata_->producer == metadata.producer);
    }

    SUBCASE("Test size") {
        REQUIRE(stream.size() == metadata.payload_size + sizeof(ImageMetadata));
    }

    SUBCASE("Test name") {
        std::array<char, NAME_LENGTH> expected_name = formatValues(metadata.timestamp, static_cast<uint8_t>(metadata.producer));
        
        CHECK(memcmp(stream.name().data(), expected_name.data(), NAME_LENGTH) == 0);
    }

   SUBCASE("Test getChunk") {
        size_t count = 0;
        size_t size = 2*sizeof(ImageMetadata);
        uint8_t chunk[2*sizeof(ImageMetadata)];
        REQUIRE(stream.initialize(chunk, size) == true);
        size = 10; // Request a chunk of 10 bytes

        REQUIRE(stream.getChunk(chunk, size) == true);
        REQUIRE(size <= 10); // The actual size should be no more than 10
        REQUIRE(size != 0);

        // Verify the chunk data (first 'size' bytes)
        for (size_t i = 0; i < size; ++i) {
            REQUIRE(chunk[i] == image_data[i]);
        }
        count += size;

        // Subsequent calls to getChunk
        REQUIRE(stream.getChunk(chunk, size) == true);
        REQUIRE(size <= 10); // The actual size should be no more than 10
        REQUIRE(size != 0);
        for (size_t i = 0; i < size; ++i) {
            REQUIRE(chunk[i] == image_data[i+count]);
        }
        count += size;
    }

    SUBCASE("Test is_empty after adding and then popping image") {
        constexpr size_t chunk_size = 10;
        REQUIRE(stream.is_empty() == false);
        size_t size = 2*sizeof(ImageMetadata);
        uint8_t chunk[2*sizeof(ImageMetadata)];
        REQUIRE(stream.initialize(chunk, size) == true);
        size_t stream_size = stream.size() - sizeof(ImageMetadata);
        while(stream_size > 0){
            size = std::min(chunk_size, stream_size);
            REQUIRE(stream.getChunk(chunk, size) == true);
            stream_size -= size;
        }
        CHECK(size==6);
        REQUIRE(stream.is_empty() == false);

        size = std::min(chunk_size, stream_size);
        REQUIRE(stream.getChunk(chunk, size) == true);
        CHECK(size==0);
        REQUIRE(stream.is_empty() == false);
        stream.finalize();
        REQUIRE(stream.is_empty() == true);

    }

}

TEST_CASE("ImageInputStream with CachedImageBuffer") {
    MockAccessor accessor(0, 2048); // Larger buffer for this test
    CachedImageBuffer<MockAccessor> image_buffer(accessor);
    ImageInputStream<CachedImageBuffer<MockAccessor>> stream(image_buffer);

    ImageMetadata metadata;
    metadata.timestamp = 0x12345678;
    metadata.payload_size = 256;
    metadata.latitude = 37.7749f;
    metadata.longitude = -122.4194f;
    metadata.producer = METADATA_PRODUCER::THERMAL;

    std::vector<uint8_t> image_data(metadata.payload_size);
    for (size_t i = 0; i < metadata.payload_size; ++i) {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    REQUIRE(image_buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    REQUIRE(image_buffer.add_data_chunk(image_data.data(), metadata.payload_size) == ImageBufferError::NO_ERROR);
    REQUIRE(image_buffer.push_image() == ImageBufferError::NO_ERROR);

    SUBCASE("Test initialize") {
        size_t size = 2*sizeof(ImageMetadata);
        uint8_t meta[2*sizeof(ImageMetadata)];
        REQUIRE(stream.initialize(meta, size) == true);
        REQUIRE(size == sizeof(ImageMetadata));
        ImageMetadata *metadata_ = reinterpret_cast<ImageMetadata*>(meta);
        CHECK(metadata_->timestamp == metadata.timestamp);
        CHECK(metadata_->producer == metadata.producer);
    }

    SUBCASE("Test size") {
        REQUIRE(stream.size() == metadata.payload_size + sizeof(ImageMetadata));
    }

    SUBCASE("Test name") {
        std::array<char, NAME_LENGTH> expected_name = formatValues(metadata.timestamp, static_cast<uint8_t>(metadata.producer));
        
        CHECK(memcmp(stream.name().data(), expected_name.data(), NAME_LENGTH) == 0);
    }

   SUBCASE("Test getChunk") {
        size_t count = 0;
        size_t size = 2*sizeof(ImageMetadata);
        uint8_t chunk[2*sizeof(ImageMetadata)];
        REQUIRE(stream.initialize(chunk, size) == true);
        size = 10; // Request a chunk of 10 bytes

        REQUIRE(stream.getChunk(chunk, size) == true);
        REQUIRE(size <= 10); // The actual size should be no more than 10
        REQUIRE(size != 0);

        // Verify the chunk data (first 'size' bytes)
        for (size_t i = 0; i < size; ++i) {
            REQUIRE(chunk[i] == image_data[i]);
        }
        count += size;

        // Subsequent calls to getChunk
        REQUIRE(stream.getChunk(chunk, size) == true);
        REQUIRE(size <= 10); // The actual size should be no more than 10
        REQUIRE(size != 0);
        for (size_t i = 0; i < size; ++i) {
            REQUIRE(chunk[i] == image_data[i+count]);
        }
        count += size;
    }

    SUBCASE("Test is_empty after adding and then popping image") {
        constexpr size_t chunk_size = 10;
        REQUIRE(stream.is_empty() == false);
        size_t size = 2*sizeof(ImageMetadata);
        uint8_t chunk[2*sizeof(ImageMetadata)];
        REQUIRE(stream.initialize(chunk, size) == true);
        size_t stream_size = stream.size() - sizeof(ImageMetadata);
        while(stream_size > 0){
            size = std::min(chunk_size, stream_size);
            REQUIRE(stream.getChunk(chunk, size) == true);
            stream_size -= size;
        }
        CHECK(size==6);
        REQUIRE(stream.is_empty() == false);

        size = std::min(chunk_size, stream_size);
        REQUIRE(stream.getChunk(chunk, size) == true);
        CHECK(size==0);
        REQUIRE(stream.is_empty() == false);
        stream.finalize();
        REQUIRE(stream.is_empty() == true);

    }
}

TEST_CASE("TrivialOuputStream satisfies OutputStreamConcept") {
    TrivialOuputStream stream;
    std::array<char, NAME_LENGTH> name = {'t', 'e', 's', 't', '.', 't', 'x', 't', 0, 0, 0};
    std::vector<uint8_t> data(10, 0);
    size_t size = data.size();

    // Call the methods to ensure they exist and don't crash
    REQUIRE(stream.initialize(name) == true);
    REQUIRE(stream.output(data.data(), size) == true);
    REQUIRE(stream.finalize() == true);
}

TEST_CASE("OutputStreamToFile satisfies OutputStreamConcept") {
    OutputStreamToFile stream;
    std::array<char, NAME_LENGTH> name = {'t', 'e', 's', 't', '.', 't', 'x', 't', 0, 0, 0};
    std::vector<uint8_t> data(10);
    for(size_t i=0; i < data.size(); ++i){
        data[i] = static_cast<uint8_t>(i);
    }
    size_t size = data.size();

    // Call the methods to ensure they exist and don't crash
    REQUIRE(stream.initialize(name) == true);
    REQUIRE(stream.output(data.data(), size) == true);
    REQUIRE(stream.finalize() == true);

    // You can add checks to verify specific behavior.
    // For example, you could check that the file was created and that the data was written to it.
    std::ifstream file(name.data(), std::ios::binary | std::ios::in);
    REQUIRE(file.is_open());

    std::vector<uint8_t> file_data(size);
    file.read(reinterpret_cast<char*>(file_data.data()), static_cast<std::streamsize>(size));
    REQUIRE(memcmp(data.data(), file_data.data(), size) == 0);

    file.close();
    remove(name.data());  // Clean up the file
}

TEST_CASE("FileInputStream satisfies InputStreamConcept") {
    // Create a test file with some known content
    std::string filename = "test_file.bin";
    std::ofstream outfile(filename, std::ios::binary);
    std::vector<uint8_t> test_data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    outfile.write(reinterpret_cast<const char*>(test_data.data()), static_cast<std::streamsize>(test_data.size()));
    outfile.close();

    FileInputStream stream(filename);

    SUBCASE("Test is_empty") {
        REQUIRE(stream.is_empty() == false);
    }

    SUBCASE("Test size") {
        REQUIRE(stream.size() == test_data.size());
    }

    SUBCASE("Test name") {
        std::array<char, NAME_LENGTH> expected_name;
        strncpy(expected_name.data(), filename.c_str(), NAME_LENGTH - 1);
        expected_name[NAME_LENGTH - 1] = '\0';
        CHECK(memcmp(stream.name().data(), expected_name.data(), NAME_LENGTH) == 0);
    }

    SUBCASE("Test initialize and getChunk") {
        constexpr size_t buffer_size = 5;
        uint8_t buffer[buffer_size];
        size_t size = buffer_size;

        REQUIRE(stream.initialize(buffer, size) == true);
        REQUIRE(size == buffer_size);
        REQUIRE(memcmp(buffer, test_data.data(), size) == 0);

        size = buffer_size;
        REQUIRE(stream.getChunk(buffer, size) == true);
        REQUIRE(size == 5);
        REQUIRE(memcmp(buffer, test_data.data() + buffer_size, size) == 0);

        size = buffer_size;
        REQUIRE(stream.getChunk(buffer, size) == true);
        REQUIRE(size == 0); // End of file
    }

    SUBCASE("Test finalize") {
        REQUIRE(stream.finalize() == true);
    }
    remove(filename.c_str());  // Clean up the file
}