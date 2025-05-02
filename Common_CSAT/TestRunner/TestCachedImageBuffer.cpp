#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "imagebuffer/accessor.hpp"
#include "imagebuffer/DirectMemoryAccessor.hpp"
#include "imagebuffer/BufferedAccessor.hpp"
#include "ImageBuffer.hpp"

#include "mock_hal.h"
#include "Checksum.hpp"

#include <vector>
#include <iostream>
#include <cstdint>
#include <cstddef>

TEST_CASE("ImageBuffer with BufferedAccessor")
{
    return;
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 4096;
    constexpr size_t block_size = 512;

    DirectMemoryAccessor base_accessor(flash_start, flash_size);
    BufferedAccessor<DirectMemoryAccessor, block_size> buffered_accessor(base_accessor);
    ImageBuffer<BufferedAccessor<DirectMemoryAccessor, block_size>> buffer(buffered_accessor);

    ImageMetadata metadata;
    metadata.timestamp = 12345;
    metadata.image_size = 256;
    metadata.latitude = 37.7749;
    metadata.longitude = -122.4194;
    metadata.camera_index = 1;

    std::vector<uint8_t> image_data(metadata.image_size);
    for (size_t i = 0; i < metadata.image_size; ++i)
    {
        image_data[i] = static_cast<uint8_t>(i % 256);
    }

    SUBCASE("Add and Get Image with BufferedAccessor, chunksize 1")
    {
        constexpr size_t CHUNKSIZE = 1;
        // std::cerr << "ImageBuffer: Calling add_image" << std::endl;
        REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
        // std::cerr << "ImageBuffer: add_image returned" << std::endl;

        for (size_t i = 0; i < image_data.size(); i++)
        {
            // std::cerr << "ImageBuffer: Calling add_data_chunk, i=" << i << std::endl;
            REQUIRE(buffer.add_data_chunk(&image_data[i], CHUNKSIZE) == ImageBufferError::NO_ERROR);
            // std::cerr << "ImageBuffer: add_data_chunk returned" << std::endl;
        }

        std::cerr << "ImageBuffer: Calling push_image" << std::endl;
        REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);
        // std::cerr << "ImageBuffer: push_image returned" << std::endl;

        ImageMetadata retrieved_metadata;

        // std::cerr << "ImageBuffer: Calling get_image" << std::endl;
        REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);
        // std::cerr << "ImageBuffer: get_image returned" << std::endl;

        REQUIRE(retrieved_metadata.magic == IMAGE_MAGIC);
        REQUIRE(retrieved_metadata.timestamp == metadata.timestamp);
        REQUIRE(retrieved_metadata.image_size == metadata.image_size);
        REQUIRE(retrieved_metadata.latitude == metadata.latitude);
        REQUIRE(retrieved_metadata.longitude == metadata.longitude);
        REQUIRE(retrieved_metadata.camera_index == metadata.camera_index);

        std::vector<uint8_t> retrieved_data(metadata.image_size);
        for (size_t i = 0; i < image_data.size(); i++)
        {
            size_t size = CHUNKSIZE;
            // std::cerr << "ImageBuffer: Calling get_data_chunk, i=" << i << std::endl;
            REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
            // std::cerr << "ImageBuffer: get_data_chunk returned, size=" << size << std::endl;
            REQUIRE(size == CHUNKSIZE);
        }

        for (size_t i = 0; i < metadata.image_size; ++i)
        {
            REQUIRE(retrieved_data[i] == image_data[i]);
        }

        buffer.pop_image();
        REQUIRE(buffer.is_empty());
    }

    SUBCASE("Add and Get Image with BufferedAccessor, chunksize 33")
    {
        constexpr size_t CHUNKSIZE = 33;
        // std::cerr << "ImageBuffer: Calling add_image" << std::endl;
        REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
        // std::cerr << "ImageBuffer: add_image returned" << std::endl;

        for (size_t i = 0; i < image_data.size(); i += CHUNKSIZE)
        {
            // std::cerr << "ImageBuffer: Calling add_data_chunk, i=" << i << std::endl;
            REQUIRE(buffer.add_data_chunk(&image_data[i], CHUNKSIZE) == ImageBufferError::NO_ERROR);
            // std::cerr << "ImageBuffer: add_data_chunk returned" << std::endl;
        }

        std::cerr << "ImageBuffer: Calling push_image" << std::endl;
        REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);
        // std::cerr << "ImageBuffer: push_image returned" << std::endl;

        ImageMetadata retrieved_metadata;

        // std::cerr << "ImageBuffer: Calling get_image" << std::endl;
        REQUIRE(buffer.get_image(retrieved_metadata) == ImageBufferError::NO_ERROR);
        // std::cerr << "ImageBuffer: get_image returned" << std::endl;

        REQUIRE(retrieved_metadata.magic == IMAGE_MAGIC);
        REQUIRE(retrieved_metadata.timestamp == metadata.timestamp);
        REQUIRE(retrieved_metadata.image_size == metadata.image_size);
        REQUIRE(retrieved_metadata.latitude == metadata.latitude);
        REQUIRE(retrieved_metadata.longitude == metadata.longitude);
        REQUIRE(retrieved_metadata.camera_index == metadata.camera_index);

        std::vector<uint8_t> retrieved_data(metadata.image_size);
        // size_t niter = image_data.size()/CHUNKSIZE;
        size_t nrem = image_data.size() % CHUNKSIZE;
        for (size_t i = 0; i < image_data.size(); i += CHUNKSIZE)
        {
            size_t size = CHUNKSIZE;
            // std::cerr << "ImageBuffer: Calling get_data_chunk, i=" << i << std::endl;
            REQUIRE(buffer.get_data_chunk(&retrieved_data[i], size) == ImageBufferError::NO_ERROR);
            // std::cerr << "ImageBuffer: get_data_chunk returned, size=" << size << std::endl;
            bool result = size == CHUNKSIZE || size == nrem;
            REQUIRE(result);
        }

        for (size_t i = 0; i < metadata.image_size; ++i)
        {
            REQUIRE(retrieved_data[i] == image_data[i]);
        }

        buffer.pop_image();
        REQUIRE(buffer.is_empty());
    }
}

TEST_CASE("ImageBuffer with BufferedAccessor: Multiple Images")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 16384;
    constexpr size_t block_size = 256;
    constexpr size_t image_size = 640;
    constexpr size_t chunk_size = 64;
    constexpr size_t image_count = 10;

    DirectMemoryAccessor base_accessor(flash_start, flash_size);
    BufferedAccessor<DirectMemoryAccessor, block_size> buffered_accessor(base_accessor);
    ImageBuffer<BufferedAccessor<DirectMemoryAccessor, block_size>> buffer(buffered_accessor);

    for (uint i = 0; i < image_count; i++)
    {
        ImageMetadata metadata;
        metadata.timestamp = 12345 + i;
        metadata.image_size = image_size;
        metadata.latitude = 37.7749 + i * 0.1;
        metadata.longitude = -122.4194 + i * 0.1;
        metadata.camera_index = i;

        std::vector<uint8_t> image_data(metadata.image_size);
        for (size_t j = 0; j < metadata.image_size; ++j)
        {
            image_data[j] = static_cast<uint8_t>(j % 256);
        }

        REQUIRE(buffer.add_image(metadata) == ImageBufferError::NO_ERROR);

        size_t total = 0;
        for (size_t j = 0; j < image_data.size(); j += chunk_size)
        {
            size_t size = chunk_size;
            REQUIRE(buffer.add_data_chunk(&image_data[j], size) == ImageBufferError::NO_ERROR);
            REQUIRE(size == chunk_size);
            total += size;
        }
        {
            size_t size = image_data.size() - total;
            REQUIRE(buffer.add_data_chunk(&image_data[total], size) == ImageBufferError::NO_ERROR);
        }
        REQUIRE(buffer.push_image() == ImageBufferError::NO_ERROR);
        REQUIRE(true);
    }
    for (uint i = 0; i < image_count; i++)
    {
        size_t tail = buffer.get_tail();
        REQUIRE(tail % block_size == 0);

        ImageMetadata metadata;
        REQUIRE(buffer.get_image(metadata) == ImageBufferError::NO_ERROR);

        REQUIRE(metadata.magic == IMAGE_MAGIC);
        REQUIRE(metadata.timestamp == 12345 + i);
        REQUIRE(metadata.image_size == image_size);
        REQUIRE(metadata.latitude == doctest::Approx(37.7749 + i * 0.1));
        REQUIRE(metadata.longitude == doctest::Approx(-122.4194 + i * 0.1));
        REQUIRE(metadata.camera_index == i);

        std::vector<uint8_t> data(image_size);
        size_t total = 0;
        for (size_t j = 0; j < image_size; j+=chunk_size)
        {
            size_t size = chunk_size;
            REQUIRE(buffer.get_data_chunk(&data[j], size) == ImageBufferError::NO_ERROR);
            REQUIRE(size == chunk_size);
            total += size;
        }
        {
            size_t size = image_size - total;
            REQUIRE(buffer.get_data_chunk(&data[total], size) == ImageBufferError::NO_ERROR);
            REQUIRE(size == image_size - total);
        }
        buffer.pop_image();

        for (size_t j = 0; j < image_size; ++j)
        {
            REQUIRE(data[j] == static_cast<uint8_t>(j % 256));
        }
    }
    REQUIRE(buffer.is_empty());
}
