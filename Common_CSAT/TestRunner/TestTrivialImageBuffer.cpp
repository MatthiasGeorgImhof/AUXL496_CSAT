#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "TrivialImageBuffer.hpp"
#include "InputOutputStream.hpp"   // for ImageBufferConcept
#include <vector>
#include <cstring>

using Buffer = TrivialImageBuffer<8192>;

// ------------------------------------------------------------
// Compile-time concept check
// ------------------------------------------------------------
static_assert(ImageBufferConcept<Buffer>, "TrivialImageBuffer must satisfy ImageBufferConcept");

// ------------------------------------------------------------
// Helper: create a dummy metadata object
// ------------------------------------------------------------
static ImageMetadata make_meta(uint32_t payload_size)
{
    ImageMetadata m{};
    m.timestamp     = 12345678;
    m.payload_size  = payload_size;
    m.latitude      = 1.23f;
    m.longitude     = 4.56f;
    m.producer      = METADATA_PRODUCER::CAMERA_1;
    m.format        = METADATA_FORMAT::UNKN;
    return m;
}

// ------------------------------------------------------------
// Test suite
// ------------------------------------------------------------
TEST_CASE("TrivialImageBuffer basic lifecycle")
{
    Buffer buf;

    SUBCASE("Initially empty")
    {
        CHECK(buf.is_empty());
        CHECK(buf.count() == 0);
        CHECK(buf.size() == 0);
    }

    SUBCASE("Add image + payload + push")
    {
        const size_t payload_size = 32;
        auto meta = make_meta(payload_size);

        // Add metadata
        CHECK(buf.add_image(meta) == ImageBufferError::NO_ERROR);
        CHECK(buf.is_empty());   // not pushed yet

        // Add payload
        std::vector<uint8_t> payload(payload_size);
        for (size_t i = 0; i < payload_size; i++)
            payload[i] = static_cast<uint8_t>(i);

        CHECK(buf.add_data_chunk(payload.data(), payload_size)
              == ImageBufferError::NO_ERROR);

        // Push image
        CHECK(buf.push_image() == ImageBufferError::NO_ERROR);
        CHECK(!buf.is_empty());
        CHECK(buf.count() == 1);
        CHECK(buf.size() == payload_size);

        // Retrieve metadata
        ImageMetadata out_meta{};
        CHECK(buf.get_image(out_meta) == ImageBufferError::NO_ERROR);
        CHECK(out_meta.timestamp == meta.timestamp);
        CHECK(out_meta.payload_size == meta.payload_size);
        CHECK(out_meta.latitude == doctest::Approx(meta.latitude));
        CHECK(out_meta.longitude == doctest::Approx(meta.longitude));
        CHECK(out_meta.producer == meta.producer);

        // Retrieve payload in chunks
        size_t offset = 0;
        while (offset < payload_size)
        {
            uint8_t chunk[8];
            size_t chunk_size = sizeof(chunk);

            CHECK(buf.get_data_chunk(chunk, chunk_size)
                  == ImageBufferError::NO_ERROR);

            for (size_t i = 0; i < chunk_size; i++)
                CHECK(chunk[i] == payload[offset + i]);

            offset += chunk_size;
        }

        // Pop image
        CHECK(buf.pop_image() == ImageBufferError::NO_ERROR);
        CHECK(buf.is_empty());
        CHECK(buf.count() == 0);
    }
}

TEST_CASE("TrivialImageBuffer rejects second image while full")
{
    Buffer buf;

    auto meta = make_meta(10);
    CHECK(buf.add_image(meta) == ImageBufferError::NO_ERROR);
    CHECK(buf.add_data_chunk(reinterpret_cast<const uint8_t*>("abc"), 3)
          == ImageBufferError::NO_ERROR);
    CHECK(buf.push_image() == ImageBufferError::NO_ERROR);

    // Now buffer is full
    CHECK(buf.add_image(meta) == ImageBufferError::FULL_BUFFER);
    CHECK(buf.add_data_chunk(reinterpret_cast<const uint8_t*>("xyz"), 3)
          == ImageBufferError::FULL_BUFFER);

    // Pop and try again
    CHECK(buf.pop_image() == ImageBufferError::NO_ERROR);
    CHECK(buf.add_image(meta) == ImageBufferError::NO_ERROR);
}

TEST_CASE("ImageInputStream<TrivialImageBuffer> basic streaming")
{
    Buffer buf;
    ImageInputStream<Buffer> stream(buf);

    const size_t payload_size = 40;
    auto meta = make_meta(payload_size);

    // Prepare payload
    std::vector<uint8_t> payload(payload_size);
    for (size_t i = 0; i < payload_size; i++)
        payload[i] = static_cast<uint8_t>(i + 1);

    SUBCASE("Initially empty")
    {
        CHECK(stream.is_empty());
    }

    SUBCASE("Full streaming lifecycle")
    {
        // Producer side
        CHECK(buf.add_image(meta) == ImageBufferError::NO_ERROR);
        CHECK(buf.add_data_chunk(payload.data(), payload_size)
              == ImageBufferError::NO_ERROR);
        CHECK(buf.push_image() == ImageBufferError::NO_ERROR);

        CHECK(!stream.is_empty());

        // Consumer side: initialize() should return metadata first
        uint8_t meta_buf[sizeof(ImageMetadata)];
        size_t  chunk_size = sizeof(meta_buf);

        CHECK(stream.initialize(meta_buf, chunk_size));
        CHECK(chunk_size == sizeof(ImageMetadata));

        ImageMetadata out_meta{};
        std::memcpy(&out_meta, meta_buf, sizeof(ImageMetadata));

        CHECK(out_meta.timestamp == meta.timestamp);
        CHECK(out_meta.payload_size == meta.payload_size);
        CHECK(out_meta.latitude == doctest::Approx(meta.latitude));
        CHECK(out_meta.longitude == doctest::Approx(meta.longitude));
        CHECK(out_meta.producer == meta.producer);

        // Now read payload in chunks
        size_t offset = 0;
        while (offset < payload_size)
        {
            uint8_t buf8[8];
            size_t  req = sizeof(buf8);

            CHECK(stream.getChunk(buf8, req));

            for (size_t i = 0; i < req; i++)
                CHECK(buf8[i] == payload[offset + i]);

            offset += req;
        }

        // Final chunk: size == 0 indicates end of payload and one should request finalization
        size_t zero = 0;
        CHECK(stream.getChunk(nullptr, zero));

        // After finalize, buffer should be empty
        CHECK(! stream.is_empty());
        CHECK(! buf.is_empty());
        stream.finalize();
        CHECK(stream.is_empty());
        CHECK(buf.is_empty());
    }
}
// ==========================================================================