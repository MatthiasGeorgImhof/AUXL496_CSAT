#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "ImageBuffer.hpp"
#include "imagebuffer/DirectMemoryAccessor.hpp"
#include "imagebuffer/storageheader.hpp"
#include "imagebuffer/image.hpp"
#include "Checksum.hpp"

#include <vector>
#include <cstring>

// Convenience alias
template <typename Accessor>
using TestBuffer = ImageBuffer<Accessor>;

static ImageMetadata make_meta(size_t image_size, uint32_t ts = 1234)
{
    ImageMetadata m{};
    m.timestamp = ts;
    m.payload_size = static_cast<uint32_t>(image_size);
    m.latitude = 1.0f;
    m.longitude = 2.0f;
    m.producer = METADATA_PRODUCER::CAMERA_1;
    return m;
}

static void erase_flash(DirectMemoryAccessor &acc, size_t flash_start, size_t flash_size)
{
    std::vector<uint8_t> blank(flash_size, 0xFF);
    acc.write(flash_start, blank.data(), flash_size);
}

TEST_CASE("initialize_from_flash: empty flash")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);
    TestBuffer<DirectMemoryAccessor> buf(acc);

    // Flash is erased by default
    ImageBufferError err = buf.initialize_from_flash();
    REQUIRE(err == ImageBufferError::NO_ERROR);

    REQUIRE(buf.is_empty());
    REQUIRE(buf.count() == 0);
    REQUIRE(buf.size() == 0);
    REQUIRE(buf.get_head() == 0);
    REQUIRE(buf.get_tail() == 0);
}

TEST_CASE("initialize_from_flash: single entry")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // First buffer writes data
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        ImageMetadata meta = make_meta(64, 1000);

        REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

        std::vector<uint8_t> payload(meta.payload_size);
        for (size_t i = 0; i < payload.size(); i++)
            payload[i] = uint8_t(i);

        REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
        REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
    }

    // New buffer reconstructs
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);

        ImageBufferError err = buf.initialize_from_flash();
        REQUIRE(err == ImageBufferError::NO_ERROR);

        REQUIRE(buf.count() == 1);

        ImageMetadata meta{};
        REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
        REQUIRE(meta.timestamp == 1000);
        REQUIRE(meta.payload_size == 64);

        std::vector<uint8_t> out(meta.payload_size);
        size_t chunk = meta.payload_size;
        REQUIRE(buf.get_data_chunk(out.data(), chunk) == ImageBufferError::NO_ERROR);
        REQUIRE(chunk == meta.payload_size);

        for (size_t i = 0; i < out.size(); i++)
            REQUIRE(out[i] == uint8_t(i));

        REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);
        REQUIRE(buf.is_empty());
    }
}

TEST_CASE("initialize_from_flash: multiple entries")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 16384;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    const size_t N = 5;
    const size_t IMG_SIZE = 128;

    // Write N images
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);

        for (size_t i = 0; i < N; i++)
        {
            ImageMetadata meta = make_meta(IMG_SIZE, static_cast<uint32_t>(2000 + i));
            REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

            std::vector<uint8_t> payload(IMG_SIZE);
            for (size_t j = 0; j < IMG_SIZE; j++)
                payload[j] = uint8_t(j + i);

            REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }
    }

    // Reconstruct
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        ImageBufferError err = buf.initialize_from_flash();
        REQUIRE(err == ImageBufferError::NO_ERROR);
        REQUIRE(buf.count() == N);

        for (size_t i = 0; i < N; i++)
        {
            ImageMetadata meta{};
            REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
            REQUIRE(meta.timestamp == 2000 + i);
            REQUIRE(meta.payload_size == IMG_SIZE);

            std::vector<uint8_t> out(IMG_SIZE);
            size_t chunk = IMG_SIZE;
            REQUIRE(buf.get_data_chunk(out.data(), chunk) == ImageBufferError::NO_ERROR);

            for (size_t j = 0; j < IMG_SIZE; j++)
                REQUIRE(out[j] == uint8_t(j + i));

            REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);
        }

        REQUIRE(buf.is_empty());
    }
}

TEST_CASE("initialize_from_flash: corrupted header truncates tail")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 8192;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write 3 images
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);

        for (size_t i = 0; i < 3; i++)
        {
            ImageMetadata meta = make_meta(32, static_cast<uint32_t>(3000 + i));
            REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

            std::vector<uint8_t> payload(meta.payload_size, uint8_t(i));
            REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }
    }

    // Corrupt the second header's magic
    {
        StorageHeader hdr{};
        size_t offset = 0;

        // Read first header to compute its size
        {
            TestBuffer<DirectMemoryAccessor> tmp(acc);
            acc.read(flash_start + 0, reinterpret_cast<uint8_t *>(&hdr), sizeof(StorageHeader));

            offset += sizeof(StorageHeader) + hdr.total_size;
        }

        // Overwrite magic of second header
        uint32_t bad_magic = 0xDEADBEEF;
        acc.write(flash_start + offset, reinterpret_cast<uint8_t *>(&bad_magic), sizeof(bad_magic));
    }

    // Reconstruct
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        ImageBufferError err = buf.initialize_from_flash();

        REQUIRE(err == ImageBufferError::CHECKSUM_ERROR);
        REQUIRE(buf.count() == 1); // only first entry survives

        ImageMetadata meta{};
        REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
        REQUIRE(meta.timestamp == 3000);

        // Drain payload (we don't care about its contents here)
        std::vector<uint8_t> dummy(meta.payload_size);
        size_t chunk = meta.payload_size;
        REQUIRE(buf.get_data_chunk(dummy.data(), chunk) == ImageBufferError::NO_ERROR);
        REQUIRE(chunk == meta.payload_size);

        // Now CRC comparison is valid
        REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);
        REQUIRE(buf.is_empty());
    }
}

TEST_CASE("initialize_from_flash: sequence_id continues correctly")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 8192;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write 2 images
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);

        for (size_t i = 0; i < 2; i++)
        {
            ImageMetadata meta = make_meta(32, static_cast<uint32_t>(4000 + i));
            REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

            std::vector<uint8_t> payload(meta.payload_size, uint8_t(i));
            REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }
    }

    // Reconstruct
    TestBuffer<DirectMemoryAccessor> buf(acc);
    REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);

    // Add a new image — sequence_id should continue from last
    ImageMetadata meta = make_meta(16, 9999);
    REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

    // Read header to check sequence_id
    StorageHeader hdr{};
    acc.read(flash_start + buf.get_tail(), reinterpret_cast<uint8_t *>(&hdr), sizeof(StorageHeader));

    REQUIRE(hdr.sequence_id == 2); // previous were 0 and 1
}

TEST_CASE("initialize_from_flash: metadata wraps across boundary")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    const size_t WRAP_PAYLOAD = 16;
    const size_t FILL_PAYLOAD = 8;

    // 1) Use small entries to push tail near the end.
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);

        while (true)
        {
            size_t tail      = buf.get_tail();
            (void)tail;
            size_t available = buf.available();
            size_t total_wrap_entry =
                sizeof(StorageHeader) + sizeof(ImageMetadata) + WRAP_PAYLOAD + sizeof(crc_t);

            // Stop when we don't have enough space for another filler + the wrapped entry.
            size_t total_filler =
                sizeof(StorageHeader) + sizeof(ImageMetadata) + FILL_PAYLOAD + sizeof(crc_t);

            if (available < total_filler + total_wrap_entry)
                break;

            ImageMetadata meta_fill = make_meta(FILL_PAYLOAD, 1);
            REQUIRE(buf.add_image(meta_fill) == ImageBufferError::NO_ERROR);

            std::vector<uint8_t> payload(FILL_PAYLOAD, 0xAA);
            REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }

        // 2) Now add the wrapped entry that will cross the boundary.
        ImageMetadata meta = make_meta(WRAP_PAYLOAD, 7777);
        REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

        std::vector<uint8_t> payload(meta.payload_size, 0xAB);
        REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
        REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
    }

    // 3) Reconstruct and verify.
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);

        // We only assert that the last entry (timestamp 7777) is present and correct.
        REQUIRE(buf.count() >= 1);

        // Drain earlier entries if any.
        while (true)
        {
            ImageMetadata meta{};
            if (buf.get_image(meta) != ImageBufferError::NO_ERROR)
                break;

            if (meta.timestamp == 7777)
            {
                REQUIRE(meta.payload_size == WRAP_PAYLOAD);
                std::vector<uint8_t> out(meta.payload_size);
                size_t chunk = meta.payload_size;
                REQUIRE(buf.get_data_chunk(out.data(), chunk) == ImageBufferError::NO_ERROR);
                REQUIRE(chunk == meta.payload_size);

                for (size_t i = 0; i < out.size(); i++)
                    REQUIRE(out[i] == 0xAB);

                break;
            }

            // Skip non-target entries.
            std::vector<uint8_t> dummy(meta.payload_size);
            size_t chunk = meta.payload_size;
            REQUIRE(buf.get_data_chunk(dummy.data(), chunk) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);
        }
    }
}

TEST_CASE("initialize_from_flash: payload wraps across boundary")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    const size_t WRAP_PAYLOAD = 100;
    const size_t FILL_PAYLOAD = 8;

    // 1) Fill with small entries to push tail near boundary.
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);

        while (true)
        {
            size_t available = buf.available();
            size_t total_wrap_entry =
                sizeof(StorageHeader) + sizeof(ImageMetadata) + WRAP_PAYLOAD + sizeof(crc_t);
            size_t total_filler =
                sizeof(StorageHeader) + sizeof(ImageMetadata) + FILL_PAYLOAD + sizeof(crc_t);

            if (available < total_filler + total_wrap_entry)
                break;

            ImageMetadata meta_fill = make_meta(FILL_PAYLOAD, 1);
            REQUIRE(buf.add_image(meta_fill) == ImageBufferError::NO_ERROR);

            std::vector<uint8_t> payload(FILL_PAYLOAD, 0x11);
            REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }

        // 2) Add the wrapped entry.
        ImageMetadata meta = make_meta(WRAP_PAYLOAD, 8888);
        REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

        std::vector<uint8_t> payload(meta.payload_size);
        for (size_t i = 0; i < payload.size(); i++)
            payload[i] = uint8_t(i);

        REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
        REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
    }

    // 3) Reconstruct and verify.
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);
        REQUIRE(buf.count() >= 1);

        while (true)
        {
            ImageMetadata meta{};
            if (buf.get_image(meta) != ImageBufferError::NO_ERROR)
                break;

            if (meta.timestamp == 8888)
            {
                REQUIRE(meta.payload_size == WRAP_PAYLOAD);
                std::vector<uint8_t> out(meta.payload_size);
                size_t chunk = meta.payload_size;
                REQUIRE(buf.get_data_chunk(out.data(), chunk) == ImageBufferError::NO_ERROR);

                for (size_t i = 0; i < out.size(); i++)
                    REQUIRE(out[i] == uint8_t(i));

                break;
            }

            std::vector<uint8_t> dummy(meta.payload_size);
            size_t chunk = meta.payload_size;
            REQUIRE(buf.get_data_chunk(dummy.data(), chunk) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);
        }
    }
}

TEST_CASE("initialize_from_flash: trailing CRC wraps across boundary")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    const size_t WRAP_PAYLOAD = 0;   // force minimal entry size
    const size_t FILL_PAYLOAD = 8;

    {
        TestBuffer<DirectMemoryAccessor> buf(acc);

        // 1) Fill until only a small region remains, so CRC is likely to wrap.
        while (true)
        {
            size_t available = buf.available();
            size_t total_wrap_entry =
                sizeof(StorageHeader) + sizeof(ImageMetadata) + WRAP_PAYLOAD + sizeof(crc_t);
            size_t total_filler =
                sizeof(StorageHeader) + sizeof(ImageMetadata) + FILL_PAYLOAD + sizeof(crc_t);

            if (available < total_filler + total_wrap_entry)
                break;

            ImageMetadata meta_fill = make_meta(FILL_PAYLOAD, 1);
            REQUIRE(buf.add_image(meta_fill) == ImageBufferError::NO_ERROR);

            std::vector<uint8_t> payload(FILL_PAYLOAD, 0x22);
            REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }

        // 2) Add the CRC-wrapping entry (no payload, just header+metadata+CRC).
        ImageMetadata meta = make_meta(WRAP_PAYLOAD, 9999);
        REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);
        REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
    }

    // 3) Reconstruct and verify that the last entry is present.
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);
        REQUIRE(buf.count() >= 1);

        bool found = false;
        while (true)
        {
            ImageMetadata meta{};
            if (buf.get_image(meta) != ImageBufferError::NO_ERROR)
                break;

            if (meta.timestamp == 9999)
            {
                REQUIRE(meta.payload_size == WRAP_PAYLOAD);
                found = true;
                break;
            }

            std::vector<uint8_t> dummy(meta.payload_size);
            size_t chunk = meta.payload_size;
            REQUIRE(buf.get_data_chunk(dummy.data(), chunk) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);
        }

        CHECK(found);
    }
}

TEST_CASE("initialize_from_flash: truncated payload causes rejection")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    size_t entry_size = 0;
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        ImageMetadata meta = make_meta(64, 1234);
        REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

        std::vector<uint8_t> payload(meta.payload_size, 0xAA);
        REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
        REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);

        entry_size = buf.size();
    }

    // Corrupt by erasing last half of payload
    size_t corrupt_off = flash_start + entry_size - 20;
    std::vector<uint8_t> ff(20, 0xFF);
    acc.write(corrupt_off, ff.data(), ff.size());

    TestBuffer<DirectMemoryAccessor> buf(acc);
    REQUIRE(buf.initialize_from_flash() == ImageBufferError::CHECKSUM_ERROR);
    REQUIRE(buf.count() == 0);
}

TEST_CASE("initialize_from_flash: truncated third entry stops reconstruction")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 8192;
    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);
    size_t offsets[3];
    // Write 3 entries
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        for (uint i = 0; i < 3; i++)
        {
            offsets[i] = buf.get_tail();
            ImageMetadata meta = make_meta(32, 5000 + i);
            REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);
            std::vector<uint8_t> payload(meta.payload_size, uint8_t(i));
            REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }
    }
    // Truncate third entry
    std::vector<uint8_t> ff(16, 0xFF);
    acc.write(flash_start + offsets[2], ff.data(), ff.size());
    TestBuffer<DirectMemoryAccessor> buf(acc);
    REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);
    REQUIRE(buf.count() == 2);
}

TEST_CASE("initialize_from_flash: non-contiguous sequence IDs stop reconstruction")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size = 8192;
    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);
    // Write two entries normally
    {
        TestBuffer<DirectMemoryAccessor> buf(acc);
        for (uint i = 0; i < 2; i++)
        {
            ImageMetadata meta = make_meta(32, 6000 + i);
            REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);
            std::vector<uint8_t> payload(meta.payload_size, uint8_t(i));
            REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }
    }
    // Corrupt second header's sequence_id 
    StorageHeader hdr{};
    acc.read(flash_start + 0, reinterpret_cast<uint8_t *>(&hdr), sizeof(hdr));
    size_t first_size = sizeof(StorageHeader) + hdr.total_size;
    uint32_t bad_seq = 10;
    acc.write(flash_start + first_size + offsetof(StorageHeader, sequence_id), reinterpret_cast<uint8_t *>(&bad_seq), sizeof(bad_seq));
    TestBuffer<DirectMemoryAccessor> buf(acc);
    REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);
    REQUIRE(buf.count() == 1);
}