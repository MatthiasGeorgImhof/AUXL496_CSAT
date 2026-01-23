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

template <typename Accessor>
class TestableImageBuffer : public ImageBuffer<Accessor>
{
public:
    using Base = ImageBuffer<Accessor>;
    using Base::Base;

    void set_tail_for_test(size_t t) { this->test_set_tail(t); }

    ImageBufferError test_validate_entry(size_t offset,
                                         size_t &entry_size,
                                         uint32_t &seq_id,
                                         ImageMetadata &meta_out)
    {
        return this->validate_entry(offset, entry_size, seq_id, meta_out);
    }
};

static void erase_flash(DirectMemoryAccessor &acc, size_t start, size_t size)
{
    std::vector<uint8_t> blank(size, 0xFF);
    acc.write(start, blank.data(), size);
}

static ImageMetadata make_meta(size_t payload_size, uint32_t ts)
{
    ImageMetadata m{};
    m.timestamp = ts;
    m.payload_size = static_cast<uint32_t>(payload_size);
    m.latitude = 1.0f;
    m.longitude = 2.0f;
    m.producer = METADATA_PRODUCER::CAMERA_1;
    return m;
}

// Write a valid entry at a given logical ring offset using ImageBuffer itself.
// Returns the total entry size (header + metadata + payload + trailing CRC).
static size_t write_valid_entry(DirectMemoryAccessor &acc,
                                size_t offset,
                                const ImageMetadata &meta)
{
    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    buf.set_tail_for_test(offset);
    REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

    std::vector<uint8_t> payload(meta.payload_size);
    for (size_t i = 0; i < payload.size(); i++)
        payload[i] = static_cast<uint8_t>(i & 0xFF);

    REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);

    return sizeof(StorageHeader) +
           sizeof(ImageMetadata) +
           meta.payload_size +
           sizeof(crc_t);
}

TEST_CASE("validate_entry: false-positive header: magic matches but header CRC wrong")
{
    constexpr size_t flash_start = 0x30000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Construct a fake header with correct magic but corrupted CRC
    StorageHeader hdr{};
    hdr.magic = STORAGE_MAGIC;
    hdr.version = STORAGE_HEADER_VERSION;
    hdr.header_size = sizeof(StorageHeader);
    hdr.sequence_id = 123;
    hdr.total_size = 64; // arbitrary

    // Compute correct CRC, then corrupt it
    DefaultChecksumPolicy c;
    c.reset();
    c.update(reinterpret_cast<uint8_t *>(&hdr), offsetof(StorageHeader, header_crc));
    crc_t crc = c.get();
    crc ^= 0xA5A5A5A5; // corrupt

    std::memcpy(reinterpret_cast<uint8_t *>(&hdr) + offsetof(StorageHeader, header_crc),
                &crc, sizeof(crc));

    // Write fake header to flash
    REQUIRE(acc.write(flash_start, reinterpret_cast<uint8_t *>(&hdr), sizeof(hdr)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata meta{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, meta) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: false-positive header: magic matches but total_size impossible")
{
    constexpr size_t flash_start = 0x31000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    StorageHeader hdr{};
    hdr.magic = STORAGE_MAGIC;
    hdr.version = STORAGE_HEADER_VERSION;
    hdr.header_size = sizeof(StorageHeader);
    hdr.sequence_id = 55;
    hdr.total_size = 999999; // impossible

    // Compute correct CRC
    DefaultChecksumPolicy c;
    c.reset();
    c.update(reinterpret_cast<uint8_t *>(&hdr), offsetof(StorageHeader, header_crc));
    crc_t crc = c.get();
    std::memcpy(reinterpret_cast<uint8_t *>(&hdr) + offsetof(StorageHeader, header_crc),
                &crc, sizeof(crc));

    REQUIRE(acc.write(flash_start, reinterpret_cast<uint8_t *>(&hdr), sizeof(hdr)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata meta{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, meta) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: false-positive header: erased region (0xFF) looks like header")
{
    constexpr size_t flash_start = 0x32000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);

    // Fill flash with 0xFF (erased NAND)
    erase_flash(acc, flash_start, flash_size);

    // But force the first 4 bytes to equal STORAGE_MAGIC
    uint32_t magic = STORAGE_MAGIC;
    REQUIRE(acc.write(flash_start, reinterpret_cast<uint8_t *>(&magic), sizeof(magic)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata meta{};

    // Header CRC must fail → CHECKSUM_ERROR
    REQUIRE(buf.test_validate_entry(0, esz, sid, meta) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: false-positive header: header CRC OK but metadata CRC fails")
{
    constexpr size_t flash_start = 0x33000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(32, 1234);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt metadata CRC
    size_t meta_crc_offset = sizeof(StorageHeader) + METADATA_SIZE_WO_CRC;
    uint8_t corrupt = 0xAA;
    REQUIRE(acc.write(flash_start + meta_crc_offset, &corrupt, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: false-positive header: header CRC OK but payload CRC fails")
{
    constexpr size_t flash_start = 0x34000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 5678);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt payload
    uint8_t corrupt = 0x55;
    size_t payload_offset = sizeof(StorageHeader) + sizeof(ImageMetadata) + 10;
    REQUIRE(acc.write(flash_start + payload_offset, &corrupt, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

// -----------------------------------------------------------------------------
// CATEGORY 2: Corrupted header claiming oversized entry
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: header claims entry larger than flash capacity -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x40000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry first
    ImageMetadata meta = make_meta(32, 1111);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Now corrupt the header.total_size to something impossible
    uint32_t huge = 999999; // larger than flash
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&huge),
                      sizeof(huge)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Validator must reject this
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: header claims entry extends past flash end (truncated metadata) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x41000;
    constexpr size_t flash_size = 256; // small to force truncation

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry at offset 0
    ImageMetadata meta = make_meta(64, 2222);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Now corrupt total_size so metadata runs off the end
    uint32_t too_big = flash_size - sizeof(StorageHeader) + 50;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_big),
                      sizeof(too_big)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Metadata read will wrap into erased region → CRC mismatch
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: header claims entry extends past flash end (truncated payload) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x42000;
    constexpr size_t flash_size = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(128, 3333);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt total_size so payload runs off the end
    uint32_t too_big = flash_size - sizeof(StorageHeader) - 10;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_big),
                      sizeof(too_big)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Payload read will hit erased region → CRC mismatch
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: header claims entry extends past flash end (trailing CRC missing) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x43000;
    constexpr size_t flash_size = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(64, 4444);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt total_size so trailing CRC is beyond flash end
    uint32_t too_big = flash_size - sizeof(StorageHeader) - sizeof(ImageMetadata) - meta.payload_size + 20;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_big),
                      sizeof(too_big)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Trailing CRC read will fail → CHECKSUM_ERROR
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: header claims entry larger than metadata+payload+CRC -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x44000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(32, 5555);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Inflate total_size but still within flash
    uint32_t inflated = sizeof(ImageMetadata) + meta.payload_size + sizeof(crc_t) + 50;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&inflated),
                      sizeof(inflated)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Size mismatch → DATA_ERROR
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) != ImageBufferError::NO_ERROR);
}

// -----------------------------------------------------------------------------
// CATEGORY 3: Corrupted header claiming undersized entry
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: header claims entry smaller than metadata size -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x45000;
    constexpr size_t flash_size = 2048;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(64, 6001);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Force total_size smaller than sizeof(ImageMetadata)
    uint32_t too_small = sizeof(ImageMetadata) - 8;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: header claims entry smaller than metadata+payload -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x46000;
    constexpr size_t flash_size = 2048;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(128, 6002);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Shrink total_size so payload cannot fit
    uint32_t too_small = sizeof(ImageMetadata) + meta.payload_size - 20;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: header claims entry smaller than metadata+payload+CRC -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x47000;
    constexpr size_t flash_size = 2048;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(32, 6003);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Shrink total_size so trailing CRC cannot fit
    uint32_t too_small = sizeof(ImageMetadata) + meta.payload_size; // missing CRC
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: header claims entry smaller than actual but still within flash -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x48000;
    constexpr size_t flash_size = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(64, 6004);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Inflate metadata.payload_size but shrink total_size
    // This creates a contradiction: metadata says payload is large,
    // but header says entry is small.
    uint32_t too_small = sizeof(ImageMetadata) + 16 + sizeof(crc_t);
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: header claims entry smaller than actual (wrapped case) -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x49000;
    constexpr size_t flash_size = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Force entry to wrap
    ImageMetadata meta = make_meta(64, 6005);
    size_t offset = flash_size - 40;
    write_valid_entry(acc, offset, meta);

    // Shrink total_size so metadata or payload is truncated
    uint32_t too_small = sizeof(ImageMetadata) + 10; // missing payload + CRC
    REQUIRE(acc.write(flash_start + offset + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, esz, sid, out) != ImageBufferError::NO_ERROR);
}

// -----------------------------------------------------------------------------
// CATEGORY 4: Metadata truncation across wrap
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: metadata truncated across wrap -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x4A000;
    constexpr size_t flash_size = 256; // small to force wrap

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry near the end so metadata wraps
    ImageMetadata meta = make_meta(32, 7001);
    size_t offset = flash_size - 10; // force wrap
    write_valid_entry(acc, offset, meta);

    // Now erase the wrapped portion of metadata
    size_t meta_offset = sizeof(StorageHeader);
    size_t wrapped_pos = (offset + meta_offset + 8) % flash_size; // inside metadata
    uint8_t erased = 0xFF;
    REQUIRE(acc.write(flash_start + wrapped_pos, &erased, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Metadata CRC must fail
    REQUIRE(buf.test_validate_entry(offset, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: metadata truncated because header.total_size too small -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x4B000;
    constexpr size_t flash_size = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry
    ImageMetadata meta = make_meta(64, 7002);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Shrink total_size so metadata cannot fully fit
    uint32_t too_small = sizeof(ImageMetadata) - 4; // impossible
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Structural impossibility → DATA_ERROR
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: metadata truncated because header claims wrap but flash ends early -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x4C000;
    constexpr size_t flash_size = 128; // tiny buffer

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry at offset 0
    ImageMetadata meta = make_meta(16, 7003);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt total_size so metadata wraps but cannot fully fit
    uint32_t too_big = flash_size - sizeof(StorageHeader) + 20;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_big),
                      sizeof(too_big)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Metadata read will hit erased region → CHECKSUM_ERROR
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: metadata CRC correct but second half erased across wrap -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x4D000;
    constexpr size_t flash_size = 256;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write valid entry near end
    ImageMetadata meta = make_meta(32, 7004);
    size_t offset = flash_size - 20;
    write_valid_entry(acc, offset, meta);

    // Erase the wrapped portion of metadata (after CRC)
    size_t meta_start = offset + sizeof(StorageHeader);
    size_t wrap_pos = (meta_start + 12) % flash_size;
    uint8_t erased = 0xFF;
    REQUIRE(acc.write(flash_start + wrap_pos, &erased, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: metadata truncated because flash ends exactly at metadata boundary -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x4E000;
    constexpr size_t flash_size = sizeof(StorageHeader) + sizeof(ImageMetadata) - 2;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry into a larger temp buffer
    ImageMetadata meta = make_meta(16, 7005);
    std::vector<uint8_t> temp(512, 0xFF);
    DirectMemoryAccessor temp_acc(reinterpret_cast<uintptr_t>(temp.data()), temp.size());
    erase_flash(temp_acc, reinterpret_cast<uintptr_t>(temp.data()), temp.size());
    write_valid_entry(temp_acc, 0, meta);

    // Copy only the truncated portion into real flash
    REQUIRE(acc.write(flash_start,
                      temp.data(),
                      flash_size) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    // Metadata cannot be fully read → CHECKSUM_ERROR
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

// -----------------------------------------------------------------------------
// CATEGORY 5: Payload truncation across wrap
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: payload truncated across wrap -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x4F000;
    constexpr size_t flash_size = 256; // small to force wrap

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write a valid entry near the end so payload wraps
    ImageMetadata meta = make_meta(64, 8001);
    size_t offset = flash_size - 20; // force wrap
    write_valid_entry(acc, offset, meta);

    // Erase the wrapped portion of payload
    size_t payload_start = offset + sizeof(StorageHeader) + sizeof(ImageMetadata);
    size_t wrapped_pos = (payload_start + 40) % flash_size; // inside payload
    uint8_t erased = 0xFF;
    REQUIRE(acc.write(flash_start + wrapped_pos, &erased, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: payload truncated because header.total_size too small -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x50000;
    constexpr size_t flash_size = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write valid entry
    ImageMetadata meta = make_meta(128, 8002);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Shrink total_size so payload cannot fully fit
    uint32_t too_small = sizeof(ImageMetadata) + meta.payload_size - 30;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: payload truncated because header claims wrap but flash ends early -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x51000;
    constexpr size_t flash_size = 128; // tiny buffer

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write valid entry at offset 0
    ImageMetadata meta = make_meta(32, 8003);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt total_size so payload wraps but cannot fully fit
    uint32_t too_big = flash_size - sizeof(StorageHeader) - sizeof(ImageMetadata) + 40;
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_big),
                      sizeof(too_big)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: payload CRC correct but second half erased across wrap -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x52000;
    constexpr size_t flash_size = 256;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write valid entry near end
    ImageMetadata meta = make_meta(64, 8004);
    size_t offset = flash_size - 30;
    write_valid_entry(acc, offset, meta);

    // Erase wrapped portion of payload
    size_t payload_start = offset + sizeof(StorageHeader) + sizeof(ImageMetadata);
    size_t wrap_pos = (payload_start + 50) % flash_size;
    uint8_t erased = 0xFF;
    REQUIRE(acc.write(flash_start + wrap_pos, &erased, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: payload truncated because flash ends exactly at payload boundary -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x53000;
    constexpr size_t flash_size =
        sizeof(StorageHeader) + sizeof(ImageMetadata) + 20; // too small for full payload

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write valid entry into a larger temp buffer
    ImageMetadata meta = make_meta(64, 8005);
    std::vector<uint8_t> temp(512, 0xFF);
    DirectMemoryAccessor temp_acc(reinterpret_cast<uintptr_t>(temp.data()), temp.size());
    erase_flash(temp_acc, reinterpret_cast<uintptr_t>(temp.data()), temp.size());
    write_valid_entry(temp_acc, 0, meta);

    // Copy only truncated portion into real flash
    REQUIRE(acc.write(flash_start,
                      temp.data(),
                      flash_size) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

// -----------------------------------------------------------------------------
// CATEGORY 6: Trailing CRC corruption & truncation
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: trailing CRC corrupted -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x54000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 9001);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt trailing CRC
    size_t crc_offset = sizeof(StorageHeader) + sizeof(ImageMetadata) + meta.payload_size;
    uint8_t corrupt = 0xAA;
    REQUIRE(acc.write(flash_start + crc_offset, &corrupt, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: trailing CRC truncated (entry ends before CRC) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x55000;
    constexpr size_t flash_size = 256;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write valid entry
    ImageMetadata meta = make_meta(64, 9002);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Shrink total_size so CRC is missing
    uint32_t too_small = sizeof(ImageMetadata) + meta.payload_size; // missing CRC
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: trailing CRC truncated across wrap -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x56000;
    constexpr size_t flash_size = 256; // small to force wrap

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write entry near end so CRC wraps
    ImageMetadata meta = make_meta(32, 9003);
    size_t offset = flash_size - 10;
    write_valid_entry(acc, offset, meta);

    // Erase wrapped portion of CRC
    size_t crc_start = offset + sizeof(StorageHeader) + sizeof(ImageMetadata) + meta.payload_size;
    size_t wrap_pos = crc_start % flash_size;
    uint8_t erased = 0xFF;
    REQUIRE(acc.write(flash_start + wrap_pos, &erased, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: trailing CRC corrupted but header+metadata+payload valid -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x57000;
    constexpr size_t flash_size = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(128, 9004);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Flip one byte of CRC
    size_t crc_offset = sizeof(StorageHeader) + sizeof(ImageMetadata) + meta.payload_size;
    uint8_t corrupt = 0x5A;
    REQUIRE(acc.write(flash_start + crc_offset, &corrupt, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: trailing CRC missing because flash ends exactly at CRC boundary -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x58000;
    constexpr size_t flash_size =
        sizeof(StorageHeader) + sizeof(ImageMetadata) + 32; // no room for CRC

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Write valid entry into a larger temp buffer
    ImageMetadata meta = make_meta(32, 9005);
    std::vector<uint8_t> temp(512, 0xFF);
    DirectMemoryAccessor temp_acc(reinterpret_cast<uintptr_t>(temp.data()), temp.size());
    erase_flash(temp_acc, reinterpret_cast<uintptr_t>(temp.data()), temp.size());
    write_valid_entry(temp_acc, 0, meta);

    // Copy truncated entry into real flash
    REQUIRE(acc.write(flash_start,
                      temp.data(),
                      flash_size) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(0, esz, sid, out) == ImageBufferError::CHECKSUM_ERROR);
}

// -----------------------------------------------------------------------------
// CATEGORY 7: Multi-entry sequences & overlaps
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: two valid entries back-to-back validate independently")
{
    constexpr size_t flash_start = 0x59000;
    constexpr size_t flash_size = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Entry A
    ImageMetadata metaA = make_meta(64, 10001);
    size_t offA = 0;
    write_valid_entry(acc, offA, metaA);

    // Entry B immediately after A
    ImageMetadata metaB = make_meta(32, 10002);
    size_t offB = offA + sizeof(StorageHeader) + sizeof(ImageMetadata) +
                  metaA.payload_size + sizeof(crc_t);
    write_valid_entry(acc, offB, metaB);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t eszA = 0, eszB = 0;
    uint32_t sidA = 0, sidB = 0;
    ImageMetadata outA{}, outB{};

    REQUIRE(buf.test_validate_entry(offA, eszA, sidA, outA) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.test_validate_entry(offB, eszB, sidB, outB) == ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: corrupted second entry does not affect first entry")
{
    constexpr size_t flash_start = 0x5A000;
    constexpr size_t flash_size = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Entry A
    ImageMetadata metaA = make_meta(64, 10003);
    size_t offA = 0;
    write_valid_entry(acc, offA, metaA);

    // Entry B
    ImageMetadata metaB = make_meta(64, 10004);
    size_t offB = offA + sizeof(StorageHeader) + sizeof(ImageMetadata) +
                  metaA.payload_size + sizeof(crc_t);
    write_valid_entry(acc, offB, metaB);

    // Corrupt payload of entry B
    uint8_t corrupt = 0xAA;
    size_t payloadB = offB + sizeof(StorageHeader) + sizeof(ImageMetadata) + 10;
    REQUIRE(acc.write(flash_start + payloadB, &corrupt, 1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t eszA = 0, eszB = 0;
    uint32_t sidA = 0, sidB = 0;
    ImageMetadata outA{}, outB{};

    REQUIRE(buf.test_validate_entry(offA, eszA, sidA, outA) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.test_validate_entry(offB, eszB, sidB, outB) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted first entry does not affect second entry")
{
    constexpr size_t flash_start = 0x5B000;
    constexpr size_t flash_size = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Entry A
    ImageMetadata metaA = make_meta(64, 10005);
    size_t offA = 0;
    write_valid_entry(acc, offA, metaA);

    // Corrupt trailing CRC of entry A
    size_t crcA = offA + sizeof(StorageHeader) + sizeof(ImageMetadata) +
                  metaA.payload_size;
    uint8_t corrupt = 0x55;
    REQUIRE(acc.write(flash_start + crcA, &corrupt, 1) == AccessorError::NO_ERROR);

    // Entry B
    ImageMetadata metaB = make_meta(32, 10006);
    size_t offB = crcA + sizeof(crc_t);
    write_valid_entry(acc, offB, metaB);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t eszA = 0, eszB = 0;
    uint32_t sidA = 0, sidB = 0;
    ImageMetadata outA{}, outB{};

    REQUIRE(buf.test_validate_entry(offA, eszA, sidA, outA) == ImageBufferError::CHECKSUM_ERROR);
    REQUIRE(buf.test_validate_entry(offB, eszB, sidB, outB) == ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: corrupted header creates false overlap -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x5C000;
    constexpr size_t flash_size = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Entry A
    ImageMetadata metaA = make_meta(32, 10007);
    size_t offA = 0;
    write_valid_entry(acc, offA, metaA);

    // Entry B
    ImageMetadata metaB = make_meta(32, 10008);
    size_t offB = offA + sizeof(StorageHeader) + sizeof(ImageMetadata) +
                  metaA.payload_size + sizeof(crc_t);
    write_valid_entry(acc, offB, metaB);

    // Corrupt header.total_size of entry A to overlap into entry B
    uint32_t inflated = 5000; // absurdly large
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&inflated),
                      sizeof(inflated)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t eszA = 0, eszB = 0;
    uint32_t sidA = 0, sidB = 0;
    ImageMetadata outA{}, outB{};

    REQUIRE(buf.test_validate_entry(offA, eszA, sidA, outA) == ImageBufferError::CHECKSUM_ERROR);

    // Entry B must still validate
    REQUIRE(buf.test_validate_entry(offB, eszB, sidB, outB) == ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: corrupted first entry must not shift second entry offset")
{
    constexpr size_t flash_start = 0x5D000;
    constexpr size_t flash_size = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Entry A
    ImageMetadata metaA = make_meta(64, 10009);
    size_t offA = 0;
    write_valid_entry(acc, offA, metaA);

    // Entry B
    ImageMetadata metaB = make_meta(64, 10010);
    size_t offB = offA + sizeof(StorageHeader) + sizeof(ImageMetadata) +
                  metaA.payload_size + sizeof(crc_t);
    write_valid_entry(acc, offB, metaB);

    // Corrupt header.total_size of entry A to claim a smaller entry
    uint32_t too_small = sizeof(ImageMetadata) + 8; // impossible
    REQUIRE(acc.write(flash_start + offsetof(StorageHeader, total_size),
                      reinterpret_cast<uint8_t *>(&too_small),
                      sizeof(too_small)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t eszA = 0, eszB = 0;
    uint32_t sidA = 0, sidB = 0;
    ImageMetadata outA{}, outB{};

    REQUIRE(buf.test_validate_entry(offA, eszA, sidA, outA) != ImageBufferError::NO_ERROR);

    // Entry B must still validate at the correct offset
    REQUIRE(buf.test_validate_entry(offB, eszB, sidB, outB) == ImageBufferError::NO_ERROR);
}

// -----------------------------------------------------------------------------
// CATEGORY 8: Rotation-invariant / fuzz-style corruption
// -----------------------------------------------------------------------------

static void write_entry_at_offset(DirectMemoryAccessor& acc,
                                  size_t /*flash_start*/,
                                  size_t /*flash_size*/,
                                  size_t offset,
                                  const ImageMetadata& meta)
{
    // Helper: write_valid_entry but respecting ring semantics externally
    write_valid_entry(acc, offset, meta);
}

TEST_CASE("validate_entry: same entry at multiple ring offsets with identical corruption -> all fail")
{
    constexpr size_t flash_start = 0x5E000;
    constexpr size_t flash_size  = 512;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 11001);

    // Place the same logical entry at three different offsets
    const size_t offsets[] = {0, 100, 300};

    for (size_t off : offsets)
    {
        write_entry_at_offset(acc, flash_start, flash_size, off, meta);

        // Corrupt a payload byte at a fixed logical position
        size_t payload_start = off + sizeof(StorageHeader) + sizeof(ImageMetadata);
        size_t corrupt_pos   = (payload_start + 10) % flash_size;
        uint8_t corrupt = 0xAB;
        REQUIRE(acc.write(flash_start + corrupt_pos, &corrupt, 1)
                == AccessorError::NO_ERROR);
    }

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    for (size_t off : offsets)
    {
        size_t esz = 0;
        uint32_t sid = 0;
        ImageMetadata out{};
        REQUIRE(buf.test_validate_entry(off, esz, sid, out)
                == ImageBufferError::CHECKSUM_ERROR);
    }
}

TEST_CASE("validate_entry: random-looking payload corruption never validates (fixed seed)")
{
    constexpr size_t flash_start = 0x5F000;
    constexpr size_t flash_size  = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(128, 11002);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Apply a deterministic 'random' pattern over the payload
    size_t payload_start = sizeof(StorageHeader) + sizeof(ImageMetadata);
    for (size_t i = 0; i < meta.payload_size; ++i)
    {
        uint8_t v = static_cast<uint8_t>((i * 37u + 91u) & 0xFFu);
        REQUIRE(acc.write(flash_start + payload_start + i, &v, 1)
                == AccessorError::NO_ERROR);
    }

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out)
            == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: random-looking header corruption never accidentally validates")
{
    constexpr size_t flash_start = 0x60000;
    constexpr size_t flash_size  = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Start from a valid entry
    ImageMetadata meta = make_meta(64, 11003);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Overwrite the header (except magic) with a deterministic random pattern
    std::vector<uint8_t> hdr_buf(sizeof(StorageHeader));
    REQUIRE(acc.read(flash_start, hdr_buf.data(), hdr_buf.size())
            == AccessorError::NO_ERROR);

    for (size_t i = sizeof(uint32_t); i < hdr_buf.size(); ++i) // skip magic
    {
        hdr_buf[i] = static_cast<uint8_t>((i * 53u + 17u) & 0xFFu);
    }

    REQUIRE(acc.write(flash_start, hdr_buf.data(), hdr_buf.size())
            == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out)
            == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: multi-byte random corruption across metadata+payload -> never NO_ERROR")
{
    constexpr size_t flash_start = 0x61000;
    constexpr size_t flash_size  = 1024;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(96, 11004);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt across boundary: last bytes of metadata + first bytes of payload
    size_t meta_end       = sizeof(StorageHeader) + sizeof(ImageMetadata);
    size_t corrupt_start  = meta_end - 8; // 8 bytes before metadata end
    size_t corrupt_length = 24;          // spans into payload

    for (size_t i = 0; i < corrupt_length; ++i)
    {
        uint8_t v = static_cast<uint8_t>((i * 73u + 29u) & 0xFFu);
        REQUIRE(acc.write(flash_start + corrupt_start + i, &v, 1)
                == AccessorError::NO_ERROR);
    }

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(0, esz, sid, out)
            != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: rotation-invariant corruption on wrapped entry -> consistent CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x62000;
    constexpr size_t flash_size  = 256;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 11005);

    // Place entry such that it wraps
    size_t offset = 200;
    write_valid_entry(acc, offset, meta);

    // Corrupt at logical payload index 10, but wrapped physically
    size_t payload_start = offset + sizeof(StorageHeader) + sizeof(ImageMetadata);
    size_t corrupt_pos   = (payload_start + 10) % flash_size;
    uint8_t corrupt = 0xDE;
    REQUIRE(acc.write(flash_start + corrupt_pos, &corrupt, 1)
            == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t esz = 0;
    uint32_t sid = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, esz, sid, out)
            == ImageBufferError::CHECKSUM_ERROR);
}
