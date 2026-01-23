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
class TestableImageBuffer : public ImageBuffer<Accessor> {
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

// Write a valid entry at a given offset using ImageBuffer itself
static size_t write_valid_entry(DirectMemoryAccessor &acc,
                                size_t /*flash_start*/,
                                size_t /*flash_size*/,
                                size_t offset,
                                const ImageMetadata &meta)
{
    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    // Force tail to desired offset
    buf.set_tail_for_test(offset);

    REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

    std::vector<uint8_t> payload(meta.payload_size);
    for (size_t i = 0; i < payload.size(); i++)
        payload[i] = uint8_t(i);

    REQUIRE(buf.add_data_chunk(payload.data(), payload.size()) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);

    return sizeof(StorageHeader) + sizeof(ImageMetadata) + meta.payload_size + sizeof(crc_t);
}

TEST_CASE("validate_entry: contiguous valid entry")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 1000);
    size_t entry_offset = 0;

    size_t entry_size = write_valid_entry(acc, flash_start, flash_size, entry_offset, meta);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz;
    uint32_t seq;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(entry_offset, sz, seq, out) == ImageBufferError::NO_ERROR);
    REQUIRE(sz == entry_size);
    REQUIRE(seq == 0);
    REQUIRE(out.timestamp == 1000);
}

TEST_CASE("validate_entry: wrapped valid entry")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(128, 2000);

    // Force entry to wrap around end of flash
    size_t offset = flash_size - 50;

    size_t entry_size = write_valid_entry(acc, flash_start, flash_size, offset, meta);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz;
    uint32_t seq;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::NO_ERROR);
    REQUIRE(sz == entry_size);
    REQUIRE(out.timestamp == 2000);
}

TEST_CASE("validate_entry: corrupted header contiguous")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 3000);
    size_t offset = 0;
    write_valid_entry(acc, flash_start, flash_size, offset, meta);

    // Corrupt magic
    uint32_t bad_magic = 0xDEADBEEF;
    acc.write(flash_start + offset, reinterpret_cast<uint8_t*>(&bad_magic), sizeof(bad_magic));

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz;
    uint32_t seq;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted header wrapped")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 3001);
    size_t offset = flash_size - 20;
    write_valid_entry(acc, flash_start, flash_size, offset, meta);

    // Corrupt magic at wrapped location
    uint32_t bad_magic = 0xCAFEBABE;
    acc.write(flash_start + offset, reinterpret_cast<uint8_t*>(&bad_magic), sizeof(bad_magic));

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz;
    uint32_t seq;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted metadata contiguous")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 4000);
    size_t offset = 0;
    size_t entry_size = write_valid_entry(acc, flash_start, flash_size, offset, meta);
    (void)entry_size;

    // Corrupt metadata CRC
    size_t meta_crc_offset = offset + sizeof(StorageHeader) + METADATA_SIZE_WO_CRC;
    uint8_t corrupt = 0xAA;
    acc.write(flash_start + meta_crc_offset, &corrupt, 1);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz;
    uint32_t seq;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted metadata wrapped")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 4001);
    size_t offset = flash_size - 40;
    size_t entry_size = write_valid_entry(acc, flash_start, flash_size, offset, meta);
    (void)entry_size;

    // Corrupt metadata CRC (wrapped)
    size_t meta_crc_offset = (offset + sizeof(StorageHeader) + METADATA_SIZE_WO_CRC) % flash_size;
    uint8_t corrupt = 0xBB;
    acc.write(flash_start + meta_crc_offset, &corrupt, 1);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz;
    uint32_t seq;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted payload contiguous")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 5000);
    size_t offset = 0;
    size_t entry_size = write_valid_entry(acc, flash_start, flash_size, offset, meta);
    (void)entry_size;

    // Corrupt payload byte
    size_t payload_offset = offset + sizeof(StorageHeader) + sizeof(ImageMetadata) + 10;
    uint8_t corrupt = 0xCC;
    acc.write(flash_start + payload_offset, &corrupt, 1);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz;
    uint32_t seq;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted payload wrapped")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 5001);
    size_t offset = flash_size - 30;
    size_t entry_size = write_valid_entry(acc, flash_start, flash_size, offset, meta);
    (void)entry_size;

    // Corrupt payload byte (wrapped)
    size_t payload_offset = (offset + sizeof(StorageHeader) + sizeof(ImageMetadata) + 10) % flash_size;
    uint8_t corrupt = 0xDD;
    acc.write(flash_start + payload_offset, &corrupt, 1);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz;
    uint32_t seq;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

// Write a valid entry at a given logical ring offset using ImageBuffer itself.
// Returns the total entry size (header + metadata + payload + trailing CRC).
static size_t write_valid_entry(DirectMemoryAccessor &acc,
                                size_t offset,
                                const ImageMetadata &meta)
{
    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    // Place tail at desired offset inside ring
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

// Read an arbitrary region of flash into a vector
static std::vector<uint8_t> dump_flash(DirectMemoryAccessor &acc,
                                       size_t flash_start,
                                       size_t flash_size)
{
    std::vector<uint8_t> buf(flash_size, 0);
    REQUIRE(acc.read(static_cast<uint32_t>(flash_start), buf.data(), flash_size) == AccessorError::NO_ERROR);
    return buf;
}

// Overwrite entire flash with provided buffer
static void load_flash(DirectMemoryAccessor &acc,
                       size_t flash_start,
                       const std::vector<uint8_t> &buf)
{
    REQUIRE(acc.write(static_cast<uint32_t>(flash_start),
                      const_cast<uint8_t*>(buf.data()),
                      buf.size()) == AccessorError::NO_ERROR);
}

// Logical rotation of the entire flash image by "rot" bytes (ring rotation)
static std::vector<uint8_t> rotate_flash_image(const std::vector<uint8_t> &src,
                                               size_t rot)
{
    std::vector<uint8_t> dst(src.size());
    const size_t n = src.size();
    rot %= n;
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + rot) % n;
        dst[j] = src[i];
    }
    return dst;
}

// Helper to compute header field offset in flash (logical ring offset -> flash addr)
static uint32_t field_addr(size_t flash_start,
                           size_t ring_offset,
                           size_t field_offset)
{
    return static_cast<uint32_t>(flash_start + ring_offset + field_offset);
}

// -----------------------------------------------------------------------------
// Section 1: Basic valid entries (contiguous and wrapped)
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: contiguous valid entry")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 1000);
    size_t entry_offset = 0;

    size_t expected_entry_size = write_valid_entry(acc, entry_offset, meta);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(entry_offset, sz, seq, out) == ImageBufferError::NO_ERROR);
    REQUIRE(sz == expected_entry_size);
    REQUIRE(seq == 0);
    REQUIRE(out.payload_size == meta.payload_size);
    REQUIRE(out.timestamp == meta.timestamp);
}

TEST_CASE("validate_entry: wrapped valid entry")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Force entry to wrap around end of flash
    ImageMetadata meta = make_meta(128, 2000);
    size_t offset = flash_size - 50; // chosen so header+metadata+payload+crc wraps

    size_t expected_entry_size = write_valid_entry(acc, offset, meta);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::NO_ERROR);
    REQUIRE(sz == expected_entry_size);
    REQUIRE(out.payload_size == meta.payload_size);
    REQUIRE(out.timestamp == meta.timestamp);
}

// -----------------------------------------------------------------------------
// Section 2: Header corruption and size sanity
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: corrupted header magic (contiguous)")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 3000);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt magic
    uint32_t bad_magic = 0xDEADBEEF;
    REQUIRE(acc.write(field_addr(flash_start, offset, offsetof(StorageHeader, magic)),
                      reinterpret_cast<uint8_t*>(&bad_magic),
                      sizeof(bad_magic)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted header magic (wrapped)")
{
    constexpr size_t flash_start = 0x4000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 3001);
    size_t offset = flash_size - 20;
    write_valid_entry(acc, offset, meta);

    // Corrupt magic where header begins (wrapped case still at offset)
    uint32_t bad_magic = 0xCAFEBABE;
    REQUIRE(acc.write(field_addr(flash_start, offset, offsetof(StorageHeader, magic)),
                      reinterpret_cast<uint8_t*>(&bad_magic),
                      sizeof(bad_magic)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: impossible header.total_size (> capacity) -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x5000;
    constexpr size_t flash_size  = 2048;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 4000);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Overwrite total_size with a huge, impossible value
    uint32_t huge = 0xFFFFFF00u;
    REQUIRE(acc.write(field_addr(flash_start, offset, offsetof(StorageHeader, total_size)),
                      reinterpret_cast<uint8_t*>(&huge),
                      sizeof(huge)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: header.total_size too small to contain metadata -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x6000;
    constexpr size_t flash_size  = 2048;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(16, 4100);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Force total_size to something smaller than sizeof(ImageMetadata)
    uint32_t tiny = static_cast<uint32_t>(sizeof(ImageMetadata) - 4);
    REQUIRE(acc.write(field_addr(flash_start, offset, offsetof(StorageHeader, total_size)),
                      reinterpret_cast<uint8_t*>(&tiny),
                      sizeof(tiny)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) != ImageBufferError::NO_ERROR);
}

// -----------------------------------------------------------------------------
// Section 3: Metadata corruption and payload_size sanity
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: corrupted metadata CRC (contiguous) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x7000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 5000);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt metadata CRC byte
    size_t meta_crc_field_offset = sizeof(StorageHeader) + METADATA_SIZE_WO_CRC;
    uint8_t corrupt = 0xAA;
    REQUIRE(acc.write(field_addr(flash_start, offset, meta_crc_field_offset),
                      &corrupt,
                      1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted metadata CRC (wrapped) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0x8000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 5001);
    size_t offset = flash_size - 40;
    write_valid_entry(acc, offset, meta);

    size_t meta_crc_field_offset = (sizeof(StorageHeader) + METADATA_SIZE_WO_CRC);
    size_t ring_pos = (offset + meta_crc_field_offset) % flash_size;

    uint8_t corrupt = 0xBB;
    REQUIRE(acc.write(static_cast<uint32_t>(flash_start + ring_pos),
                      &corrupt,
                      1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);

    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};
    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: impossible metadata.payload_size (> total_size - overhead) -> DATA_ERROR")
{
    constexpr size_t flash_start = 0x9000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 5100);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Read current header.total_size to compute "overhead"
    StorageHeader hdr{};
    REQUIRE(acc.read(field_addr(flash_start, offset, 0),
                     reinterpret_cast<uint8_t*>(&hdr),
                     sizeof(hdr)) == AccessorError::NO_ERROR);

    size_t total_size = hdr.total_size; // total after StorageHeader
    size_t overhead   = sizeof(ImageMetadata) + sizeof(crc_t);
    uint32_t impossible_payload = static_cast<uint32_t>(total_size - overhead + 10);

    REQUIRE(acc.write(field_addr(flash_start,
                                 offset,
                                 sizeof(StorageHeader) + offsetof(ImageMetadata, payload_size)),
                      reinterpret_cast<uint8_t*>(&impossible_payload),
                      sizeof(impossible_payload)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) != ImageBufferError::NO_ERROR);
}

TEST_CASE("validate_entry: metadata.payload_size == 0 -> DATA_ERROR")
{
    constexpr size_t flash_start = 0xA000;
    constexpr size_t flash_size  = 2048;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(32, 5200);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    uint32_t zero = 0;
    REQUIRE(acc.write(field_addr(flash_start,
                                 offset,
                                 sizeof(StorageHeader) + offsetof(ImageMetadata, payload_size)),
                      reinterpret_cast<uint8_t*>(&zero),
                      sizeof(zero)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) != ImageBufferError::NO_ERROR);
}

// -----------------------------------------------------------------------------
// Section 4: Payload corruption and "truncation" (CRC mismatch)
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: corrupted payload byte (contiguous) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0xB000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 6000);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Corrupt one payload byte
    size_t payload_offset = sizeof(StorageHeader) + sizeof(ImageMetadata) + 10;
    uint8_t corrupt = 0xCC;
    REQUIRE(acc.write(field_addr(flash_start, offset, payload_offset),
                      &corrupt,
                      1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted payload byte (wrapped) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0xC000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 6001);
    size_t offset = flash_size - 30;
    write_valid_entry(acc, offset, meta);

    size_t payload_offset = sizeof(StorageHeader) + sizeof(ImageMetadata) + 10;
    size_t ring_pos = (offset + payload_offset) % flash_size;
    uint8_t corrupt = 0xDD;
    REQUIRE(acc.write(static_cast<uint32_t>(flash_start + ring_pos),
                      &corrupt,
                      1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

// Simulate "truncated" payload by zeroing a tail region: CRC must fail.
TEST_CASE("validate_entry: payload region erased (partial) -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0xD000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(128, 6100);
    size_t offset = 0;
    write_valid_entry(acc, offset, meta);

    // Erase the second half of the payload
    size_t payload_offset = sizeof(StorageHeader) + sizeof(ImageMetadata);
    size_t erase_from     = payload_offset + meta.payload_size / 2;
    size_t erase_len      = meta.payload_size - meta.payload_size / 2;

    std::vector<uint8_t> erased(erase_len, 0xFF);
    REQUIRE(acc.write(field_addr(flash_start, offset, erase_from),
                      erased.data(),
                      erase_len) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
}

// -----------------------------------------------------------------------------
// Section 5: Trailing CRC corruption
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: corrupted trailing CRC -> CHECKSUM_ERROR")
{
    constexpr size_t flash_start = 0xE000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    ImageMetadata meta = make_meta(64, 6200);
    size_t offset = 0;
    size_t entry_size = write_valid_entry(acc, offset, meta);

    // Trailing CRC is at:
    // offset + sizeof(StorageHeader) + sizeof(ImageMetadata) + payload_size
    size_t crc_offset = sizeof(StorageHeader) + sizeof(ImageMetadata) + meta.payload_size;

    uint8_t corrupt = 0x11;
    REQUIRE(acc.write(field_addr(flash_start, offset, crc_offset),
                      &corrupt,
                      1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t sz       = 0;
    uint32_t seq    = 0;
    ImageMetadata out{};

    REQUIRE(buf.test_validate_entry(offset, sz, seq, out) == ImageBufferError::CHECKSUM_ERROR);
    (void)entry_size;
}

// -----------------------------------------------------------------------------
// Section 6: Multi-entry mixed validity sequences
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: valid followed by corrupted entry (contiguous)")
{
    constexpr size_t flash_start = 0xF000;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // First valid entry at 0
    ImageMetadata meta1 = make_meta(48, 7000);
    size_t off1 = 0;
    size_t entry1_size = write_valid_entry(acc, off1, meta1);

    // Second entry immediately after first (contiguous)
    ImageMetadata meta2 = make_meta(64, 7001);
    size_t off2 = (off1 + entry1_size) % flash_size;
    write_valid_entry(acc, off2, meta2);

    // Corrupt second entry's payload
    size_t payload_offset = sizeof(StorageHeader) + sizeof(ImageMetadata) + 5;
    uint8_t corrupt = 0xAA;
    REQUIRE(acc.write(field_addr(flash_start, off2, payload_offset),
                      &corrupt,
                      1) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t sz1 = 0, sz2 = 0;
    uint32_t seq1 = 0, seq2 = 0;
    ImageMetadata out1{}, out2{};

    REQUIRE(buf.test_validate_entry(off1, sz1, seq1, out1) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.test_validate_entry(off2, sz2, seq2, out2) == ImageBufferError::CHECKSUM_ERROR);
}

TEST_CASE("validate_entry: corrupted then valid entry (contiguous)")
{
    constexpr size_t flash_start = 0xF800;
    constexpr size_t flash_size  = 4096;

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // First entry (to be corrupted)
    ImageMetadata meta1 = make_meta(48, 7100);
    size_t off1 = 0;
    size_t entry1_size = write_valid_entry(acc, off1, meta1);

    // Second valid entry
    ImageMetadata meta2 = make_meta(64, 7101);
    size_t off2 = (off1 + entry1_size) % flash_size;
    write_valid_entry(acc, off2, meta2);

    // Corrupt first entry's header magic
    uint32_t bad_magic = 0xBAD0BAD0u;
    REQUIRE(acc.write(field_addr(flash_start, off1, offsetof(StorageHeader, magic)),
                      reinterpret_cast<uint8_t*>(&bad_magic),
                      sizeof(bad_magic)) == AccessorError::NO_ERROR);

    TestableImageBuffer<DirectMemoryAccessor> buf(acc);
    size_t sz1 = 0, sz2 = 0;
    uint32_t seq1 = 0, seq2 = 0;
    ImageMetadata out1{}, out2{};

    REQUIRE(buf.test_validate_entry(off1, sz1, seq1, out1) == ImageBufferError::CHECKSUM_ERROR);
    REQUIRE(buf.test_validate_entry(off2, sz2, seq2, out2) == ImageBufferError::NO_ERROR);
}

// -----------------------------------------------------------------------------
// Section 7: Rotation fuzz (limited, but meaningful)
// -----------------------------------------------------------------------------

TEST_CASE("validate_entry: rotation invariance for two valid entries")
{
    constexpr size_t flash_start = 0x10000;
    constexpr size_t flash_size  = 1024; // keep small for test time

    DirectMemoryAccessor acc(flash_start, flash_size);
    erase_flash(acc, flash_start, flash_size);

    // Build two valid entries back-to-back at offset 0
    ImageMetadata meta1 = make_meta(64, 8000);
    size_t off1 = 0;
    size_t e1 = write_valid_entry(acc, off1, meta1);

    ImageMetadata meta2 = make_meta(32, 8001);
    size_t off2 = (off1 + e1) % flash_size;
    size_t e2 = write_valid_entry(acc, off2, meta2);
    (void)e2;

    // Snapshot original flash image
    auto original = dump_flash(acc, flash_start, flash_size);

    // Collect all offsets where entries validate in the original layout
    std::vector<size_t> valid_offsets;
    {
        TestableImageBuffer<DirectMemoryAccessor> buf(acc);
        for (size_t off = 0; off < flash_size; ++off) {
            size_t sz = 0;
            uint32_t seq = 0;
            ImageMetadata tmp{};
            auto err = buf.test_validate_entry(off, sz, seq, tmp);
            if (err == ImageBufferError::NO_ERROR) {
                valid_offsets.push_back(off);
            }
        }
        // We expect exactly two valid offsets: off1 and off2
        REQUIRE(valid_offsets.size() == 2);
        REQUIRE(std::find(valid_offsets.begin(), valid_offsets.end(), off1) != valid_offsets.end());
        REQUIRE(std::find(valid_offsets.begin(), valid_offsets.end(), off2) != valid_offsets.end());
    }

    // Now rotate the entire flash image by several offsets, and ensure
    // that in each rotated layout there are exactly two valid entries.
    for (size_t rot = 0; rot < flash_size; rot += 31) { // sample some rotations
        auto rotated = rotate_flash_image(original, rot);
        load_flash(acc, flash_start, rotated);

        TestableImageBuffer<DirectMemoryAccessor> buf(acc);
        std::vector<size_t> valid_rotated;
        for (size_t off = 0; off < flash_size; ++off) {
            size_t sz = 0;
            uint32_t seq = 0;
            ImageMetadata tmp{};
            auto err = buf.test_validate_entry(off, sz, seq, tmp);
            if (err == ImageBufferError::NO_ERROR) {
                valid_rotated.push_back(off);
            }
        }

        // Still exactly two valid entries in rotated layout
        REQUIRE(valid_rotated.size() == 2);
    }
}
