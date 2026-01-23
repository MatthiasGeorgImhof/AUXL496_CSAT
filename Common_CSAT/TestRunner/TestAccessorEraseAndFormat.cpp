#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "imagebuffer/DirectMemoryAccessor.hpp"
#include "imagebuffer/MT29F4G01Accessor.hpp"
#include "ImageBuffer.hpp"
#include "imagebuffer/storageheader.hpp"
#include "imagebuffer/image.hpp"

#include "imagebuffer/configurable_memory_accessor.hpp"

// static void dump_flash(const std::vector<uint8_t> &mem, size_t bytes = 256)
// {
//     printf("\nFLASH DUMP (%zu bytes):\n", bytes);
//     for (size_t i = 0; i < bytes; i++)
//     {
//         if (i % 16 == 0)
//             printf("%04zu: ", i);
//         printf("%02X ", mem[i]);
//         if (i % 16 == 15)
//             printf("\n");
//     }
//     printf("\n");
// }

// -----------------------------------------------------------------------------
// Helper: simple mock transport for MT29F4G01Accessor
// -----------------------------------------------------------------------------
struct MockSPITransport
{
    struct config_type
    {
        using mode_tag = stream_mode_tag;
    };

    std::vector<uint8_t> log;

    bool write(const uint8_t *data, size_t size)
    {
        log.insert(log.end(), data, data + size);
        return true;
    }

    bool read(uint8_t *data, size_t size)
    {
        std::fill(data, data + size, 0x00);
        return true;
    }
};

// -----------------------------------------------------------------------------
// Group 1: DirectMemoryAccessor erase semantics
// -----------------------------------------------------------------------------
TEST_CASE("DirectMemoryAccessor: erase one byte")
{
    DirectMemoryAccessor acc(0x1000, 16);
    auto &mem = acc.getFlashMemory();

    // Fill with pattern
    for (size_t i = 0; i < mem.size(); i++)
        mem[i] = uint8_t(i);

    REQUIRE(acc.erase(0x1000 + 5) == AccessorError::NO_ERROR);

    CHECK(mem[5] == 0xFF);
    CHECK(mem[4] == 4);
    CHECK(mem[6] == 6);
}

TEST_CASE("DirectMemoryAccessor: erase out of range")
{
    DirectMemoryAccessor acc(0x1000, 16);
    REQUIRE(acc.erase(0x1000 + 100) == AccessorError::OUT_OF_BOUNDS);
}

// -----------------------------------------------------------------------------
// Group 2: DirectMemoryAccessor format semantics
// -----------------------------------------------------------------------------
TEST_CASE("DirectMemoryAccessor: format wipes entire region")
{
    DirectMemoryAccessor acc(0x1000, 32);
    auto &mem = acc.getFlashMemory();

    std::fill(mem.begin(), mem.end(), 0x12);
    acc.format();

    for (auto b : mem)
        CHECK(b == 0xFF);
}

// -----------------------------------------------------------------------------
// Group 3: BufferedAccessor erase + format forwarding: deleted functionality
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Group 4A: ImageBuffer erase-on-pop behavior (RAM/NOR)
// -----------------------------------------------------------------------------
TEST_CASE("ImageBuffer (RAM/NOR): pop_image removes single entry")
{
    DirectMemoryAccessor acc(0x4000, 1024);
    acc.format();

    using Buf = ImageBuffer<DirectMemoryAccessor>;
    Buf buf(acc);

    REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);
    CHECK(buf.is_empty());

    // Write one image
    ImageMetadata meta{};
    meta.payload_size = 4;
    REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

    uint8_t payload[4] = {10, 11, 12, 13};
    REQUIRE(buf.add_data_chunk(payload, 4) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);

    CHECK(buf.count() == 1);

    // Read before pop (matches current pop_image contract)
    ImageMetadata out_meta{};
    REQUIRE(buf.get_image(out_meta) == ImageBufferError::NO_ERROR);
    CHECK(out_meta.payload_size == 4);

    uint8_t out[4];
    size_t chunk = 4;
    REQUIRE(buf.get_data_chunk(out, chunk) == ImageBufferError::NO_ERROR);
    CHECK(chunk == 4);

    // Now pop
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    CHECK(buf.is_empty());
    CHECK(buf.count() == 0);
}

TEST_CASE("ImageBuffer (RAM/NOR): reconstruction after popping entries")
{
    DirectMemoryAccessor acc(0x5000, 2048);
    acc.format();

    using Buf = ImageBuffer<DirectMemoryAccessor>;

    {
        Buf buf(acc);
        REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);

        // Write 3 entries
        for (int i = 0; i < 3; i++)
        {
            ImageMetadata m{};
            m.payload_size = 4;
            m.timestamp = uint64_t(100) + uint64_t(i);
            REQUIRE(buf.add_image(m) == ImageBufferError::NO_ERROR);

            uint8_t p[4] = {
                uint8_t(i),
                uint8_t(i + 1),
                uint8_t(i + 2),
                uint8_t(i + 3)};
            REQUIRE(buf.add_data_chunk(p, 4) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }

        CHECK(buf.count() == 3);

        // Drain + pop first entry
        ImageMetadata meta{};
        REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);

        std::vector<uint8_t> out(meta.payload_size);
        size_t chunk = meta.payload_size;
        REQUIRE(buf.get_data_chunk(out.data(), chunk) == ImageBufferError::NO_ERROR);
        CHECK(chunk == meta.payload_size);

        // printf("BEFORE POP:\n");
        //dump_flash(acc.getFlashMemory());

        REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

        // printf("AFTER POP:\n");
        //dump_flash(acc.getFlashMemory());
        CHECK(buf.count() == 2);
    }

    // Reconstruct from RAM/NOR
    Buf buf(acc);
    REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);

    // With byte-addressable erase and no alignment, headers of entries 2 and 3 survive
    CHECK(buf.count() == 2);
}

// -----------------------------------------------------------------------------
// Group 4B: ImageBuffer erase-on-pop behavior (NAND)
// -----------------------------------------------------------------------------
TEST_CASE("ImageBuffer (NAND): pop_image erases exactly one entry's blocks")
{
    ConfigurableMemoryAccessor acc(0x6000, 1024, 16); // 16-byte erase blocks

    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;
    Buf buf(acc);

    REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);
    CHECK(buf.is_empty());

    // Write one image
    ImageMetadata meta{};
    meta.payload_size = 4;
    REQUIRE(buf.add_image(meta) == ImageBufferError::NO_ERROR);

    uint8_t payload[4] = {10, 11, 12, 13};
    REQUIRE(buf.add_data_chunk(payload, 4) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);

    CHECK(buf.count() == 1);

    // Read before pop
    ImageMetadata out_meta{};
    REQUIRE(buf.get_image(out_meta) == ImageBufferError::NO_ERROR);
    CHECK(out_meta.payload_size == 4);

    uint8_t out[4];
    size_t chunk = 4;
    REQUIRE(buf.get_data_chunk(out, chunk) == ImageBufferError::NO_ERROR);
    CHECK(chunk == 4);

    // Pop
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    CHECK(buf.is_empty());
    CHECK(buf.count() == 0);
}

TEST_CASE("ImageBuffer (NAND): reconstruction after popping entries")
{
    ConfigurableMemoryAccessor acc(0x7000, 2048, 16); // NAND-like: 16-byte blocks

    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;

    {
        Buf buf(acc);
        REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);

        // Write 3 entries
        for (int i = 0; i < 3; i++)
        {
            ImageMetadata m{};
            m.payload_size = 4;
            m.timestamp = uint64_t(100) + uint64_t(i);
            REQUIRE(buf.add_image(m) == ImageBufferError::NO_ERROR);

            uint8_t p[4] = {
                uint8_t(i),
                uint8_t(i + 1),
                uint8_t(i + 2),
                uint8_t(i + 3)};
            REQUIRE(buf.add_data_chunk(p, 4) == ImageBufferError::NO_ERROR);
            REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
        }

        CHECK(buf.count() == 3);

        // Drain + pop first entry
        ImageMetadata meta{};
        REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);

        std::vector<uint8_t> out(meta.payload_size);
        size_t chunk = meta.payload_size;
        REQUIRE(buf.get_data_chunk(out.data(), chunk) == ImageBufferError::NO_ERROR);
        CHECK(chunk == meta.payload_size);

            // printf("BEFORE POP:\n");
        //dump_flash(acc.raw());

        REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

        // printf("AFTER POP:\n");
        //dump_flash(acc.raw());
        CHECK(buf.count() == 2);
    }

    // Reconstruct from NAND-like medium
    Buf buf(acc);
    REQUIRE(buf.initialize_from_flash() == ImageBufferError::NO_ERROR);

    // With page alignment and 16-byte erase blocks,
    // popping the first entry must leave the next two intact.
    CHECK(buf.count() == 2);
}

// -----------------------------------------------------------------------------
// Group 6: MT29F4G01Accessor erase semantics (mock transport)
// -----------------------------------------------------------------------------
TEST_CASE("MT29F4G01Accessor: erase calls correct block")
{
    MockSPITransport mock;
    MT29F4G01Accessor<MockSPITransport> acc(mock, 0);

    // Erase at address 0 should target block 0
    REQUIRE(acc.erase(0) == AccessorError::NO_ERROR);

    // The BLOCK_ERASE command is 0xD8
    bool found = false;
    for (uint8_t b : mock.log)
        if (b == 0xD8)
            found = true;

    CHECK(found);
}

// ============================================================================
//  SECTION 1 — RAM/NOR TESTS (DirectMemoryAccessor)
// ============================================================================

TEST_CASE("RAM/NOR: pop second entry, reconstruct → expect 1 entry")
{
    DirectMemoryAccessor acc(0x1000, 2048);
    acc.format();

    using Buf = ImageBuffer<DirectMemoryAccessor>;
    Buf buf(acc);

    // Write 3 entries
    for (int i = 0; i < 3; i++)
    {
        ImageMetadata m{};
        m.payload_size = 4;
        m.timestamp = static_cast<uint64_t>(100) + static_cast<uint64_t>(i);

        REQUIRE(buf.add_image(m) == ImageBufferError::NO_ERROR);

        uint8_t p[4] = {uint8_t(i), uint8_t(i+1), uint8_t(i+2), uint8_t(i+3)};
        REQUIRE(buf.add_data_chunk(p, 4) == ImageBufferError::NO_ERROR);
        REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
    }

    // Pop second entry
    ImageMetadata meta{};
    size_t chunk = 4;
    uint8_t tmp[4];

    // Pop entry 0
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    // Pop entry 1 (the "middle" entry)
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    chunk = 4;
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    // Reconstruct
    Buf buf2(acc);
    REQUIRE(buf2.initialize_from_flash() == ImageBufferError::NO_ERROR);

    CHECK(buf2.count() == 1);
}

// ============================================================================
//  SECTION 2 — NAND TESTS (ConfigurableMemoryAccessor)
// ============================================================================

static void write_entry(ImageBuffer<ConfigurableMemoryAccessor>& buf,
                        uint32_t ts,
                        size_t payload_size)
{
    ImageMetadata m{};
    m.payload_size = static_cast<uint32_t>(payload_size);
    m.timestamp = ts;

    REQUIRE(buf.add_image(m) == ImageBufferError::NO_ERROR);

    std::vector<uint8_t> p(payload_size);
    for (size_t i = 0; i < payload_size; i++)
        p[i] = uint8_t(i);

    REQUIRE(buf.add_data_chunk(p.data(), payload_size) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.push_image() == ImageBufferError::NO_ERROR);
}

TEST_CASE("NAND: pop second entry, reconstruct → expect 1 entry")
{
    ConfigurableMemoryAccessor acc(0x6000, 4096, 16);
    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;
    Buf buf(acc);

    write_entry(buf, 0x12123434, 4);
    write_entry(buf, 0x56567878, 4);
    write_entry(buf, 0x9A9ABCBC, 4);

    // printf("BEFORE ANY POP:\n");
    // dump_flash(acc.raw(), 320);

    // Pop entry 0
    ImageMetadata meta{};
    size_t chunk = 4;
    uint8_t tmp[4];
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    // printf("AFTER POP 0:\n");
    // dump_flash(acc.raw(), 320);

    // Pop entry 1
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    chunk = 4;
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    // printf("AFTER POP 1:\n");
    // dump_flash(acc.raw(), 320);

    // Reconstruct
    Buf buf2(acc);
    REQUIRE(buf2.initialize_from_flash() == ImageBufferError::NO_ERROR);

    CHECK(buf2.count() == 1);
}

// ============================================================================
//  SECTION 3 — NAND PAYLOAD SIZE EDGE CASES
// ============================================================================

TEST_CASE("NAND: payload ends inside block")
{
    ConfigurableMemoryAccessor acc(0x7000, 4096, 16);
    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;
    Buf buf(acc);

    write_entry(buf, 100, 3);   // ends well inside block
    write_entry(buf, 101, 4);

    // Pop first
    ImageMetadata meta{};
    size_t chunk = 3;
    uint8_t tmp[4];
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    Buf buf2(acc);
    REQUIRE(buf2.initialize_from_flash() == ImageBufferError::NO_ERROR);
    CHECK(buf2.count() == 1);
}

TEST_CASE("NAND: payload ends near block boundary")
{
    ConfigurableMemoryAccessor acc(0x7100, 4096, 16);
    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;
    Buf buf(acc);

    write_entry(buf, 100, 15);  // ends at offset 15 inside block
    write_entry(buf, 101, 4);

    // Pop first
    ImageMetadata meta{};
    size_t chunk = 15;
    uint8_t tmp[16];
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    Buf buf2(acc);
    REQUIRE(buf2.initialize_from_flash() == ImageBufferError::NO_ERROR);
    CHECK(buf2.count() == 1);
}

TEST_CASE("NAND: payload ends exactly at block boundary")
{
    ConfigurableMemoryAccessor acc(0x7200, 4096, 16);
    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;
    Buf buf(acc);

    write_entry(buf, 100, 16);  // ends exactly at block boundary
    write_entry(buf, 101, 4);

    // Pop first
    ImageMetadata meta{};
    size_t chunk = 16;
    uint8_t tmp[32];
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    Buf buf2(acc);
    REQUIRE(buf2.initialize_from_flash() == ImageBufferError::NO_ERROR);
    CHECK(buf2.count() == 1);
}

TEST_CASE("NAND: payload ends just past block boundary")
{
    ConfigurableMemoryAccessor acc(0x7300, 4096, 16);
    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;
    Buf buf(acc);

    write_entry(buf, 100, 17);  // spills into next block
    write_entry(buf, 101, 4);

    // Pop first
    ImageMetadata meta{};
    size_t chunk = 17;
    uint8_t tmp[32];
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    Buf buf2(acc);
    REQUIRE(buf2.initialize_from_flash() == ImageBufferError::NO_ERROR);
    CHECK(buf2.count() == 1);
}

TEST_CASE("NAND: payload spans multiple blocks")
{
    ConfigurableMemoryAccessor acc(0x7400, 4096, 16);
    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;
    Buf buf(acc);

    write_entry(buf, 100, 64);  // spans 4 blocks
    write_entry(buf, 101, 4);

    // Pop first
    ImageMetadata meta{};
    size_t chunk = 64;
    std::vector<uint8_t> tmp(64);
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.get_data_chunk(tmp.data(), chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    Buf buf2(acc);
    REQUIRE(buf2.initialize_from_flash() == ImageBufferError::NO_ERROR);
    CHECK(buf2.count() == 1);
}

// ============================================================================
//  SECTION 4 — ADD NEW ENTRY AFTER POP
// ============================================================================

TEST_CASE("NAND: add new entry after pop → reconstruction sees all remaining entries")
{
    ConfigurableMemoryAccessor acc(0x7500, 4096, 16);
    using Buf = ImageBuffer<ConfigurableMemoryAccessor>;
    Buf buf(acc);

    // Write 3 entries
    write_entry(buf, 100, 4);
    write_entry(buf, 101, 4);
    write_entry(buf, 102, 4);

    // Pop entry 0
    ImageMetadata meta{};
    size_t chunk = 4;
    uint8_t tmp[4];
    REQUIRE(buf.get_image(meta) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.get_data_chunk(tmp, chunk) == ImageBufferError::NO_ERROR);
    REQUIRE(buf.pop_image() == ImageBufferError::NO_ERROR);

    // Add new entry
    write_entry(buf, 200, 4);

    // Reconstruct
    Buf buf2(acc);
    REQUIRE(buf2.initialize_from_flash() == ImageBufferError::NO_ERROR);

    CHECK(buf2.count() == 3);
}
