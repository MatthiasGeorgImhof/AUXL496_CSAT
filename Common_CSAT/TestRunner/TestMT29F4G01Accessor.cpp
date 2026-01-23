#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "imagebuffer/MT29F4G01Accessor.hpp"
#include "Transport.hpp"
#include "imagebuffer/buffer_state.hpp"
#include "imagebuffer/image.hpp"
#include "ImageBuffer.hpp"

template <typename Accessor>
using CachedImageBuffer = ImageBuffer<Accessor>;

// ------------------------------------------------------------
// Mock SPI transport satisfying StreamAccessTransport
// ------------------------------------------------------------
class MockSPITransport
{
public:
    using config_type = struct
    {
        using mode_tag = stream_mode_tag;
    };

    bool write(const uint8_t *data, uint16_t len)
    {
        last_write.assign(data, data + len);
        return true;
    }

    bool read(uint8_t *data, uint16_t len)
    {
        for (uint16_t i = 0; i < len; i++)
            data[i] = static_cast<uint8_t>(0xA0 + i);
        return true;
    }

    std::vector<uint8_t> last_write;
};

static_assert(StreamAccessTransport<MockSPITransport>, "MockSPITransport must satisfy StreamAccessTransport");

static_assert(Accessor<MT29F4G01Accessor<MockSPITransport>>, "Accessor concept failed");

// ------------------------------------------------------------
// Test suite
// ------------------------------------------------------------
TEST_SUITE("MT29F4G01Accessor Template")
{
    TEST_CASE("Geometry constants sanity check")
    {
        using A = MT29F4G01Accessor<MockSPITransport>;

        CHECK(A::PAGE_SIZE == 4096);
        CHECK(A::SPARE_SIZE == 256);
        CHECK(A::PAGE_TOTAL_SIZE == 4352);

        CHECK(A::PAGES_PER_BLOCK == 64);
        CHECK(A::BLOCK_SIZE == 4352 * 64); // 278,528 bytes

        CHECK(A::TOTAL_BLOCKS == 2048);
        CHECK(A::TOTAL_SIZE == A::BLOCK_SIZE * A::TOTAL_BLOCKS);
    }

    TEST_CASE("Instantiation and Accessor concept compliance")
    {
        MockSPITransport spi;
        using A = MT29F4G01Accessor<MockSPITransport>;
        A acc(spi);

        CHECK(acc.getAlignment() == A::PAGE_SIZE);
        CHECK(acc.getFlashMemorySize() == A::TOTAL_SIZE);
        CHECK(acc.getFlashStartAddress() == 0);

        static_assert(Accessor<A>, "MT29F4G01Accessor must satisfy Accessor concept");
    }

    TEST_CASE("Logical to physical mapping sanity")
    {
        MockSPITransport spi;
        using A = MT29F4G01Accessor<MockSPITransport>;
        A acc(spi);

        // Address 0 → block 0, page 0, column 0
        auto p0 = acc.logicalToPhysical(0);
        CHECK(p0.block == 0);
        CHECK(p0.page_in_block == 0);
        CHECK(p0.column == 0);

        // End of first page
        auto p1 = acc.logicalToPhysical(A::PAGE_SIZE - 1);
        CHECK(p1.block == 0);
        CHECK(p1.page_in_block == 0);
        CHECK(p1.column == A::PAGE_SIZE - 1);

        // Start of page 1
        auto p2 = acc.logicalToPhysical(A::PAGE_SIZE);
        CHECK(p2.block == 0);
        CHECK(p2.page_in_block == 1);
        CHECK(p2.column == 0);

        // Start of block 1
        size_t logical_block_stride = A::PAGE_SIZE * A::PAGES_PER_BLOCK; // 4096 * 64 = 262,144

        auto p3 = acc.logicalToPhysical(logical_block_stride);
        CHECK(p3.block == 1);
        CHECK(p3.page_in_block == 0);
    }

    TEST_CASE("Read/write/erase behavior at bounds")
    {
        MockSPITransport spi;
        using A = MT29F4G01Accessor<MockSPITransport>;
        A acc(spi);

        std::array<uint8_t, 16> buf{};

        // Out-of-bounds still enforced
        CHECK(acc.read(A::TOTAL_SIZE, buf.data(), buf.size()) == AccessorError::OUT_OF_BOUNDS);

        CHECK(acc.write(A::TOTAL_SIZE, buf.data(), buf.size()) == AccessorError::OUT_OF_BOUNDS);

        CHECK(acc.erase(A::TOTAL_SIZE) == AccessorError::OUT_OF_BOUNDS);

        // In-bounds operations succeed with the current stubs
        CHECK(acc.read(0, buf.data(), buf.size()) == AccessorError::NO_ERROR);
        CHECK(acc.write(0, buf.data(), buf.size()) == AccessorError::NO_ERROR);
        CHECK(acc.erase(0) == AccessorError::NO_ERROR);
    }

    TEST_CASE("CachedImageBuffer instantiation")
    {
        MockSPITransport spi;
        using A = MT29F4G01Accessor<MockSPITransport>;
        A acc(spi);

        CachedImageBuffer<A> buffer(acc);

        CHECK(buffer.is_empty());
        CHECK(buffer.capacity() == A::TOTAL_SIZE);
    }
}
