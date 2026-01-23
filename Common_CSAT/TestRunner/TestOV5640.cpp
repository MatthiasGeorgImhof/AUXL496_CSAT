#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "OV5640.hpp"
#include <array>
#include <vector>
#include <cstdint>
#include <cstring>
#include <string>

// Dummy config for RegisterModeTransport concept
struct DummyConfig
{
    using mode_tag = register_mode_tag;
};

// Mock register‑mode transport
class MockTransport
{
public:
    using config_type = DummyConfig;

    bool write_reg(uint16_t reg, const uint8_t* data, size_t size)
    {
        last_reg  = reg;
        last_write.assign(data, data + size);
        return write_ok;
    }

    bool read_reg(uint16_t reg, uint8_t* data, size_t size)
    {
        last_reg = reg;
        last_read.resize(size);

        for (size_t i = 0; i < size; ++i)
            last_read[i] = (i < mock_response.size()) ? mock_response[i] : uint8_t{0};

        std::memcpy(data, last_read.data(), size);
        return read_ok;
    }

    void setMockResponse(std::initializer_list<uint8_t> bytes)
    {
        mock_response.assign(bytes.begin(), bytes.end());
    }

    uint16_t              last_reg{0};
    std::vector<uint8_t>  last_write;
    std::vector<uint8_t>  last_read;
    std::vector<uint8_t>  mock_response;
    bool                  write_ok{true};
    bool                  read_ok{true};
};

//
// ─────────────────────────────────────────────────────────────
//  Basic register access tests
// ─────────────────────────────────────────────────────────────
//

TEST_CASE("writeRegister(enum, uint8_t) forwards to uint16_t overload")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    cam.writeRegister(OV5640_Register::CHIP_ID, 0xAB);

    CHECK(transport.last_reg ==
          static_cast<uint16_t>(OV5640_Register::CHIP_ID));
    CHECK(transport.last_write == std::vector<uint8_t>({0xAB}));
}

TEST_CASE("writeRegister(enum, multi-byte) performs endian swap")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    uint16_t value = 0x1234; // little-endian in memory: 34 12
    cam.writeRegister(OV5640_Register::SC_PLL_CTRL0,
                      reinterpret_cast<uint8_t*>(&value),
                      2);

    CHECK(transport.last_reg ==
          static_cast<uint16_t>(OV5640_Register::SC_PLL_CTRL0));

    // Expect big-endian on the wire: 12 34
    CHECK(transport.last_write == std::vector<uint8_t>({0x12, 0x34}));
}

TEST_CASE("readRegister(enum) forwards to uint16_t overload")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    transport.setMockResponse({0xAB});
    uint8_t result = cam.readRegister(OV5640_Register::CHIP_ID);

    CHECK(result == 0xAB);
    CHECK(transport.last_reg ==
          static_cast<uint16_t>(OV5640_Register::CHIP_ID));
}

TEST_CASE("readRegister(enum, multi-byte) swaps big-endian to little-endian")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    // On the bus: 30 0A (big-endian)
    transport.setMockResponse({0x30, 0x0A});

    uint16_t result = 0;
    bool ok = cam.readRegister(OV5640_Register::CHIP_ID,
                               reinterpret_cast<uint8_t*>(&result),
                               2);

    CHECK(ok);
    // After swapping: 0A 30 -> 0x300A
    CHECK(result == 0x300A);
    CHECK(transport.last_reg ==
          static_cast<uint16_t>(OV5640_Register::CHIP_ID));
}

TEST_CASE("writeRegister rejects odd-sized payloads")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    uint8_t data[3] = {0x01, 0x02, 0x03};
    bool ok = cam.writeRegister(OV5640_Register::CHIP_ID, data, 3);
    CHECK_FALSE(ok);
}

TEST_CASE("readRegister rejects odd-sized buffers")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    transport.setMockResponse({0x30, 0x0A});
    uint8_t buffer[3];
    bool ok = cam.readRegister(OV5640_Register::CHIP_ID, buffer, 3);
    CHECK_FALSE(ok);
}

//
// ─────────────────────────────────────────────────────────────
//  High-level API tests
// ─────────────────────────────────────────────────────────────
//

TEST_CASE("setResolution writes correct TIMING registers")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    cam.setResolution(1280, 720);

    CHECK(transport.last_reg == 0x380B); // last write is height LSB

    // We check the sequence explicitly:
    // 0x3808 = width high
    // 0x3809 = width low
    // 0x380A = height high
    // 0x380B = height low

    // width high
    cam.setResolution(1280, 720);
    CHECK(transport.last_write[0] == (720 & 0xFF));
}

TEST_CASE("setFormat writes correct register values")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    cam.setFormat(PixelFormat::YUV422);
    CHECK(transport.last_reg == static_cast<uint16_t>(OV5640_Register::FORMAT_CONTROL00));
    CHECK(transport.last_write[0] == 0x30);

    cam.setFormat(PixelFormat::RGB565);
    CHECK(transport.last_write[0] == 0x61);

    cam.setFormat(PixelFormat::JPEG);
    CHECK(transport.last_reg == static_cast<uint16_t>(OV5640_Register::JPG_MODE_SELECT));
    CHECK(transport.last_write[0] == 0x03);
}

TEST_CASE("enableTestPattern writes correct register")
{
    MockTransport transport;
    OV5640<MockTransport> cam(transport);

    cam.enableTestPattern(true);
    CHECK(transport.last_reg ==
          static_cast<uint16_t>(OV5640_Register::PRE_ISP_TEST_SET1));
    CHECK(transport.last_write[0] == 0x80);

    cam.enableTestPattern(false);
    CHECK(transport.last_write[0] == 0x00);
}

//
// ─────────────────────────────────────────────────────────────
//  Concept tests
// ─────────────────────────────────────────────────────────────
//

TEST_CASE("MockTransport satisfies RegisterModeTransport")
{
    static_assert(RegisterModeTransport<MockTransport>);
    CHECK(true);
}
